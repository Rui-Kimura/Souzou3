import RPi.GPIO as GPIO
import time
import board
import adafruit_bno055
import numpy as np
import heapq
import math
import os
import json
from pmw3901 import PMW3901

from fastapi import FastAPI, Request
import uvicorn
import threading


# ======== 1. 設定と定数 ========
# --- マップ・グリッド設定 ---
GRID_SIZE_MM = 50.0      # 1マスの大きさ (mm)
ROBOT_WIDTH_MM = 400.0   # 筐体幅
ROBOT_DEPTH_MM = 350.0   # 筐体奥行
# 障害物膨張半径 (ロボットの半径 / グリッドサイズ) 
# 半径 約270mm / 50mm = 5.4 -> 切り上げ+余裕で 6マス
INFLATION_RADIUS = 6      

# --- モーター制御設定 ---
BASE_SPEED = 40.0
TURN_SPEED = 30.0
KP_DIST = -1.5  # 直進補正ゲイン
KP_TURN = -1.2  # 回転制御ゲイン
TURN_THRESHOLD_DEG = 3.0 # 回転停止許容誤差
DIST_THRESHOLD_MM = 40.0 # 目標点到達許容誤差

# --- ピン設定 (動作確認済みのもの) ---
L_EN = 19
IN1 = 21 # LEFT_F
IN2 = 20 # LEFT_B
R_EN = 26
IN3 = 16 # RIGHT_F
IN4 = 12 # RIGHT_B
FREQ = 100

# --- リニアアクチュエータ ピン ---
LINEAR_IN1 = 5
LINEAR_IN2 = 6

# --- センサー設定 ---
SENSOR_HEIGHT_MM = 95.0
PIXEL_TO_MM = 0.0017 * SENSOR_HEIGHT_MM
PROFILE_FILE = "bno_profile.json"

# ======== 2. マップ定義 ========

MAP_DATA_PATH = "room.dat"
RAW_MAP_DATA = []
# ファイルが存在しない場合のエラー回避用ダミーデータ（必要に応じて修正してください）
if os.path.exists(MAP_DATA_PATH):
    with open(MAP_DATA_PATH, "r") as f:
        RAW_MAP_DATA = [line.rstrip() for line in f]
else:
    # デフォルトのダミーマップ（テスト用）
    RAW_MAP_DATA = ["0"*20 for _ in range(20)]

target_grid = (10, 10, 0)

manual_control = False
manual_speed = 0.0
manual_direction = 0.0
manual_angle = 0.0
manual_liniar = 0

planner = None
# 排他制御用ロック (位置情報の更新と読み取りが競合しないようにする)
position_lock = threading.Lock()

# ======== 3. クラス定義 ========

class RobotState:
    def __init__(self, start_grid_x, start_grid_y, start_heading):
        # 現在位置 (mm単位, グローバル座標)
        self.x = start_grid_x * GRID_SIZE_MM
        self.y = start_grid_y * GRID_SIZE_MM
        # 現在の向き (度, 0=北/上, 時計回り正)
        self.heading = start_heading
        # グローバル座標系でのオフセット (起動時のセンサ値を0とするため)
        self.heading_offset = 0.0

    def update(self, bno_heading, pmw_dx, pmw_dy):
        """センサー値をもとに自己位置を更新"""
        if bno_heading is not None:
            # BNO055の生の値を、起動時を基準とした角度に変換
            # (必要に応じてマップの方位と合わせる処理をここに入れる)
            self.heading = bno_heading

        # PMW3901の移動量をmmに変換
        dist_fwd = pmw_dy * PIXEL_TO_MM # センサY軸 = ロボット前後
        dist_side = pmw_dx * PIXEL_TO_MM # センサX軸 = ロボット左右

        # ロボット座標系 -> グローバル座標系への回転変換
        # マップ座標系: X=右+, Y=下+ (画像座標)
        # 角度: 0=上(-Y), 90=右(+X), 180=下(+Y), 270=左(-X) と仮定
        rad = math.radians(self.heading)
        
        # 移動ベクトルを回転
        # global_dx = fwd * sin(theta) + side * cos(theta)
        # global_dy = -fwd * cos(theta) + side * sin(theta) (Y軸反転注意)
        
        # ここではシンプルに: 
        # Heading 0度(上)のとき: dyが負(減る), dxが変化なし
        # これを三角関数で表現
        dx_global = dist_fwd * math.sin(rad) + dist_side * math.cos(rad)
        dy_global = -(dist_fwd * math.cos(rad) - dist_side * math.sin(rad))

        self.x += dx_global
        self.y += dy_global

    def get_grid_pos(self):
        return int(self.x / GRID_SIZE_MM), int(self.y / GRID_SIZE_MM)

class PathPlanner:
    def __init__(self, raw_map, inflation_r):
        self.grid = self._parse_map(raw_map)
        self.height = len(self.grid)
        self.width = len(self.grid[0])
        self.inflation_r = inflation_r
        self.cost_map = self._create_cost_map()

    def _parse_map(self, raw_data):
        # 行の長さを揃えて2次元配列(0/1)にする
        max_len = max(len(row) for row in raw_data)
        grid = []
        for row_str in raw_data:
            # 足りない部分は '1' (壁) で埋める
            padded = row_str + '1' * (max_len - len(row_str))
            grid.append([int(c) for c in padded])
        return np.array(grid)

    def _create_cost_map(self):
        cost_map = self.grid.copy()
        rows, cols = cost_map.shape
        
        # 障害物('1')を探し、その周囲を埋める
        obstacles = np.argwhere(self.grid == 1)
        
        for r, c in obstacles:
            r_min = max(0, r - self.inflation_r)
            r_max = min(rows, r + self.inflation_r + 1)
            c_min = max(0, c - self.inflation_r)
            c_max = min(cols, c + self.inflation_r + 1)
            
            cost_map[r_min:r_max, c_min:c_max] = 1
            
        return cost_map

    def get_path_astar(self, start_grid, goal_grid):
        # A*アルゴリズムで経路探索
        start = tuple(start_grid)
        goal = tuple(goal_grid)

        if self.cost_map[start[1]][start[0]] == 1:
            print("警告: スタート地点が障害物(またはその付近)内です")
        if self.cost_map[goal[1]][goal[0]] == 1:
            print("エラー: ゴール地点が障害物内または到達不能エリアです")
            return None

        # (F値, (x, y))
        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self._reconstruct_path(came_from, current)

            # 上下左右4方向 (斜め移動なしの場合)
            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 範囲外チェック
                if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height):
                    continue
                # 障害物チェック
                if self.cost_map[neighbor[1]][neighbor[0]] == 1:
                    continue

                # コスト計算 (距離1)
                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    # ヒューリスティック: マンハッタン距離
                    f_score = tentative_g + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return None # 経路なし

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

# ======== 4. ハードウェア制御用関数 ========

# PWMインスタンス (グローバル)
p1, p2, p3, p4 = None, None, None, None

def setup_hardware():
    global p1, p2, p3, p4
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pins = [IN1, IN2, IN3, IN4, L_EN, R_EN, LINEAR_IN1, LINEAR_IN2]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    p1 = GPIO.PWM(IN1, FREQ); p2 = GPIO.PWM(IN2, FREQ)
    p3 = GPIO.PWM(IN3, FREQ); p4 = GPIO.PWM(IN4, FREQ)
    GPIO.output(L_EN, GPIO.HIGH)
    GPIO.output(R_EN, GPIO.HIGH)
    
    p1.start(0); p2.start(0); p3.start(0); p4.start(0)

def set_motor_speed(left_speed, right_speed, brake=False):
    if(brake == True):
        """緊急停止用ブレーキ"""
        p1.ChangeDutyCycle(100); p2.ChangeDutyCycle(100)
        p3.ChangeDutyCycle(100); p4.ChangeDutyCycle(100)
        return
    
    """左右のモーター速度を設定 (-100 ~ 100)"""
    l = max(-100, min(100, left_speed))
    r = max(-100, min(100, right_speed))
    
    # 左
    if l > 0: p1.ChangeDutyCycle(l); p2.ChangeDutyCycle(0)
    else: p1.ChangeDutyCycle(0); p2.ChangeDutyCycle(abs(l))
    # 右
    if r > 0: p3.ChangeDutyCycle(r); p4.ChangeDutyCycle(0)
    else: p3.ChangeDutyCycle(0); p4.ChangeDutyCycle(abs(r))

def get_bno_heading(sensor):
    """安定するまで何度かリトライして角度取得"""
    for _ in range(5):
        try:
            h = sensor.euler[0]
            if h is not None: return h
        except: pass
        time.sleep(0.005)
    return None

# ======== 5. メイン移動ロジック ========

def monitor_position(robot_instance, bno_sensor, pmw_sensor):
    """並列スレッドで常に位置情報を更新し続ける"""
    while True:
        # センサー読み取り
        h = get_bno_heading(bno_sensor)
        try:
            # get_motion()は呼び出すとレジスタがクリアされるため、ここで一度だけ呼ぶ
            dx, dy = pmw_sensor.get_motion()
        except:
            dx, dy = 0, 0
        
        # ロックを取得して更新
        with position_lock:
            robot_instance.update(h, dx, dy)
        
        # CPU負荷軽減のため少し待つ
        time.sleep(0.02)

def move_to_target(_planner, robot, sensor_bno, sensor_pmw, target_grid_pos):
    """現在地から目標グリッドまでの経路を計算して移動。target_grid_pos=(x, y, angle)"""
    
    # 目標情報 (x, y) と オプションの角度 (angle) を分離
    target_angle = None
    if len(target_grid_pos) == 3:
        target_x, target_y, target_angle = target_grid_pos
        goal_grid = (target_x, target_y)
    else:
        goal_grid = target_grid_pos

    # 1. 現在のグリッド座標を取得
    with position_lock:
        start_grid = robot.get_grid_pos()
    print(f"経路計画開始: {start_grid} -> {goal_grid} (Heading: {target_angle})")

    # 2. A*で経路計算
    path = _planner.get_path_astar(start_grid, goal_grid)
    
    if not path:
        print("エラー: 経路が見つかりません。")
        return False
    
    print(f"経路決定: {len(path)} ステップ")
    # path = [(x,y), (x,y), ...] 
    
    # 3. 経路の各ポイントを順に通過 (最初の点は現在地なのでスキップ)
    for i in range(1, len(path)):
        next_node = path[i]
        # 目標座標 (mm) 中心へ
        target_x_mm = next_node[0] * GRID_SIZE_MM + GRID_SIZE_MM/2
        target_y_mm = next_node[1] * GRID_SIZE_MM + GRID_SIZE_MM/2
        
        print(f"WayPoint {i}/{len(path)-1}: Grid{next_node}を目ざします")

        # --- ウェイポイント到達ループ ---
        while True:
            # センサー更新は monitor_position スレッドで行っている
            
            # ロックをして最新の座標を取得
            with position_lock:
                current_x = robot.x
                current_y = robot.y
                current_heading = robot.heading

            # 目標までの距離と角度を計算
            dx_global = target_x_mm - current_x
            dy_global = target_y_mm - current_y
            distance = math.sqrt(dx_global**2 + dy_global**2)
            
            # 到達判定
            if distance < DIST_THRESHOLD_MM:
                print("WayPoint 到達")
                set_motor_speed(0, 0)
                break

            # 目標方位 (atan2は math.atan2(y, x))
            target_angle_rad = math.atan2(dx_global, -dy_global) 
            target_angle_deg = math.degrees(target_angle_rad)
            if target_angle_deg < 0: target_angle_deg += 360

            # 回転必要量
            heading_diff = (target_angle_deg - current_heading + 180) % 360 - 180

            # --- 制御ロジック ---
            # 1. 角度ズレが大きい場合は、その場で回転
            if abs(heading_diff) > 20:
                turn_pow = KP_TURN * heading_diff
                # 最低出力確保
                if turn_pow > 0: turn_pow = max(turn_pow, 25)
                else: turn_pow = min(turn_pow, -25)
                
                set_motor_speed(-turn_pow, turn_pow) # 左回転: 左-, 右+
            
            # 2. 向きが合っていれば直進 + 角度微調整
            else:
                correction = heading_diff * KP_DIST
                l_speed = BASE_SPEED - correction
                r_speed = BASE_SPEED + correction
                set_motor_speed(l_speed, r_speed)
            
            time.sleep(0.02)

    print("目標地点座標に到着しました。")
    set_motor_speed(0, 0)

    # 4. 最終角度への回転 (指定がある場合)
    if target_angle is not None:
        print(f"最終角度調整: {target_angle}度 へ回転します")
        while True:
            with position_lock:
                current_heading = robot.heading
            
            heading_diff = (target_angle - current_heading + 180) % 360 - 180
            
            if abs(heading_diff) <= TURN_THRESHOLD_DEG:
                print("最終角度到達")
                set_motor_speed(0, 0)
                break
            
            turn_pow = KP_TURN * heading_diff
            if turn_pow > 0: turn_pow = max(turn_pow, 25)
            else: turn_pow = min(turn_pow, -25)
            set_motor_speed(-turn_pow, turn_pow)
            time.sleep(0.02)

    return True

def move_linear(status):
    if(status==1):  #上昇
        GPIO.output(LINEAR_IN1, GPIO.HIGH)
        GPIO.output(LINEAR_IN2, GPIO.LOW)
    elif(status==-1): #下降
        GPIO.output(LINEAR_IN1, GPIO.LOW)
        GPIO.output(LINEAR_IN2, GPIO.HIGH)
    else: #STOP
        GPIO.output(LINEAR_IN1, GPIO.LOW)
        GPIO.output(LINEAR_IN2, GPIO.LOW)

# localhost api
app = FastAPI()

robot = None

@app.middleware("http")
async def add_my_headers(request: Request, call_next):
    response = await call_next(request)
    response.headers["Access-Control-Allow-Origin"] = "http://localhost:3000"
    return response
@app.get("/")
async def root():
    return {"message":"Hello World"}
@app.get("/position")
async def position():
    # API呼び出し時もロックを使って安全に読み取る
    with position_lock:
        if robot:
            return {"x":robot.x,"y":robot.y}
        return {"x":0, "y":0}

@app.get("/mapdata")
async def mapdata():
    return{"mapdata":RAW_MAP_DATA}

@app.get("/costmapdata")
async def costmapdata():
    # 変更点: NumPy配列をリストに変換してJSONシリアライズ可能にする
    if planner and hasattr(planner, 'cost_map'):
        return {"mapdata": planner.cost_map.tolist()}
    return {"mapdata": []}

@app.get("/target_point")
async def target_point():
    return{
        "x" : target_grid[0],
        "y" : target_grid[1],
        "angle" : target_grid[2]
    }

@app.get("/controll_api")
def control_api(
    direction: int, 
    speed: float,      
    angle: float       
):
    global manual_control, manual_speed, manual_direction, manual_angle
    manual_direction = direction
    manual_speed = speed / 5.0
    manual_angle = angle
    print(f"受信データ: direction={direction}, speed={manual_speed}, angle={angle}")
    return {
        "status": "controlled"
    }

@app.get("/set_manual_mode")
def manual_mode(mode:bool):
    global manual_control
    manual_control = bool(mode)
    return{
        "manual_control": manual_control
    }

@app.get("/linear")
def linear_move(mode:str):
    global manual_liniar
    if(mode=="up"):
        manual_liniar = 1
    elif(mode=="down"):
        manual_liniar = -1
    else: #STOP
        manual_liniar = 0
    return{
        "linear": manual_liniar
    }


def main():
    try:
        setup_hardware()
        i2c = board.I2C()
        bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
        if os.path.exists(PROFILE_FILE):
            with open(PROFILE_FILE, "r") as f:
                saved_offsets = json.load(f)
            bno.offsets_accelerometer = tuple(saved_offsets["accel"])
            bno.offsets_gyroscope = tuple(saved_offsets["gyro"])
            bno.offsets_magnetometer = tuple(saved_offsets["mag"])
            time.sleep(0.05)
            print("BNOプロファイルを読み込みました。")
        else:
            print(f"警告: {PROFILE_FILE} が見つかりません。")
        
        bno.mode = adafruit_bno055.NDOF_MODE
        time.sleep(1)
        
        pmw = PMW3901()

        start_heading = get_bno_heading(bno)
        if start_heading is None:
            print("BNOエラー")
            return

        start_x_grid = 7
        start_y_grid = 7
        
        global robot
        robot = RobotState(start_x_grid, start_y_grid, start_heading)
        global planner
        planner = PathPlanner(RAW_MAP_DATA, inflation_r=INFLATION_RADIUS)
        
        # --- 位置監視用スレッドの開始 ---
        monitor_thread = threading.Thread(target=monitor_position, args=(robot, bno, pmw), daemon=True)
        monitor_thread.start()
        print("位置監視システムを開始しました。")

        def run_api():
            uvicorn.run(app, host="0.0.0.0", port=8100, log_level="debug")
        
        api_thread = threading.Thread(target=run_api, daemon=True)
        api_thread.start()
        
        # target_gridは (x, y) または (x, y, angle) で指定
        # global target_grid = (10, 10, 180) 

        #print(f"現在地: ({robot.x:.1f}, {robot.y:.1f}) Heading:{robot.heading:.1f}")
        
        # 移動開始
        # move_to_target(planner, robot, bno, pmw, target_grid)
        
        OP_QUEUE = []
        while True: #主ループ
            EB = False
            # 手動コントロール用
            while(manual_control):
                if(manual_direction != 0):
                    if(manual_speed < 0):
                        manual_left = 0
                        manual_right = 0
                        EB = True
                    elif(manual_direction == 1 or manual_direction == -1):
                        EB = False
                        if(manual_angle > 0): #右曲がり
                            manual_left = manual_direction * manual_speed * (100 - manual_angle)
                            manual_right = manual_direction * manual_speed * 100
                        elif(manual_angle <= 0): #左曲がり
                            manual_left = manual_direction * manual_speed * 100
                            manual_right = manual_direction * manual_speed * (100 + manual_angle)
                        #END IF
                    #END IF
                else:
                    manual_right = 0
                    manual_left = 0
                #END IF

                set_motor_speed(manual_left, manual_right,EB)
                move_linear(manual_liniar)
                continue
            #END WHILE(manual_control)
            time.sleep(0.05)
            continue
        #END WHILE(True)
    #END TRY


    except KeyboardInterrupt:
        print("\n停止")
    finally:
        set_motor_speed(0, 0)
        GPIO.cleanup()

if __name__ == "__main__":
    main()