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
import serial
import serial.tools.list_ports

from fastapi import FastAPI, Request
import uvicorn
import threading
import queue

# --- マップ・グリッド設定 ---
GRID_SIZE_MM = 50.0      # 1マスの大きさ (mm)
ROBOT_WIDTH_MM = 400.0   # 筐体幅
ROBOT_DEPTH_MM = 350.0   # 筐体奥行
# 障害物膨張半径 (ロボットの半径 / グリッドサイズ) 
INFLATION_RADIUS = 6      

# --- モーター制御設定 ---
BASE_SPEED = 80.0
KP_DIST = 1.5  # 直進補正ゲイン
KP_TURN = 1.2  # 回転制御ゲイン
TURN_THRESHOLD_DEG = 3.0 # 回転停止許容誤差
DIST_THRESHOLD_MM = 40.0 # 目標点到達許容誤差

# --- ピン設定 ---
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
if os.path.exists(MAP_DATA_PATH):
    with open(MAP_DATA_PATH, "r") as f:
        RAW_MAP_DATA = [line.rstrip() for line in f]
else:
    # ダミーマップ（テスト用）
    RAW_MAP_DATA = ["0"*20 for _ in range(20)]

# 変更: Grid座標ではなく物理座標(mm)の初期値に変更 (例: x=1000mm, y=1000mm)
target_pose = (1000.0, 1000.0, 0.0)


manual_control = False
manual_speed = 0.0
manual_direction = 0.0
manual_angle = 0.0
manual_rotate = False
manual_liniar = 0

move_queue = queue.Queue()
#----- 動作キュー -----
AUTOMOVE = 1

planner = None
# 排他制御用ロック (位置情報の更新と読み取りが競合しないようにする)
position_lock = threading.Lock()



# ---  ArduinoController ---
class ArduinoController:
    def __init__(self, baudrate=9600):
        self.ser = None
        self.baudrate = baudrate
        self.port_name = self._find_arduino_port()
        
        if self.port_name:
            self._connect()
        else:
            print("Arduinoが見つかりません。")
            self.ser = None

    def _find_arduino_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if "Arduino" in p.description or "Uno" in p.description:
                return p.device
        return None

    def _connect(self):
        try:
            print(f"Arduinoを {self.port_name} で検出。接続中...")
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1)
            time.sleep(2)
            
        except serial.SerialException as e:
            print(f"Arduino接続エラー: {e}")
            self.ser = None

    def send_command(self, command):
        if self.ser and self.ser.is_open:
            try:
                data = command.encode('utf-8')
                self.ser.write(data)
                self.ser.flush() 
                print(f">> Arduinoへコマンド '{command}' を送信しました")
            except serial.SerialException as e:
                print(f"送信エラー: {e}")
        else:
            print(f"エラー: Arduinoが接続されていないため '{command}' を送信できません")

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Arduino接続を終了しました。")


class RobotState:
    def __init__(self, start_x, start_y, start_heading):
        """
        初期化: 物理座標 (mm) で受け取る
        """
        self.x = float(start_x)
        self.y = float(start_y)
        self.heading = start_heading
        self.heading_offset = 0.0

    def update(self, bno_heading, pmw_dx, pmw_dy):
        """センサー値をもとに自己位置を更新"""
        if bno_heading is not None:
            self.heading = bno_heading

        dist_fwd = pmw_dy * PIXEL_TO_MM
        dist_side = pmw_dx * PIXEL_TO_MM

        rad = math.radians(self.heading)
        
        dx_global = dist_fwd * math.sin(rad) + dist_side * math.cos(rad)
        dy_global = -(dist_fwd * math.cos(rad) - dist_side * math.sin(rad))

        self.x += dx_global
        self.y += dy_global

    def get_grid_pos(self):
        """マップ操作用: 現在の物理座標をグリッド座標に変換して返す"""
        return int(self.x / GRID_SIZE_MM), int(self.y / GRID_SIZE_MM)


class PathPlanner:
    def __init__(self, raw_map, inflation_r):
        self.grid = self._parse_map(raw_map)
        self.height = len(self.grid)
        self.width = len(self.grid[0])
        self.inflation_r = inflation_r
        self.cost_map = self._create_cost_map()

    def _parse_map(self, raw_data):
        max_len = max(len(row) for row in raw_data)
        grid = []
        for row_str in raw_data:
            padded = row_str + '1' * (max_len - len(row_str))
            grid.append([int(c) for c in padded])
        return np.array(grid)

    def _create_cost_map(self):
        cost_map = self.grid.copy()
        rows, cols = cost_map.shape
        
        obstacles = np.argwhere(self.grid == 1)
        
        for r, c in obstacles:
            r_min = max(0, r - self.inflation_r)
            r_max = min(rows, r + self.inflation_r + 1)
            c_min = max(0, c - self.inflation_r)
            c_max = min(cols, c + self.inflation_r + 1)
            
            cost_map[r_min:r_max, c_min:c_max] = 1
            
        return cost_map

    def get_path_astar(self, start_grid, goal_grid):
        start = tuple(start_grid)
        goal = tuple(goal_grid)

        if not (0 <= start[0] < self.width and 0 <= start[1] < self.height):
             print(f"エラー: スタート地点 {start} がマップ範囲外です")
             return None
        if not (0 <= goal[0] < self.width and 0 <= goal[1] < self.height):
             print(f"エラー: ゴール地点 {goal} がマップ範囲外です")
             return None

        if self.cost_map[start[1]][start[0]] == 1:
            print("警告: スタート地点が障害物(またはその付近)内です")
        if self.cost_map[goal[1]][goal[0]] == 1:
            print("エラー: ゴール地点が障害物内または到達不能エリアです")
            return None

        open_set = []
        heapq.heappush(open_set, (0, start))
        
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            current = heapq.heappop(open_set)[1]

            if current == goal:
                return self._reconstruct_path(came_from, current)

            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height):
                    continue
                if self.cost_map[neighbor[1]][neighbor[0]] == 1:
                    continue

                tentative_g = g_score[current] + 1
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f_score = tentative_g + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                    heapq.heappush(open_set, (f_score, neighbor))
                    
        return None

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path


# PWMインスタンス
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

def set_motor_speed(left_speed, right_speed, rotate=False, brake=False,):
    if(brake == True):
        p1.ChangeDutyCycle(100); p2.ChangeDutyCycle(100)
        p3.ChangeDutyCycle(100); p4.ChangeDutyCycle(100)
        return
    
    l = max(-100, min(100, left_speed))
    r = max(-100, min(100, right_speed))
    
    # 左
    if l > 0: p1.ChangeDutyCycle(l); p2.ChangeDutyCycle(0)
    else: p1.ChangeDutyCycle(0); p2.ChangeDutyCycle(abs(l))
    # 右
    if r > 0: p3.ChangeDutyCycle(r); p4.ChangeDutyCycle(0)
    else: p3.ChangeDutyCycle(0); p4.ChangeDutyCycle(abs(r))

def get_bno_heading(sensor):
    for _ in range(5):
        try:
            h = sensor.euler[0]
            if h is not None: return h
        except: pass
        time.sleep(0.005)
    return None

def monitor_position(robot_instance, bno_sensor, pmw_sensor):
    while True:
        h = get_bno_heading(bno_sensor)
        try:
            dx, dy = pmw_sensor.get_motion()
        except:
            dx, dy = 0, 0
        
        with position_lock:
            robot_instance.update(h, dx, dy)
        
        time.sleep(0.02)

# ======== 変更箇所: target_pos_mm (物理座標) を受け取るように変更 ========
def move_to_target(_planner, robot, sensor_bno, sensor_pmw, target_pos_mm):
    """
    現在地から目標物理座標までの経路を計算して移動。
    target_pos_mm = (x_mm, y_mm, angle)
    """
    # 物理座標(mm) を Grid座標(index) に変換して A* に渡す
    target_x_mm, target_y_mm, target_angle = target_pos_mm
    
    goal_grid_x = int(target_x_mm / GRID_SIZE_MM)
    goal_grid_y = int(target_y_mm / GRID_SIZE_MM)
    goal_grid = (goal_grid_x, goal_grid_y)

    with position_lock:
        start_grid = robot.get_grid_pos()
    
    print(f"経路計画開始: Start Grid{start_grid} -> Goal Grid{goal_grid} (Target mm: {target_x_mm:.1f}, {target_y_mm:.1f})")

    path = _planner.get_path_astar(start_grid, goal_grid)
    
    if not path:
        print("エラー: 経路が見つかりません。")
        return False
    
    print(f"経路決定: {len(path)} ステップ")
    
    # 経路の各ポイントを順に通過
    for i in range(1, len(path)):
        next_node = path[i]
        
        # 基本的にはグリッドの中心を目指す
        next_target_x_mm = next_node[0] * GRID_SIZE_MM + GRID_SIZE_MM/2
        next_target_y_mm = next_node[1] * GRID_SIZE_MM + GRID_SIZE_MM/2
        
        # ★最終ステップのみ、ユーザー指定の正確な座標を目指すように上書き
        if i == len(path) - 1:
            next_target_x_mm = target_x_mm
            next_target_y_mm = target_y_mm
        
        print(f"WayPoint {i}/{len(path)-1}: {next_node} (mm: {next_target_x_mm:.1f}, {next_target_y_mm:.1f}) を目指します")

        # --- ウェイポイント到達ループ ---
        while True:            
            with position_lock:
                current_x = robot.x
                current_y = robot.y
                current_heading = robot.heading

            dx_global = next_target_x_mm - current_x
            dy_global = next_target_y_mm - current_y
            distance = math.sqrt(dx_global**2 + dy_global**2)
            
            if distance < DIST_THRESHOLD_MM:
                print("WayPoint 到達")
                set_motor_speed(0, 0)
                break

            target_angle_rad = math.atan2(dx_global, -dy_global) 
            target_angle_deg = math.degrees(target_angle_rad)
            if target_angle_deg < 0: target_angle_deg += 360

            heading_diff = (target_angle_deg - current_heading + 180) % 360 - 180

            print(f"Cur: {current_heading:.1f} | Tgt: {target_angle_deg:.1f} | Diff: {heading_diff:.1f} | Dist: {distance:.1f}")

            if abs(heading_diff) > 20:
                turn_pow = KP_TURN * heading_diff
                if turn_pow > 0: turn_pow = max(turn_pow, 25)
                else: turn_pow = min(turn_pow, -25)
                set_motor_speed(-turn_pow, turn_pow) 
            else:
                correction = heading_diff * KP_DIST
                l_speed = BASE_SPEED - correction
                r_speed = BASE_SPEED + correction
                set_motor_speed(l_speed, r_speed)
            
            time.sleep(0.02)

    print("目標地点座標に到着しました。")
    set_motor_speed(0, 0)

    # 最終角度への回転 
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
        GPIO.output(LINEAR_IN1, GPIO.LOW)
        GPIO.output(LINEAR_IN2, GPIO.HIGH)
    elif(status==-1): #下降
        GPIO.output(LINEAR_IN1, GPIO.HIGH)
        GPIO.output(LINEAR_IN2, GPIO.LOW)
    else: #STOP
        GPIO.output(LINEAR_IN1, GPIO.LOW)
        GPIO.output(LINEAR_IN2, GPIO.LOW)

app = FastAPI()

robot = None
arduino = ArduinoController()

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
    with position_lock:
        if robot:
            return {"x":robot.x,"y":robot.y,"angle":robot.heading}
        return {"x":0, "y":0,"angle":0}

@app.get("/mapdata")
async def mapdata():
    return{"mapdata":RAW_MAP_DATA}

@app.get("/costmapdata")
async def costmapdata():
    if planner and hasattr(planner, 'cost_map'):
        return {"mapdata": planner.cost_map.tolist()}
    return {"mapdata": []}

@app.get("/target_point")
async def target_point():
    return{
        "x" : target_pose[0],
        "y" : target_pose[1],
        "angle" : target_pose[2]
    }

@app.get("/set_target_point")
async def set_target_point(
    x: float,
    y:float,
    angle:float,
):
    global target_pose
    # ユーザーからは物理座標 (mm) が送られてくると想定してそのまま格納
    target_pose = (x, y, angle)
    return{
        "x":x,
        "y":y,
        "angle":angle
    }

@app.get("/automove_start")
async def automove_start():
    move_queue.put(AUTOMOVE)
    return{
        "auto":"started"
    }

@app.get("/controll_api")
def control_api(
    direction: int, 
    speed: float,      
    angle: float,
    rotate: bool  
):
    global manual_control, manual_speed, manual_direction, manual_angle, manual_rotate
    manual_direction = direction
    manual_speed = speed / 5.0
    manual_angle = angle
    manual_rotate = rotate
    print(f"受信データ: direction={direction}, speed={manual_speed}, angle={angle}, rotate={rotate}")
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

@app.get("/slide")
def slide_move(mode:str):
    print(f"Slide Command Received: {mode}")
    
    if mode == "open":
        arduino.send_command('o')
    elif mode == "close":
        arduino.send_command('c')
    else:
        print(f"Unknown slide mode: {mode}")

    return {"slide": mode}


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
        
        # Grid座標から物理座標(mm)へ変換して渡す
        start_x_mm = start_x_grid * GRID_SIZE_MM
        start_y_mm = start_y_grid * GRID_SIZE_MM
        
        global robot
        robot = RobotState(start_x_mm, start_y_mm, start_heading)

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
        
        OP_QUEUE = []
        while True: #主ループ
            EB = False
            # 手動コントロール用
            while(manual_control):
                if(manual_direction != 0 and not manual_rotate):
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
                elif(manual_direction != 0 and manual_rotate): #超信地旋回
                    EB = False
                    manual_right = manual_direction * manual_speed * manual_angle
                    manual_left = -1 * manual_right
                else:
                    manual_right = 0
                    manual_left = 0
                #END IF

                set_motor_speed(manual_left, manual_right,False,EB)
                move_linear(manual_liniar)
                time.sleep(0.05)
                continue
            #END WHILE(manual_control)
            time.sleep(0.05)
            
            if(move_queue.qsize() > 0):
                buf_queue = move_queue.get()
                print("queue now")
                print(move_queue.qsize())
                match buf_queue:
                    case  int(x) if buf_queue == AUTOMOVE:
                        # 修正: グローバル変数の名前変更に対応
                        print("auto move")
                        move_to_target(planner, robot, bno, pmw, target_pose)
                    #END CASE AUTOMOVE

            #END IF buf_queue
        #END WHILE(True)
    #END TRY


    except KeyboardInterrupt:
        print("\n停止")
    finally:
        set_motor_speed(0, 0)
        GPIO.cleanup()
        arduino.close()

if __name__ == "__main__":
    main()