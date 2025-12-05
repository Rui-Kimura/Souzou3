from fastapi import FastAPI, Request
import uvicorn
import threading
import time
import sys
import math
import numpy as np
import heapq
import random
import serial
import serial.tools.list_ports

# --- クラス定義: ArduinoController (グローバル変数の前に定義する必要があります) ---
class ArduinoController:
    def __init__(self, baudrate=9600):
        self.ser = None
        self.baudrate = baudrate
        self.port_name = self._find_arduino_port()
        
        if self.port_name:
            self._connect()
        else:
            # サーバーを落とさないために、見つからない場合は警告のみにする
            print("【警告】Arduinoが見つかりません。シリアル通信機能は無効化されます。")
            self.ser = None

    def _find_arduino_port(self):
        """Arduinoポートを自動検出する内部メソッド"""
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            # Arduino Uno または 互換機の特徴で検索
            if "Arduino" in p.description or "Uno" in p.description:
                return p.device
            # 必要に応じてVID:PID検索を追加
            # if "2341:0043" in p.hwid: return p.device
        return None

    def _connect(self):
        """シリアル接続を確立し、初期リセット待機を行う"""
        try:
            print(f"Arduinoを {self.port_name} で検出。接続中...")
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1)
            
            # 【重要】ここでのみ2秒待つ（初回接続時のリセット対策）
            print("通信安定化のため2秒待機します...")
            time.sleep(2)
            print("Arduino接続完了。準備OKです。")
            
        except serial.SerialException as e:
            print(f"Arduino接続エラー: {e}")
            self.ser = None

    def send_command(self, command):
        """
        待機時間なしで即座にコマンドを送信するメソッド
        """
        if self.ser and self.ser.is_open:
            try:
                # 送信データ作成
                data = command.encode('utf-8')
                self.ser.write(data)
                self.ser.flush() # 即時送信を確定させる
                print(f">> Arduinoへコマンド '{command}' を送信しました")
            except serial.SerialException as e:
                print(f"送信エラー: {e}")
                # 切断されていた場合、再接続を試みるロジックを入れることも可能
        else:
            print(f"エラー: Arduinoが接続されていないため '{command}' を送信できません")

    def close(self):
        """終了時にポートを閉じる"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Arduino接続を終了しました。")


# --- グローバル変数と定数 ---
# マップデータ
MAP_DATA_PATH = "../room.dat"

planner  = None

# ここでArduinoControllerを初期化 (クラス定義の後である必要がある)
arduino = ArduinoController()

# マップデータの読み込み (ファイルがない場合のフォールバック付き)
try:
    with open(MAP_DATA_PATH, 'r') as f:
        RAW_MAP_DATA = [line.strip() for line in f.readlines()]
except FileNotFoundError:
    print(f"警告: {MAP_DATA_PATH} が見つかりません。ダミーマップを使用します。")
    RAW_MAP_DATA = [
        "11111111111111111111",
        "10000000000000000001",
        "10000000000000000001",
        "10001111111111000001",
        "10001111111111000001",
        "10000000000000000001",
        "10000000000000000001",
        "11111111111111111111"
    ]

# 現在位置と角度（グローバル変数としてアクセス）
x = 75.0  
y = 75.0
angle = 0.0 # 0度 = 右向き (時計回り)

goal_grid = None  # 目標地点 (グリッド座標 x, y, angle)

# 移動パラメータ
SPEED = 10.0            # 1ステップあたりの移動量 (mm)
TURN_RATE = 15.0        # 1ステップあたりの回転量 (度)
UPDATE_INTERVAL = 0.05  # 更新間隔 (秒)
TILE_WIDTH = 50.0       # マス目一つの大きさ (mm)

# A*用パラメータ: 壁からの安全マージン（インフレーション半径）
INFLATION_RADIUS = 2

# マップの寸法
MAP_HEIGHT = len(RAW_MAP_DATA)
MAP_MAX_WIDTH = max(len(row) for row in RAW_MAP_DATA)

# グローバル変数を保護するためのロック
position_lock = threading.Lock()
manual_control = False
manual_speed = 0.0
manual_direction = 0
manual_angle = 0.0


# --- クラス定義: A* PathPlanner ---
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
            # '1'は壁、それ以外('0')は通路とみなす
            grid.append([1 if c == '1' else 0 for c in padded])
        return np.array(grid)

    def _create_cost_map(self):
        cost_map = self.grid.copy()
        rows, cols = cost_map.shape
        
        # 障害物('1')を探し、その周囲を埋める (Inflation)
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
        
        # 範囲外チェック
        if not (0 <= start[0] < self.width and 0 <= start[1] < self.height):
            return None
        if not (0 <= goal[0] < self.width and 0 <= goal[1] < self.height):
            return None

        # スタート地点が障害物内の場合のリカバリ
        if self.cost_map[start[1]][start[0]] == 1:
            print("Start point is in obstacle. Searching nearby...")
            found = False
            # 周囲2マスを探索
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    nx, ny = start[0]+dx, start[1]+dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                         if self.cost_map[ny][nx] == 0:
                            start = (nx, ny)
                            found = True
                            break
                if found: break
            if not found: return None

        if self.cost_map[goal[1]][goal[0]] == 1:
            # Goalが障害物なら到達不能
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

            # 上下左右4方向
            neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0)]
            
            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)
                
                # 範囲外チェック
                if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height):
                    continue
                # 障害物チェック
                if self.cost_map[neighbor[1]][neighbor[0]] == 1:
                    continue

                # コスト計算
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

# --- マップ判定ヘルパー関数 ---
def is_wall(check_x, check_y):
    tile_col = int(check_x // TILE_WIDTH)
    tile_row = int(check_y // TILE_WIDTH)

    if not (0 <= tile_row < MAP_HEIGHT): return True
    
    row_len = len(RAW_MAP_DATA[tile_row])
    if not (0 <= tile_col < row_len): return True

    return RAW_MAP_DATA[tile_row][tile_col] == '1'

def move_player_in_background():
    """
    A*アルゴリズムを用いてランダムな地点へ移動し続ける
    """
    global x, y, angle
    
    # 経路計画クラスの初期化
    global planner
    planner = PathPlanner(RAW_MAP_DATA, INFLATION_RADIUS)
    print("シミュレーションエンジン: 起動完了")

    while True:
        # 手動モード中は自動移動ロジックをバイパス
        if manual_control:
            # 手動操作ロジック (簡易実装)
            if manual_speed != 0 or manual_direction != 0:
                 with position_lock:
                    # 旋回
                    if manual_angle != 0:
                         angle = (angle + manual_angle * 0.5) % 360
                    
                    # 前進/後退
                    current_speed = manual_speed
                    if manual_direction == -1: current_speed *= -1
                    
                    rad = math.radians(angle)
                    new_x = x + current_speed * math.cos(rad)
                    new_y = y + current_speed * math.sin(rad)
                    
                    # 壁判定
                    if not is_wall(new_x, new_y):
                        x, y = new_x, new_y
            
            time.sleep(UPDATE_INTERVAL)
            continue

        # --- 自動運転ロジック ---
        start_grid = (int(x // TILE_WIDTH), int(y // TILE_WIDTH))
        
        global goal_grid
        target_xy = None
        target_angle = 0.0

        for _ in range(100): # 最大100回試行
            gy = random.randint(0, planner.height - 1)
            gx = random.randint(0, planner.width - 1)
            # コストマップ上で0（安全）な場所を選ぶ
            if planner.cost_map[gy][gx] == 0:
                if abs(gx - start_grid[0]) + abs(gy - start_grid[1]) > 5:
                    target_xy = (gx, gy)
                    target_angle = float(random.randint(0, 360))
                    goal_grid = (gx, gy, target_angle)
                    break
        
        if target_xy is None:
            # print("有効なゴールが見つかりませんでした。再探索します...")
            time.sleep(1)
            continue

        # print(f"新しい目的地を設定: {start_grid} -> {target_xy} (最終角度: {target_angle})")
        path = planner.get_path_astar(start_grid, target_xy)
        
        if not path:
            time.sleep(1)
            continue
            
        # 4. 経路追従ループ
        path_interrupted = False
        
        for i in range(1, len(path)):
            if manual_control: 
                path_interrupted = True
                break
                
            next_node = path[i]
            target_x = next_node[0] * TILE_WIDTH + TILE_WIDTH / 2.0
            target_y = next_node[1] * TILE_WIDTH + TILE_WIDTH / 2.0
            
            while True:
                if manual_control: 
                    path_interrupted = True
                    break
                
                with position_lock:
                    dx = target_x - x
                    dy = target_y - y
                    dist = math.sqrt(dx**2 + dy**2)
                    
                    if dist < SPEED:
                        x = target_x
                        y = target_y
                        break 
                    
                    target_angle_rad = math.atan2(dy, dx)
                    target_angle_deg = math.degrees(target_angle_rad)
                    if target_angle_deg < 0: target_angle_deg += 360
                    
                    angle_diff = (target_angle_deg - angle + 360) % 360
                    if angle_diff > 180: angle_diff -= 360
                    
                    if abs(angle_diff) > TURN_RATE:
                        turn = TURN_RATE if angle_diff > 0 else -TURN_RATE
                        angle = (angle + turn) % 360
                    else:
                        angle = target_angle_deg
                        move_rad = math.radians(angle)
                        x += SPEED * math.cos(move_rad)
                        y += SPEED * math.sin(move_rad)
                
                time.sleep(UPDATE_INTERVAL)
            
            if path_interrupted: break
        
        # 5. 最終地点での角度調整
        if not path_interrupted:
            while True:
                if manual_control:
                    path_interrupted = True
                    break
                
                with position_lock:
                    angle_diff = (target_angle - angle + 360) % 360
                    if angle_diff > 180: angle_diff -= 360
                    
                    if abs(angle_diff) < TURN_RATE:
                        angle = target_angle
                        break
                    
                    turn = TURN_RATE if angle_diff > 0 else -TURN_RATE
                    angle = (angle + turn) % 360
                
                time.sleep(UPDATE_INTERVAL)

        if not path_interrupted:
            time.sleep(2) 

app = FastAPI()

@app.on_event("startup")
def startup_event():
    thread = threading.Thread(target=move_player_in_background, daemon=True)
    thread.start()

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
        return {"x": round(x), "y": round(y), "angle": round(angle)}

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
    if goal_grid:
        return {
            "x": goal_grid[0],
            "y": goal_grid[1],
            "angle": goal_grid[2]
        }
    else:
        return {
            "x": None,
            "y": None,
            "angle": None
        }

@app.get("/controll_api")
def control_api(
    direction: int, 
    speed: float,     
    angle: float      
):
    global manual_speed, manual_direction, manual_angle
    
    manual_direction = direction
    manual_speed = speed / 5.0 
    manual_angle = angle
    
    print(f"Manual Control: direction={direction}, speed={manual_speed}, angle={angle}")
    
    return {
        "status": "controlled"
    }

@app.get("/set_manual_mode")
def manual_mode(mode:bool):
    global manual_control
    manual_control = bool(mode)
    print(f"Manual Mode: {manual_control}")
    return{
        "manual_control": manual_control
    }

@app.get("/get_manual_mode")
def get_manual_mode():
    return{
        "manual_control": manual_control
    }

@app.get("/linear")
def linear_move(mode:str):
    print(f"Linear: {mode}")
    return {"linear": mode}

# -----------------------------------------------------
# 修正箇所: /slide APIの実装
# -----------------------------------------------------
@app.get("/slide")
def slide_move(mode:str):
    print(f"Slide: {mode}")
    
    # modeに応じてArduinoへコマンド送信
    if mode == "open":
        arduino.send_command('o')
    elif mode == "close":
        arduino.send_command('c')
    else:
        print(f"Unknown slide mode: {mode}")

    return {"slide": mode}


def main():
    uvicorn.run(app, host="0.0.0.0", port=8100, log_level="info")
    
if __name__ == "__main__":
    main()