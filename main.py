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

from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from typing import List, Optional
import uvicorn
import threading
import queue
import asyncio

# ==========================================
# 1. 設定・定数
# ==========================================

SAVED_POINTS_FILE = "saved_points.dat"
STOCKER_FILE = "stocker.dat"
MAP_DATA_PATH = "room.dat"
PROFILE_FILE = "bno_profile.json"

# --- マップ・グリッド設定 ---
GRID_SIZE_MM = 50.0      # 1マスの大きさ (mm)
ROBOT_WIDTH_MM = 400.0   # 筐体幅
ROBOT_DEPTH_MM = 350.0   # 筐体奥行
INFLATION_RADIUS = 6     # 障害物膨張半径

# --- モーター制御設定 ---
BASE_SPEED = 90.0
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

# ==========================================
# 2. グローバル変数・状態管理
# ==========================================

target_pose = (1000.0, 1000.0, 0.0)

manual_control = False
manual_speed = 0.0
manual_direction = 0.0
manual_angle = 0.0
manual_rotate = False
manual_linear = 0

move_queue = queue.Queue()
AUTOMOVE = 1

planner = None
robot = None
position_lock = threading.Lock()

# ==========================================
# 3. クラス定義
# ==========================================

class Point(BaseModel):
    name: str
    x: float
    y: float
    angle: float

# WebSocket接続管理クラス (追加)
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: str):
        for connection in list(self.active_connections):
            try:
                await connection.send_text(message)
            except Exception:
                self.disconnect(connection)

manager = ConnectionManager()

# --- ArduinoController (本番用) ---
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
        self.x = float(start_x)
        self.y = float(start_y)
        self.heading = start_heading

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
        self.y -= dy_global

    def get_grid_pos(self):
        """マップ操作用: 現在の物理座標をグリッド座標に変換して返す"""
        return int(self.x / GRID_SIZE_MM), int(self.y / GRID_SIZE_MM)


# --- PathPlanner (機能強化版) ---
class PathPlanner:
    def __init__(self, raw_map, inflation_r):
        self.grid = self._parse_map(raw_map)
        self.height = len(self.grid)
        self.width = len(self.grid[0])
        self.inflation_r = inflation_r
        # 初期化時にストッカー情報を反映したコストマップを作成
        self.cost_map = self._create_cost_map(include_stocker=True)

    def _parse_map(self, raw_data):
        max_len = max(len(row) for row in raw_data)
        grid = []
        for row_str in raw_data:
            padded = row_str + '1' * (max_len - len(row_str))
            # 1:障害物, 0:通路
            grid.append([int(c) for c in padded])
        return np.array(grid)

    def _to_grid(self, cx, cy, angle, px, py):
        """相対座標(px, py)を絶対グリッド座標(r, c)に変換"""
        rad = math.radians(angle)
        cos_v = math.cos(rad)
        sin_v = math.sin(rad)
        
        # 回転
        tx = px * cos_v - py * sin_v
        ty = px * sin_v + py * cos_v
        
        # 平行移動
        abs_x = cx + tx
        abs_y = cy + ty
        
        c = int(abs_x / GRID_SIZE_MM)
        r = int(abs_y / GRID_SIZE_MM)
        return r, c

    def _create_cost_map(self, include_stocker=True):
        rows, cols = self.grid.shape
        temp_obstacle_map = self.grid.copy()
        
        stocker_safe_indices = []

        # ストッカー情報の反映
        if include_stocker and os.path.exists(STOCKER_FILE):
            try:
                with open(STOCKER_FILE, 'r') as f:
                    data = json.load(f)
                    sx, sy, s_ang = data['x'], data['y'], data['angle']
                    
                    stocker_wall_indices = []
                    
                    # 奥のバー
                    for i in range(11): stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, -250 + i*50, -400))
                    # 左アーム
                    for i in range(4): stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, -250, -200 - i*50))
                    # 右アーム
                    for i in range(4): stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, 250, -200 - i*50))
                    
                    # 内部空間 (左)
                    for x_off in range(-200, 0, 50): 
                        for y_off in range(-350, -150, 50): 
                             stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, x_off, y_off))
                    # 内部空間 (右)
                    for x_off in range(50, 250, 50): 
                        for y_off in range(-350, -150, 50): 
                             stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, x_off, y_off))

                    for r, c in stocker_wall_indices:
                        if 0 <= r < rows and 0 <= c < cols:
                            temp_obstacle_map[r][c] = 1

                    # 安全地帯（中心ルート）
                    for y_off in range(-150, 150, 50):
                        stocker_safe_indices.append(self._to_grid(sx, sy, s_ang, 0, y_off))

            except Exception as e:
                print(f"Stocker load error: {e}")

        # 膨張処理
        cost_map = np.zeros_like(temp_obstacle_map)
        obstacles = np.argwhere(temp_obstacle_map == 1)
        
        for r, c in obstacles:
            r_min = max(0, r - self.inflation_r)
            r_max = min(rows, r + self.inflation_r + 1)
            c_min = max(0, c - self.inflation_r)
            c_max = min(cols, c + self.inflation_r + 1)
            cost_map[r_min:r_max, c_min:c_max] = 1
            
        # 安全地帯のくり抜き
        if include_stocker:
            for r, c in stocker_safe_indices:
                if 0 <= r < rows and 0 <= c < cols:
                    cost_map[r][c] = 0
                
        return cost_map

    def update_cost_map(self):
        """ストッカーファイル変更時などに再生成する"""
        self.cost_map = self._create_cost_map(include_stocker=True)

    def get_path_astar(self, start_grid, goal_grid):
        start = tuple(start_grid)
        goal = tuple(goal_grid)

        if not (0 <= start[0] < self.width and 0 <= start[1] < self.height):
             print(f"エラー: スタート地点 {start} がマップ範囲外です")
             return None
        if not (0 <= goal[0] < self.width and 0 <= goal[1] < self.height):
             print(f"エラー: ゴール地点 {goal} がマップ範囲外です")
             return None

        # ゴールが障害物内なら到達不能
        if self.cost_map[goal[1]][goal[0]] == 1:
            print("エラー: ゴール地点が障害物内または到達不能エリアです")
            return None

        # スタート地点が障害物内の場合のリカバリ（近くの空きマスを探す）
        if self.cost_map[start[1]][start[0]] == 1:
            print("警告: スタート地点が障害物内です。近傍を探索します。")
            found = False
            for dy in range(-2, 3):
                for dx in range(-2, 3):
                    nx, ny = start[0]+dx, start[1]+dy
                    if 0 <= nx < self.width and 0 <= ny < self.height:
                        if self.cost_map[ny][nx] == 0:
                            start = (nx, ny)
                            found = True
                            break
                if found: break
            if not found:
                print("エラー: スタート地点周辺に安全な場所が見つかりません")
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

def move_to_target(_planner, robot, sensor_bno, sensor_pmw, target_pos_mm):
    target_x_mm, target_y_mm, target_angle = target_pos_mm
    
    goal_grid_x = int(target_x_mm / GRID_SIZE_MM)
    goal_grid_y = int(target_y_mm / GRID_SIZE_MM)
    goal_grid = (goal_grid_x, goal_grid_y)

    with position_lock:
        start_grid = robot.get_grid_pos()
    
    print(f"Start: {start_grid} -> Goal: {goal_grid}")
    path = _planner.get_path_astar(start_grid, goal_grid)
    
    if not path:
        print("経路なし")
        return False
    
    state_turning = True 

    for i in range(1, len(path)):
        next_node = path[i]
        
        next_target_x_mm = next_node[0] * GRID_SIZE_MM + GRID_SIZE_MM/2
        next_target_y_mm = next_node[1] * GRID_SIZE_MM + GRID_SIZE_MM/2
        
        if i == len(path) - 1:
            next_target_x_mm = target_x_mm
            next_target_y_mm = target_y_mm
        
        print(f"Next WP: ({next_target_x_mm:.0f}, {next_target_y_mm:.0f})")

        while True:            
            with position_lock:
                cx = robot.x
                cy = robot.y
                ch = robot.heading

            dx = next_target_x_mm - cx
            dy = next_target_y_mm - cy
            dist = math.sqrt(dx**2 + dy**2)
            
            if dist < DIST_THRESHOLD_MM:
                set_motor_speed(0, 0)
                break

            target_rad = math.atan2(dx, -dy)
            target_deg = math.degrees(target_rad)
            if target_deg < 0: target_deg += 360

            diff = (target_deg - ch + 180) % 360 - 180
            if abs(diff) > 170:
                print(f"U-Turn Mode: Diff={diff:.1f}")
                set_motor_speed(70, -70) 
                time.sleep(0.05)
                continue
            
            # print(f"Tgt:{target_deg:.0f} Cur:{ch:.0f} Diff:{diff:.0f} Dist:{dist:.0f} Mode:{'TURN' if state_turning else 'GO'}")

            if state_turning:
                if abs(diff) < 10.0:
                    state_turning = False
                    set_motor_speed(0, 0)
                    time.sleep(0.1)
                else:
                    turn_pow = KP_TURN * diff
                    min_p = 80
                    if turn_pow > 0: turn_pow = max(turn_pow, min_p)
                    else: turn_pow = min(turn_pow, -min_p)
                    set_motor_speed(-turn_pow, turn_pow)

            else: # 直進モード
                if abs(diff) > 30.0:
                    state_turning = True
                    set_motor_speed(0, 0)
                else:
                    correction = diff * KP_DIST
                    curr_base = BASE_SPEED
                    l = curr_base - correction
                    r = curr_base + correction
                    set_motor_speed(l, r)
            
            time.sleep(0.05)

    set_motor_speed(0, 0)
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


# ==========================================
# 4. API定義
# ==========================================

app = FastAPI()

robot = None
arduino = ArduinoController()

# --- 非同期ブロードキャストタスク ---
async def broadcast_position_task():
    while True:
        await asyncio.sleep(0.05) 
        with position_lock:
            if robot:
                data = {
                    "type": "position",
                    "x": robot.x,
                    "y": robot.y,
                    "angle": robot.heading
                }
                # JSONシリアライズして送信
                await manager.broadcast(json.dumps(data))

@app.on_event("startup")
async def startup_event():
    # FastAPI起動時にブロードキャストタスクをバックグラウンドで開始
    asyncio.create_task(broadcast_position_task())

@app.middleware("http")
async def add_cors_header(request: Request, call_next):
    response = await call_next(request)
    response.headers["Access-Control-Allow-Origin"] = "*"
    return response

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)
    try:
        while True:
            # クライアントからのメッセージを受信（必要であれば処理）
            await websocket.receive_text()
    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.get("/")
async def root():
    return {"message":"Hello! I'm MoFeL Real Robot!"}

@app.get("/position")
async def position():
    with position_lock:
        if robot:
            return {"x":robot.x,"y":robot.y,"angle":robot.heading}
        return {"x":0, "y":0,"angle":0}

@app.get("/mapdata")
async def mapdata():
    if planner:
        str_map = []
        for row in planner.grid:
            str_map.append("".join(map(str, row)))
        return {"mapdata": str_map}
    return{"mapdata": []}

@app.get("/costmapdata")
async def costmapdata(mode: str = "normal"):
    """
    mode="base" の場合はストッカーを除外したマップを返す（編集用）
    それ以外は経路計画用のキャッシュされたマップを返す
    """
    if planner:
        target_map = None
        if mode == "base":
            target_map = planner._create_cost_map(include_stocker=False)
        else:
            target_map = planner.cost_map
            
        cost_str_list = []
        for row in target_map:
            cost_str_list.append("".join(map(str, row)))
        return {"costmapdata": cost_str_list}
    return {"costmapdata": []}

@app.get("/saved_target_points")
async def target_points():
    if os.path.exists(SAVED_POINTS_FILE):
        try:
            with open(SAVED_POINTS_FILE, mode='r', encoding='utf-8') as file:
                return json.load(file)
        except:
             return []
    return []

# 既存の /saved は廃止し、/add_target_point に統一
@app.post("/add_target_point")
def save_points(point: Point):
    data = point.dict()
    try:
        points = []
        if os.path.exists(SAVED_POINTS_FILE):
            try:
                with open(SAVED_POINTS_FILE, "r", encoding="utf-8") as rf:
                    points = json.load(rf)
            except json.JSONDecodeError:
                points = []
        found = False
        for i, p in enumerate(points):
            if p.get("name") == data["name"]:
                points[i] = data
                found = True
                break
        if not found:
            points.append(data)
        with open(SAVED_POINTS_FILE, "w", encoding="utf-8") as wf:
            json.dump(points, wf, ensure_ascii=False, indent=4)
        return {"message": "saved"}
    except Exception as e:
        return {"message": "error", "error": str(e)}

@app.get("/delete_target_point")
def delete_target_point(name: str):
    try:
        points = []
        if os.path.exists(SAVED_POINTS_FILE):
            with open(SAVED_POINTS_FILE, "r", encoding="utf-8") as rf:
                points = json.load(rf)
        points = [p for p in points if p.get("name") != name]
        with open(SAVED_POINTS_FILE, "w", encoding="utf-8") as wf:
            json.dump(points, wf, ensure_ascii=False, indent=4)
        return {"message": "deleted"}
    except Exception as e:
        return {"message": "error", "error": str(e)}

@app.get("/target_point")
async def target_point():
    return{
        "x" : target_pose[0],
        "y" : target_pose[1],
        "angle" : target_pose[2]
    }

@app.post("/set_target_point")
async def set_target_point(point: Point):
    global target_pose
    target_pose = (point.x, point.y, point.angle)
    return point

@app.post("/set_stocker")
def set_stocker(stocker:Point):
    global planner
    data = stocker.dict()
    try:
        with open(STOCKER_FILE, "w", encoding="utf-8") as wf:
            json.dump(data, wf, ensure_ascii=False, indent=4)
        
        if planner:
            planner.update_cost_map()
            
        return {"message": "stocker set"}
    except Exception as e:
        return {"message": "error", "error": str(e)}

@app.get("/get_stocker")
def get_stocker():
    if os.path.exists(STOCKER_FILE):
        try:
            with open(STOCKER_FILE, mode='r', encoding='utf-8') as f:
                return json.load(f)
        except:
            return {"message": "error reading stocker"}
    return {"message": "no stocker set"}

# 物理動作制御用API

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
def linear_move_api(mode:str):
    global manual_linear
    if(mode=="up"):
        manual_linear = 1
    elif(mode=="down"):
        manual_linear = -1
    else: #STOP
        manual_linear = 0
    return{
        "linear": manual_linear
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
        
        start_x_mm = start_x_grid * GRID_SIZE_MM
        start_y_mm = start_y_grid * GRID_SIZE_MM
        
        global robot
        robot = RobotState(start_x_mm, start_y_mm, start_heading)

        # マップデータ読み込み
        raw_map_data = []
        if os.path.exists(MAP_DATA_PATH):
            with open(MAP_DATA_PATH, "r") as f:
                raw_map_data = [line.rstrip() for line in f]
        else:
            raw_map_data = ["0"*20 for _ in range(20)]

        global planner
        planner = PathPlanner(raw_map_data, inflation_r=INFLATION_RADIUS)
        
        # --- 位置監視用スレッドの開始 ---
        monitor_thread = threading.Thread(target=monitor_position, args=(robot, bno, pmw), daemon=True)
        monitor_thread.start()
        print("位置監視システムを開始しました。")

        def run_api():
            # FastAPIをスレッドで動かす
            uvicorn.run(app, host="0.0.0.0", port=8100, log_level="info")
        
        api_thread = threading.Thread(target=run_api, daemon=True)
        api_thread.start()
        
        # --- メイン制御ループ ---
        while True:
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
                elif(manual_direction != 0 and manual_rotate): #超信地旋回
                    EB = False
                    manual_right = manual_direction * manual_speed * manual_angle
                    manual_left = -1 * manual_right
                else:
                    manual_right = 0
                    manual_left = 0

                set_motor_speed(manual_left, manual_right,False,EB)
                move_linear(manual_linear)
                time.sleep(0.05)
                continue
            
            time.sleep(0.05)
            
            if(move_queue.qsize() > 0):
                buf_queue = move_queue.get()
                print("queue now")
                print(move_queue.qsize())
                if buf_queue == AUTOMOVE:
                    print("auto move start")
                    move_to_target(planner, robot, bno, pmw, target_pose)

    except KeyboardInterrupt:
        print("\n停止")
    finally:
        set_motor_speed(0, 0)
        GPIO.cleanup()
        arduino.close()

if __name__ == "__main__":
    main()