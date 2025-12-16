from fastapi import FastAPI, Request
from pydantic import BaseModel
from typing import List, Optional
import uvicorn
import threading
import time
import math
import numpy as np
import heapq
import random
import serial
import serial.tools.list_ports
import json
import os

# ==========================================
# 1. 設定・定数
# ==========================================

SAVED_POINTS_FILE = "../saved_points.dat"
MAP_DATA_PATH = "../room.dat"
PROFILE_FILE = "../bno_profile.json"

# グリッド・マップ設定
GRID_SIZE_MM = 50.0       # 1マスの大きさ (mm)
INFLATION_RADIUS = 5      # 障害物膨張半径 (グリッド数)

# 動作パラメータ (シミュレーション用)
SPEED = 15.0            # 直進速度 (mm/step)
TURN_RATE_PIVOT = 15.0  # その場旋回時の回転速度 (度/step)
TURN_RATE_MOVE = 5.0    # 走行中の操舵速度 (度/step)
ANGLE_THRESHOLD = 20.0  # 「その場旋回」から「前進」に切り替える角度差の閾値
UPDATE_INTERVAL = 0.05  # ループ更新間隔 (秒)

# ==========================================
# 2. グローバル変数・状態管理
# ==========================================

planner = None

# ロボットの現在状態
robot_state = {
    "x": 200.0,
    "y": 200.0,
    "angle": 0.0  # 0度=上(北), 90度=右(東)
}

# 目標地点
target_state = {
    "x": 0.0,
    "y": 0.0,
    "angle": 0.0
}
current_goal_grid = None 

# 排他制御用ロック
position_lock = threading.Lock()

# 手動操作フラグ
manual_control = False
manual_speed = 0.0
manual_direction = 0
manual_angle = 0.0
manual_rotate = False
manual_linear = 0

# ==========================================
# 3. クラス定義
# ==========================================

class Point(BaseModel):
    name: str
    x: float
    y: float
    angle: float

class ArduinoController:
    """Arduino接続エミュレータ"""
    def __init__(self, baudrate=9600):
        self.ser = None
        self.baudrate = baudrate
        self.port_name = self._find_arduino_port()
        if self.port_name:
            self._connect()
        else:
            print("【Sim】Arduinoが見つかりません。シリアル通信はスキップされます。")

    def _find_arduino_port(self):
        ports = list(serial.tools.list_ports.comports())
        for p in ports:
            if "Arduino" in p.description or "Uno" in p.description:
                return p.device
        return None

    def _connect(self):
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=1)
            time.sleep(2)
            print(f"【Sim】Arduino接続完了: {self.port_name}")
        except Exception as e:
            print(f"【Sim】Arduino接続エラー: {e}")
            self.ser = None

    def send_command(self, command):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(command.encode('utf-8'))
                self.ser.flush()
                print(f"【Sim】Arduino送信: {command}")
            except Exception as e:
                print(f"【Sim】送信エラー: {e}")
        else:
            print(f"【Sim】(仮想送信) Arduinoコマンド: {command}")

    def close(self):
        if self.ser: self.ser.close()

class PathPlanner:
    """A*アルゴリズムを用いた経路計画クラス"""
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
            grid.append([1 if c == '1' else 0 for c in padded])
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
        
        if not (0 <= start[0] < self.width and 0 <= start[1] < self.height): return None
        if not (0 <= goal[0] < self.width and 0 <= goal[1] < self.height): return None
        if self.cost_map[goal[1]][goal[0]] == 1: return None 

        if self.cost_map[start[1]][start[0]] == 1:
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
            if not found: return None

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        
        while open_set:
            current = heapq.heappop(open_set)[1]
            if current == goal:
                return self._reconstruct_path(came_from, current)

            for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
                neighbor = (current[0] + dx, current[1] + dy)
                if not (0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.height): continue
                if self.cost_map[neighbor[1]][neighbor[0]] == 1: continue

                tentative_g = g_score[current] + 1
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g
                    f = tentative_g + abs(neighbor[0] - goal[0]) + abs(neighbor[1] - goal[1])
                    heapq.heappush(open_set, (f, neighbor))
        return None

    def _reconstruct_path(self, came_from, current):
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

# ==========================================
# 4. バックグラウンド シミュレーション
# ==========================================

def background_simulation():
    global robot_state, current_goal_grid, target_state, planner
    
    # マップ読み込み
    raw_map = []
    if os.path.exists(MAP_DATA_PATH):
        with open(MAP_DATA_PATH, "r") as f:
            raw_map = [line.strip() for line in f]
    else:
        print("マップファイルが見つかりません。ダミーを使用します。")
        raw_map = ["1"*20, "1"+"0"*18+"1"] + ["1"+"0"*18+"1" for _ in range(15)] + ["1"*20]

    planner = PathPlanner(raw_map, INFLATION_RADIUS)
    print("シミュレーション開始: 非ホロノミック移動モード (0度=上)")

    while True:
        time.sleep(UPDATE_INTERVAL)
        
        # ----------------------------------
        # 1. 手動モード (Manual Control)
        # ----------------------------------
        if manual_control:
            with position_lock:
                # (1) 旋回
                if manual_angle != 0:
                    turn_amount = manual_angle * 0.5
                    robot_state["angle"] = (robot_state["angle"] + turn_amount) % 360
                
                # (2) 前進・後退
                if manual_speed != 0 and manual_direction != 0:
                    v = manual_speed * 10.0
                    if manual_direction == -1: v *= -1
                    
                    rad = math.radians(robot_state["angle"])
                    
                    # 【修正箇所1】本番プログラムに合わせて座標系を変更
                    # 0度=上(Y-), 90度=右(X+)
                    # X移動量: v * sin(angle)
                    # Y移動量: -v * cos(angle)
                    nx = robot_state["x"] + v * math.sin(rad)
                    ny = robot_state["y"] - v * math.cos(rad)
                    
                    gx, gy = int(nx / GRID_SIZE_MM), int(ny / GRID_SIZE_MM)
                    if 0 <= gy < planner.height and 0 <= gx < planner.width:
                        if planner.grid[gy][gx] == 0:
                            robot_state["x"], robot_state["y"] = nx, ny
            continue

        # ----------------------------------
        # 2. 自動移動モード (Auto Move)
        # ----------------------------------
        start_grid = (int(robot_state["x"] / GRID_SIZE_MM), int(robot_state["y"] / GRID_SIZE_MM))
        
        if current_goal_grid is None:
            while True:
                gy = random.randint(1, planner.height - 2)
                gx = random.randint(1, planner.width - 2)
                if planner.cost_map[gy][gx] == 0:
                    current_goal_grid = (gx, gy)
                    target_state["x"] = gx * GRID_SIZE_MM + GRID_SIZE_MM/2
                    target_state["y"] = gy * GRID_SIZE_MM + GRID_SIZE_MM/2
                    target_state["angle"] = float(random.randint(0, 360))
                    print(f"新しい目的地: {current_goal_grid}")
                    break
        
        path = planner.get_path_astar(start_grid, current_goal_grid)
        
        if not path or len(path) < 2:
            if path and len(path) == 1:
                diff = (target_state["angle"] - robot_state["angle"] + 360) % 360
                if diff > 180: diff -= 360
                
                if abs(diff) > 2.0:
                    turn = TURN_RATE_PIVOT if diff > 0 else -TURN_RATE_PIVOT
                    if abs(turn) > abs(diff): turn = diff
                    robot_state["angle"] = (robot_state["angle"] + turn) % 360
                else:
                    time.sleep(1)
                    current_goal_grid = None
            else:
                time.sleep(1)
                current_goal_grid = None
            continue

        next_node = path[1]
        tgt_x = next_node[0] * GRID_SIZE_MM + GRID_SIZE_MM/2
        tgt_y = next_node[1] * GRID_SIZE_MM + GRID_SIZE_MM/2
        
        dx = tgt_x - robot_state["x"]
        dy = tgt_y - robot_state["y"]
        dist = math.sqrt(dx**2 + dy**2)
        
        # 【修正箇所2】ターゲット方位の計算を本番ロジックに合わせる
        # 本番: target_rad = math.atan2(dx, -dy) 
        # (Y軸が下向きプラスのため、上向きを0度にするには -dy を使う)
        tgt_rad = math.atan2(dx, -dy)
        tgt_deg = math.degrees(tgt_rad)
        if tgt_deg < 0: tgt_deg += 360
        
        angle_diff = (tgt_deg - robot_state["angle"] + 360) % 360
        if angle_diff > 180: angle_diff -= 360
        
        with position_lock:
            if dist < SPEED:
                robot_state["x"] = tgt_x
                robot_state["y"] = tgt_y
            else:
                if abs(angle_diff) > ANGLE_THRESHOLD:
                    turn = TURN_RATE_PIVOT if angle_diff > 0 else -TURN_RATE_PIVOT
                    robot_state["angle"] = (robot_state["angle"] + turn) % 360
                else:
                    turn = TURN_RATE_MOVE if angle_diff > 0 else -TURN_RATE_MOVE
                    if abs(turn) > abs(angle_diff): turn = angle_diff
                    robot_state["angle"] = (robot_state["angle"] + turn) % 360
                    
                    # 【修正箇所3】自動移動の前進計算を本番ロジックに合わせる
                    # X += speed * sin(angle)
                    # Y -= speed * cos(angle)
                    current_rad = math.radians(robot_state["angle"])
                    robot_state["x"] += SPEED * math.sin(current_rad)
                    robot_state["y"] -= SPEED * math.cos(current_rad)


# ==========================================
# 5. API エンドポイント定義
# ==========================================

app = FastAPI()
arduino = ArduinoController()

@app.on_event("startup")
def startup_event():
    t = threading.Thread(target=background_simulation, daemon=True)
    t.start()

@app.middleware("http")
async def add_cors_header(request: Request, call_next):
    response = await call_next(request)
    response.headers["Access-Control-Allow-Origin"] = "*"
    return response

@app.get("/")
def root():
    return {"message": "Simulation API Online"}

@app.get("/position")
def get_position():
    with position_lock:
        return robot_state

@app.get("/mapdata")
def get_mapdata():
    if planner:
        str_map = []
        for row in planner.grid:
            str_map.append("".join(map(str, row)))
        return {"mapdata": str_map}
    return {"mapdata": []}

@app.get("/costmapdata")
def get_costmapdata():
    if planner is not None:
        cost_str_list = []
        for row in planner.cost_map:
            cost_str_list.append("".join(map(str, row)))
        return {"costmapdata": cost_str_list}
    return {"costmapdata": []}

@app.get("/target_point")
def get_target_point():
    return target_state

@app.get("/set_target_point")
def set_target_point(x: float, y: float, angle: float):
    global target_state, current_goal_grid
    target_state = {"x": x, "y": y, "angle": angle}
    gx = int(x / GRID_SIZE_MM)
    gy = int(y / GRID_SIZE_MM)
    if planner and 0 <= gx < planner.width and 0 <= gy < planner.height:
        current_goal_grid = (gx, gy)
        print(f"ユーザー指定ターゲット: {current_goal_grid}")
    return target_state

@app.get("/automove_start")
def automove_start():
    return {"auto": "started"}

@app.get("/controll_api")
def control_api(direction: int, speed: float, angle: float, rotate: bool):
    global manual_direction, manual_speed, manual_angle, manual_rotate
    manual_direction = direction
    manual_speed = speed / 5.0
    manual_angle = angle
    manual_rotate = rotate
    return {"status": "controlled"}

@app.get("/set_manual_mode")
def set_manual_mode(mode: bool):
    global manual_control
    manual_control = mode
    print(f"Manual Mode: {manual_control}")
    return {"manual_control": manual_control}

@app.get("/linear")
def linear_move(mode: str):
    global manual_linear
    if mode == "up": manual_linear = 1
    elif mode == "down": manual_linear = -1
    else: manual_linear = 0
    return {"linear": manual_linear}

@app.get("/slide")
def slide_move(mode: str):
    if mode == "open": arduino.send_command('o')
    elif mode == "close": arduino.send_command('c')
    return {"slide": mode}

@app.get("/saved_target_points")
def get_saved_points():
    if os.path.exists(SAVED_POINTS_FILE):
        try:
            with open(SAVED_POINTS_FILE, mode='r', encoding='utf-8') as f:
                return json.load(f)
        except:
            return {"points": []}
    return {"points": []}

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

def main():
    uvicorn.run(app, host="0.0.0.0", port=8100, log_level="info")

if __name__ == "__main__":
    main()