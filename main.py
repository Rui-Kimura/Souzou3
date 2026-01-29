import asyncio
import heapq
import json
import math
import os
import queue
import subprocess
import threading
import time
from datetime import datetime
from typing import List, Optional, Dict
from contextlib import asynccontextmanager

import adafruit_bno055
import board
import cv2
import numpy as np
import RPi.GPIO as GPIO
import serial
import serial.tools.list_ports
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from pydantic import BaseModel
from pmw3901 import PMW3901
from pyzbar.pyzbar import decode, ZBarSymbol

# I2C-6を直接扱うためのライブラリ
import smbus2

# ==========================================
# Custom I2C Class for I2C-6 (Final Fix)
# ==========================================
class I2C6_Wrapper:
    """
    adafruit_bno055などが期待するI2Cインターフェースを
    smbus2を使ってI2C-6で再現するラッパークラス
    """
    def __init__(self, bus_id=6):
        self.bus = smbus2.SMBus(bus_id)

    def try_lock(self):
        return True

    def unlock(self):
        pass

    def writeto(self, address, buffer, start=0, end=None, stop=True):
        if end is None:
            end = len(buffer)
        data = list(buffer[start:end])
        msg = smbus2.i2c_msg.write(address, data)
        self.bus.i2c_rdwr(msg)

    def readfrom_into(self, address, buffer, start=0, end=None, stop=True):
        if end is None:
            end = len(buffer)
        read_len = end - start
        msg = smbus2.i2c_msg.read(address, read_len)
        self.bus.i2c_rdwr(msg)
        read_data = list(msg)
        for i in range(read_len):
            buffer[start + i] = read_data[i]

    def writeto_then_readfrom(self, address, out_buffer, in_buffer, out_start=0, out_end=None, in_start=0, in_end=None, stop=True):
        if out_end is None:
            out_end = len(out_buffer)
        if in_end is None:
            in_end = len(in_buffer)
        data_to_write = list(out_buffer[out_start:out_end])
        write_msg = smbus2.i2c_msg.write(address, data_to_write)
        read_len = in_end - in_start
        read_msg = smbus2.i2c_msg.read(address, read_len)
        self.bus.i2c_rdwr(write_msg, read_msg)
        read_data = list(read_msg)
        for i in range(read_len):
            in_buffer[in_start + i] = read_data[i]

# ==========================================
# Constants & Configuration
# ==========================================

IS_DEMO = False

SAVED_POINTS_FILE = "saved_points.dat"
STOCKER_FILE = "stocker.dat"
MAP_DATA_PATH = "room.dat"
TABLE_DATA_FILE = "tables.dat"
SCHEDULES_FILE = "schedules.dat"
PROFILE_FILE = "bno_profile.json"
CONFIG_FILE = "robot_config.json"

GRID_SIZE_MM = 50.0
ROBOT_WIDTH_MM = 400.0
ROBOT_DEPTH_MM = 350.0
INFLATION_RADIUS = 6

BASE_SPEED = 90.0
KP_DIST = 1.5
KP_TURN = 1.2
TURN_THRESHOLD_DEG = 3.0
DIST_THRESHOLD_MM = 40.0

# GPIO Pins
L_EN = 19
IN1 = 21
IN2 = 20
R_EN = 26
IN3 = 16
IN4 = 12
FREQ = 100

LINEAR_IN1 = 5
LINEAR_IN2 = 6

SENSOR_HEIGHT_MM = 95.0

# Command Constants
AUTOMOVE = 1
PICKTABLE = 2
SCHEDULE_ACTION = 3

# ==========================================
# Data Models
# ==========================================
class Point(BaseModel):
    name: str
    x: float
    y: float
    angle: float

class TableItem(BaseModel):
    id: str
    name: str
    memo: str

class ScheduleItem(BaseModel):
    id: str
    name: str
    Day: str  # 例: "Monday"
    time: str # 例: "14:30"
    position: str # 変更: 座標オブジェクトではなく、saved_pointsのnameを指定
    tableId: str

# ==========================================
# Global Variables
# ==========================================
target_pose = (1000.0, 1000.0, 0.0)

manual_control = False
manual_speed = 0.0
manual_direction = 0.0
manual_angle = 0.0
manual_rotate = False
manual_linear = 0

move_queue = queue.Queue()

planner = None
robot = None
position_lock = threading.Lock()
holding_table_id = None
reserved_table_id = None

scheduled_job_data = None
last_scheduled_minute = ""

# PWM Objects
pwm1 = None
pwm2 = None
pwm3 = None
pwm4 = None

# ==========================================
# Utility Functions
# ==========================================
def load_robot_config():
    default_config = {
        "pmw_rotation_deg": 0.0,
        "pixel_to_mm": 0.0017 * SENSOR_HEIGHT_MM,
        "bno_offset_deg": 0.0
    }
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except:
            pass
    return default_config

ROBOT_CONFIG = load_robot_config()

def pos_to_grid(x_mm, y_mm):
    return int(x_mm / GRID_SIZE_MM), int(y_mm / GRID_SIZE_MM)

def grid_to_pos_center(gx, gy):
    return gx * GRID_SIZE_MM + GRID_SIZE_MM / 2.0, gy * GRID_SIZE_MM + GRID_SIZE_MM / 2.0

def normalize_angle_diff(target_deg, current_deg):
    return (target_deg - current_deg + 180) % 360 - 180

def load_data_from_file() -> List[dict]:
    if not os.path.exists(TABLE_DATA_FILE):
        return []
    try:
        with open(TABLE_DATA_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError:
        return []

def save_data_to_file(data: List[dict]):
    with open(TABLE_DATA_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)

def load_schedules_from_file() -> List[dict]:
    if not os.path.exists(SCHEDULES_FILE):
        return []
    try:
        with open(SCHEDULES_FILE, "r", encoding="utf-8") as f:
            return json.load(f)
    except json.JSONDecodeError:
        return []

def save_schedules_to_file(data: List[dict]):
    with open(SCHEDULES_FILE, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=4, ensure_ascii=False)

def get_saved_point_by_name(name: str):
    if not os.path.exists(SAVED_POINTS_FILE):
        return None
    try:
        with open(SAVED_POINTS_FILE, "r", encoding="utf-8") as f:
            points = json.load(f)
            for p in points:
                if p.get("name") == name:
                    return p
    except:
        pass
    return None

# ==========================================
# Classes
# ==========================================
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

    def update(self, bno_raw_heading, pmw_raw_dx, pmw_raw_dy):
        if bno_raw_heading is not None:
            corrected_heading = -bno_raw_heading - ROBOT_CONFIG["bno_offset_deg"]
            self.heading = corrected_heading % 360

        rot_rad = math.radians(ROBOT_CONFIG["pmw_rotation_deg"])
        cos_r = math.cos(rot_rad)
        sin_r = math.sin(rot_rad)

        dx_robot = pmw_raw_dx * cos_r - pmw_raw_dy * sin_r
        dy_robot = pmw_raw_dx * sin_r + pmw_raw_dy * cos_r

        scale = ROBOT_CONFIG["pixel_to_mm"]
        dist_fwd = dy_robot * scale
        dist_side = dx_robot * scale

        map_rad = math.radians(self.heading)
        sin_map = math.sin(map_rad)
        cos_map = math.cos(map_rad)

        dx_global = dist_fwd * sin_map + dist_side * cos_map
        dy_global = -dist_fwd * cos_map + dist_side * sin_map

        self.x += dx_global
        self.y += dy_global

    def get_grid_pos(self):
        return pos_to_grid(self.x, self.y)

class PathPlanner:
    def __init__(self, raw_map, inflation_r):
        self.grid = self._parse_map(raw_map)
        self.height = len(self.grid)
        self.width = len(self.grid[0])
        self.inflation_r = inflation_r
        self.cost_map = self._create_cost_map(include_stocker=True)

    def _parse_map(self, raw_data):
        max_len = max(len(row) for row in raw_data)
        grid = []
        for row_str in raw_data:
            padded = row_str + '1' * (max_len - len(row_str))
            grid.append([int(c) for c in padded])
        return np.array(grid)

    def _to_grid(self, cx, cy, angle, px, py):
        rad = math.radians(angle)
        cos_v = math.cos(rad)
        sin_v = math.sin(rad)
        
        tx = px * cos_v - py * sin_v
        ty = px * sin_v + py * cos_v
        
        abs_x = cx + tx
        abs_y = cy + ty
        
        gx, gy = pos_to_grid(abs_x, abs_y)
        return gy, gx

    def _create_cost_map(self, include_stocker=True):
        rows, cols = self.grid.shape
        temp_obstacle_map = self.grid.copy()
        
        stocker_safe_indices = []

        if include_stocker and os.path.exists(STOCKER_FILE):
            try:
                with open(STOCKER_FILE, 'r') as f:
                    data = json.load(f)
                    sx, sy, s_ang = data['x'], data['y'], data['angle']
                    
                    stocker_wall_indices = []
                    
                    for i in range(11): stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, -250 + i*50, -400))
                    for i in range(4): stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, -250, -200 - i*50))
                    for i in range(4): stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, 250, -200 - i*50))
                    
                    for x_off in range(-200, 0, 50): 
                        for y_off in range(-350, -150, 50): 
                             stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, x_off, y_off))
                    for x_off in range(50, 250, 50): 
                        for y_off in range(-350, -150, 50): 
                             stocker_wall_indices.append(self._to_grid(sx, sy, s_ang, x_off, y_off))

                    for r, c in stocker_wall_indices:
                        if 0 <= r < rows and 0 <= c < cols:
                            temp_obstacle_map[r][c] = 1

                    for x_off in [-50, 0, 50]:
                        for y_off in range(-150, 150, 50):
                            stocker_safe_indices.append(self._to_grid(sx, sy, s_ang, x_off, y_off))

            except Exception as e:
                print(f"Stocker load error: {e}")

        cost_map = np.zeros_like(temp_obstacle_map)
        obstacles = np.argwhere(temp_obstacle_map == 1)
        
        for r, c in obstacles:
            r_min = max(0, r - self.inflation_r)
            r_max = min(rows, r + self.inflation_r + 1)
            c_min = max(0, c - self.inflation_r)
            c_max = min(cols, c + self.inflation_r + 1)
            cost_map[r_min:r_max, c_min:c_max] = 1
            
        if include_stocker:
            for r, c in stocker_safe_indices:
                if 0 <= r < rows and 0 <= c < cols:
                    cost_map[r][c] = 0
                
        return cost_map

    def update_cost_map(self):
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

        if self.cost_map[goal[1]][goal[0]] == 1:
            print("エラー: ゴール地点が障害物内または到達不能エリアです")
            return None

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

# ==========================================
# Hardware Control Functions
# ==========================================
arduino = ArduinoController()

def setup_hardware():
    global pwm1, pwm2, pwm3, pwm4
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pins = [IN1, IN2, IN3, IN4, L_EN, R_EN, LINEAR_IN1, LINEAR_IN2]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW)
    
    pwm1 = GPIO.PWM(IN1, FREQ)
    pwm2 = GPIO.PWM(IN2, FREQ)
    pwm3 = GPIO.PWM(IN3, FREQ)
    pwm4 = GPIO.PWM(IN4, FREQ)
    GPIO.output(L_EN, GPIO.HIGH)
    GPIO.output(R_EN, GPIO.HIGH)
    
    pwm1.start(0)
    pwm2.start(0)
    pwm3.start(0)
    pwm4.start(0)

def set_motor_speed(left_speed, right_speed, rotate=False, brake=False):
    if brake:
        pwm1.ChangeDutyCycle(100)
        pwm2.ChangeDutyCycle(100)
        pwm3.ChangeDutyCycle(100)
        pwm4.ChangeDutyCycle(100)
        return
    
    left_duty = max(-100, min(100, left_speed))
    right_duty = max(-100, min(100, right_speed))
    
    if left_duty > 0:
        pwm1.ChangeDutyCycle(left_duty)
        pwm2.ChangeDutyCycle(0)
    else:
        pwm1.ChangeDutyCycle(0)
        pwm2.ChangeDutyCycle(abs(left_duty))
    
    if right_duty > 0:
        pwm3.ChangeDutyCycle(right_duty)
        pwm4.ChangeDutyCycle(0)
    else:
        pwm3.ChangeDutyCycle(0)
        pwm4.ChangeDutyCycle(abs(right_duty))

def move_linear(status):
    if status == 1:
        GPIO.output(LINEAR_IN1, GPIO.LOW)
        GPIO.output(LINEAR_IN2, GPIO.HIGH)
    elif status == -1:
        GPIO.output(LINEAR_IN1, GPIO.HIGH)
        GPIO.output(LINEAR_IN2, GPIO.LOW)
    else:
        GPIO.output(LINEAR_IN1, GPIO.LOW)
        GPIO.output(LINEAR_IN2, GPIO.LOW)

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

def find_empty_stock(camera_id=0, timeout_sec=25):
    cap = cv2.VideoCapture(camera_id)
    
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    
    if not cap.isOpened():
        print("Error: カメラを起動できませんでした。")
        return False

    SCAN_RATIO = 0.3
    start_time = time.time()

    lower_red1, upper_red1 = np.array([0, 120, 70]), np.array([10, 255, 255])
    lower_red2, upper_red2 = np.array([170, 120, 70]), np.array([180, 255, 255])
    lower_blue, upper_blue = np.array([100, 150, 0]), np.array([140, 255, 255])
    lower_green, upper_green = np.array([40, 50, 50]), np.array([80, 255, 255])

    print(f"Monitoring started... (Timeout: {timeout_sec}s)")

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > timeout_sec:
                print("\nTimeout: 条件を満たすパターンが見つかりませんでした。")
                return False

            ret, frame = cap.read()
            if not ret:
                continue

            height, width = frame.shape[:2]
            scan_h = int(height * SCAN_RATIO)
            center_y, center_x = height // 2, width // 2
            top_y, bottom_y = center_y - (scan_h // 2), center_y + (scan_h // 2)
            
            roi = frame[top_y:bottom_y, 0:width]
            hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

            mask_red = cv2.addWeighted(cv2.inRange(hsv, lower_red1, upper_red1), 1.0,
                                         cv2.inRange(hsv, lower_red2, upper_red2), 1.0, 0)
            mask_blue = cv2.inRange(hsv, lower_blue, upper_blue)
            mask_green = cv2.inRange(hsv, lower_green, upper_green)

            red_rects = find_shape_info(mask_red, is_line=False)
            blue_rects = find_shape_info(mask_blue, is_line=False)
            green_lines = find_shape_info(mask_green, is_line=True)

            has_red_left = any(r['cx'] < center_x for r in red_rects)
            has_blue_right = any(b['cx'] > center_x for b in blue_rects)
            has_green_line = len(green_lines) > 0

            if has_red_left and has_blue_right:
                if has_green_line:
                    print(f"\r[STATUS] Pattern matched but Green Line exists. Continuing... ({int(elapsed)}s)", end="")
                    continue
                else:
                    print("\nSuccess: Condition met (Red-Left, Blue-Right).")
                    return True
            
            print(f"\rMonitoring... {int(elapsed)}s", end="")
            time.sleep(0.05) 

    finally:
        cap.release()

def find_shape_info(mask, is_line=False):
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    found = []
    for cnt in contours:
        if cv2.contourArea(cnt) < 500:
            continue
        x, y, w, h = cv2.boundingRect(cnt)
        aspect = float(w) / h
        if (is_line and aspect > 3.0) or (not is_line and 0.5 < aspect < 2.0):
            found.append({'cx': x + w // 2})
    return found

def get_jan_code_value(camera_id=0, timeout_sec=25):
    cap = cv2.VideoCapture(camera_id)
    if not cap.isOpened():
        print("Error: カメラを起動できませんでした。")
        return None

    SCAN_RATIO = 0.3 
    start_time = time.time()
    
    try:
        while True:
            if timeout_sec is not None:
                if (time.time() - start_time) > timeout_sec:
                    print("Timeout: バーコードが見つかりませんでした。")
                    return None

            ret, frame = cap.read()
            if not ret:
                continue

            height, width = frame.shape[:2]
            
            scan_h = int(height * SCAN_RATIO)
            center_y = height // 2
            top_y = center_y - (scan_h // 2)
            bottom_y = center_y + (scan_h // 2)
            roi_img = frame[top_y:bottom_y, 0:width]

            barcodes = decode(roi_img, symbols=[ZBarSymbol.EAN13, ZBarSymbol.EAN8])

            if len(barcodes) > 0:
                detected_value = barcodes[0].data.decode('utf-8')
                return detected_value
                
    finally:
        cap.release()

# ==========================================
# Action & Navigation Logic
# ==========================================
def move_to_target(_planner, robot, sensor_bno, sensor_pmw, target_pos_mm):
    target_x_mm, target_y_mm, target_angle = target_pos_mm
    
    goal_grid = pos_to_grid(target_x_mm, target_y_mm)

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
        
        next_target_x_mm, next_target_y_mm = grid_to_pos_center(next_node[0], next_node[1])
        
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

            diff = normalize_angle_diff(target_deg, ch)
            if abs(diff) > 170:
                print(f"U-Turn Mode: Diff={diff:.1f}")
                set_motor_speed(70, -70) 
                time.sleep(0.05)
                continue
            
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

            else: 
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
    
    print(f"Aligning to final angle: {target_angle:.1f}")
    align_start_time = time.time()
    while time.time() - align_start_time < 5.0:
        with position_lock:
            ch = robot.heading
        
        diff = normalize_angle_diff(target_angle, ch)
        if abs(diff) < TURN_THRESHOLD_DEG:
            break
        
        turn_pow = KP_TURN * diff
        min_p = 80
        if turn_pow > 0: turn_pow = max(turn_pow, min_p)
        else: turn_pow = min(turn_pow, -min_p)
        
        turn_pow = max(-100, min(100, turn_pow))
        
        set_motor_speed(-turn_pow, turn_pow)
        time.sleep(0.05)

    set_motor_speed(0, 0)

    return True

def return_table():
    global holding_table_id
    if holding_table_id is None:
        return None
    FoundEmpty = find_empty_stock()
    if FoundEmpty:
        move_linear(0)
        time.sleep(0.5)
        move_linear(1)
        time.sleep(1)
        move_linear(0)
        arduino.send_command('r')
        time.sleep(1)
        arduino.send_command('o')
        time.sleep(5)
        move_linear(-1)
        time.sleep(1.5)
        move_linear(0)
        arduino.send_command('c')
        time.sleep(5)
        arduino.send_command('g')
        move_linear(-1)
        time.sleep(24)
        move_linear(0)
        holding_table_id = None

def pick_table(target_table_id: str):
    if target_table_id == holding_table_id:
        return holding_table_id
    arduino.send_command('g')
    time.sleep(1)
    if holding_table_id is not None:
        return_table()
    
    move_linear(1)
    detected_id = get_jan_code_value()
    
    if detected_id is None:
        print("テーブルが発見できませんでした。")
        move_linear(-1)
        time.sleep(25)
        move_linear(0)
        return None
    else:
        move_linear(0)
        time.sleep(0.5)
        move_linear(-1)
        time.sleep(1)
        move_linear(0)
        arduino.send_command('r')
        arduino.send_command('o')
        time.sleep(5)
        move_linear(1)
        time.sleep(1.5)
        move_linear(0)
        arduino.send_command('c')
        time.sleep(5)
        arduino.send_command('g')
        time.sleep(1)
        arduino.send_command('r')
        time.sleep(1)
        arduino.send_command('g')
        time.sleep(1)
        move_linear(-1)
        time.sleep(24)
        move_linear(0)
    return detected_id

# ==========================================
# FastAPI Setup & Handlers
# ==========================================

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
                await manager.broadcast(json.dumps(data))

@asynccontextmanager
async def lifespan(app: FastAPI):
    # Startup event
    asyncio.create_task(broadcast_position_task())
    yield
    # Shutdown event (if needed) goes here

app = FastAPI(lifespan=lifespan)

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
            data_text = await websocket.receive_text()
            message = json.loads(data_text)
            msg_type = message.get("type")

            if msg_type == "control":
                global manual_direction, manual_speed, manual_angle, manual_rotate
                manual_direction = message["direction"]
                manual_speed = message["speed"] / 5.0
                manual_angle = message["angle"]
                manual_rotate = message["rotate"]
                print(f"受信データ: direction={manual_direction}, speed={manual_speed}, angle={manual_angle}, rotate={manual_rotate}")
            
            elif msg_type == "manual_mode":
                global manual_control
                manual_control = bool(message["mode"])

            elif msg_type == "linear":
                global manual_linear
                mode = message["mode"]
                if mode == "up":
                    manual_linear = 1
                elif mode == "down":
                    manual_linear = -1
                else:
                    manual_linear = 0
                print(f"Linear Command Received: {mode}")

            elif msg_type == "slide":
                mode = message["mode"]
                print(f"Slide Command Received: {mode}")
                if mode == "open":
                    arduino.send_command('o')
                elif mode == "close":
                    arduino.send_command('c')
                else:
                    print(f"Unknown slide mode: {mode}")

    except WebSocketDisconnect:
        manager.disconnect(websocket)

@app.get("/shutdown")
def shutdown():
    os.system("sudo shutdown -h now")
    return {}

@app.get("/version")
def get_version():
    try:
        cmd = [
            'git',
            'log',
            '-1',
            '--date=format:%Y.%m.%d.%H.%M.%S',
            '--format=%cd.%h'
        ]

        result = subprocess.run(
            cmd,
            capture_output=True, 
            text=True,           
            check=True           
        )

        return result.stdout.strip()

    except subprocess.CalledProcessError:
        return "error"
    except FileNotFoundError:
        return "error"

@app.get("/")
async def root():
    return {"message":"Hello! I'm MoFeL!"}

# --- Position & Map API ---

@app.get("/position")
async def get_position():
    with position_lock:
        if robot:
            return {"x":robot.x,"y":robot.y,"angle":robot.heading}
        return {"x":0, "y":0,"angle":0}

@app.get("/mapdata")
async def get_map_data():
    if planner:
        str_map = []
        for row in planner.grid:
            str_map.append("".join(map(str, row)))
        return {"mapdata": str_map}
    return{"mapdata": []}

@app.get("/costmapdata")
async def get_cost_map_data(mode: str = "normal"):
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

# --- Target Point API ---

@app.get("/saved_target_points")
async def get_target_points():
    if os.path.exists(SAVED_POINTS_FILE):
        try:
            with open(SAVED_POINTS_FILE, mode='r', encoding='utf-8') as file:
                return json.load(file)
        except:
             return []
    return []

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
async def get_target_point():
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

# --- Schedule API (Modified) ---

@app.get("/get_schedules")
def get_schedules():
    """保存されたスケジュール一覧を取得"""
    return load_schedules_from_file()

@app.post("/set_schedule")
def set_schedule(item: ScheduleItem):
    """スケジュールを追加または上書き保存"""
    schedules = load_schedules_from_file()
    new_data = item.dict()
    is_updated = False
    
    for i, s in enumerate(schedules):
        if s["id"] == item.id:
            schedules[i] = new_data
            is_updated = True
            break
    
    if not is_updated:
        schedules.append(new_data)
        
    save_schedules_to_file(schedules)
    
    action = "updated" if is_updated else "created"
    return {"message": f"Schedule successfully {action}", "data": item}

# --- Control & Action API ---

@app.get("/automove_start")
async def automove_start():
    move_queue.put(AUTOMOVE)
    return{
        "auto":"started"
    }

@app.get("/controll_api")
def control_robot_api(
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
def set_manual_mode_endpoint(mode:bool):
    global manual_control
    manual_control = bool(mode)
    return{
        "manual_control": manual_control
    }

@app.get("/linear")
def set_linear_move_endpoint(mode:str):
    global manual_linear
    if(mode=="up"):
        manual_linear = 1
    elif(mode=="down"):
        manual_linear = -1
    else: 
        manual_linear = 0
    return{
        "linear": manual_linear
    }

@app.get("/slide")
def set_slide_move_endpoint(mode:str):
    print(f"Slide Command Received: {mode}")
    
    if mode == "open":
        arduino.send_command('o')
    elif mode == "close":
        arduino.send_command('c')
    else:
        print(f"Unknown slide mode: {mode}")

    return {"slide": mode}

@app.get("/table_data")
def get_table_data_api():
    data = load_data_from_file()
    return data

@app.post("/add_table_data")
def add_table_data(item: TableItem):
    current_data = load_data_from_file()
    
    is_updated = False
    new_item_dict = item.model_dump() 

    for index, existing_item in enumerate(current_data):
        if existing_item["id"] == item.id:
            current_data[index] = new_item_dict
            is_updated = True
            break
    
    if not is_updated:
        current_data.append(new_item_dict)
    
    save_data_to_file(current_data)
    
    action = "updated" if is_updated else "created"
    return {"message": f"Data successfully {action}", "data": item}

@app.get("/table_now")
def get_current_table_api():
    return{"table_id":holding_table_id}

@app.get("/pick_table")
def pick_table_api(id:str):        
    move_queue.put(PICKTABLE)
    global reserved_table_id
    reserved_table_id = id
    return {"table_id": id}

@app.get("/is_demo")
def is_demo():
    return IS_DEMO

@app.get("/set_is_demo")
def set_is_demo(mode: bool):
    global IS_DEMO
    IS_DEMO = mode
    print(f"Demo Mode: {IS_DEMO}")
    return {"demo_mode": IS_DEMO}

# ==========================================
# Scheduler Logic
# ==========================================
def scheduler_loop():
    """
    1分ごとに時刻を確認し、スケジュールを実行キューに入れるスレッド関数
    """
    global scheduled_job_data, last_scheduled_minute

    print("Scheduler thread started.")
    while True:
        try:
            now = datetime.now()
            current_day = now.strftime("%A") # Monday, Tuesday...
            current_time = now.strftime("%H:%M") # 14:30
            
            # 同じ分に重複実行しないようにチェック
            if current_time == last_scheduled_minute:
                time.sleep(1)
                continue

            # ロボットが操作中(manual_control)や移動中(queue not empty)ならスケジュール実行しない
            if manual_control or not move_queue.empty():
                time.sleep(1)
                continue

            schedules = load_schedules_from_file()
            
            for sch in schedules:
                # 曜日と時刻が一致するか
                if sch["Day"] == current_day and sch["time"] == current_time:
                    print(f"Schedule Matched: {sch['name']} at {current_time}")
                    
                    # 実行するジョブ情報をセット
                    scheduled_job_data = sch
                    
                    # キューにスケジュール実行タスクを追加
                    move_queue.put(SCHEDULE_ACTION)
                    
                    # 実行フラグ更新（この分はもう実行しない）
                    last_scheduled_minute = current_time
                    break
                    
            time.sleep(1) # CPU負荷軽減

        except Exception as e:
            print(f"Scheduler Error: {e}")
            time.sleep(5)

# ==========================================
# Main Loop
# ==========================================

def main():
    global holding_table_id, robot, planner
    try:
        setup_hardware()
        
        i2c = I2C6_Wrapper(bus_id=6)
        
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

        start_raw_heading = get_bno_heading(bno)
        if start_raw_heading is None:
            print("BNOエラー")
            return

        start_map_heading = (start_raw_heading - ROBOT_CONFIG["bno_offset_deg"]) % 360

        start_x_grid = 7
        start_y_grid = 7
        
        start_x_mm = start_x_grid * GRID_SIZE_MM
        start_y_mm = start_y_grid * GRID_SIZE_MM
        
        robot = RobotState(start_x_mm, start_y_mm, start_map_heading)

        raw_map_data = []
        if os.path.exists(MAP_DATA_PATH):
            with open(MAP_DATA_PATH, "r") as f:
                raw_map_data = [line.rstrip() for line in f]
        else:
            raw_map_data = ["0"*20 for _ in range(20)]

        planner = PathPlanner(raw_map_data, inflation_r=INFLATION_RADIUS)
        
        # 既存のスレッド: 位置監視
        monitor_thread = threading.Thread(target=monitor_position, args=(robot, bno, pmw), daemon=True)
        monitor_thread.start()
        print("位置監視システムを開始しました。")

        # 既存のスレッド: APIサーバー
        def run_api():
            uvicorn.run(app, host="0.0.0.0", port=8100, log_level="info")
        
        api_thread = threading.Thread(target=run_api, daemon=True)
        api_thread.start()

        # 追加: スケジューラースレッド
        sched_thread = threading.Thread(target=scheduler_loop, daemon=True)
        sched_thread.start()
        
        while True:
            EB = False
            while manual_control:
                if manual_direction != 0 and not manual_rotate:
                    if manual_speed < 0:
                        manual_left = 0
                        manual_right = 0
                        EB = True
                    elif manual_direction == 1 or manual_direction == -1:
                        EB = False
                        if manual_angle > 0: 
                            manual_left = manual_direction * manual_speed * (100 - manual_angle)
                            manual_right = manual_direction * manual_speed * 100
                        elif manual_angle <= 0: 
                            manual_left = manual_direction * manual_speed * 100
                            manual_right = manual_direction * manual_speed * (100 + manual_angle)
                elif manual_direction != 0 and manual_rotate: 
                    EB = False
                    manual_right = manual_direction * manual_speed * manual_angle
                    manual_left = -1 * manual_right
                else:
                    manual_right = 0
                    manual_left = 0

                set_motor_speed(manual_left, manual_right, False, EB)
                move_linear(manual_linear)
                time.sleep(0.05)
                if not move_queue.empty():
                    move_queue.get() 
                continue
            
            time.sleep(0.05)
            
            if move_queue.qsize() > 0:
                current_task = move_queue.get()
                
                if current_task == AUTOMOVE:
                    print("auto move start")
                    move_to_target(planner, robot, bno, pmw, target_pose)
                
                # Manual Pick Table Logic
                if current_task == PICKTABLE and reserved_table_id != None:
                    if os.path.exists(STOCKER_FILE):
                        try:
                            with open(STOCKER_FILE, 'r') as f:
                                s_data = json.load(f)
                                s_x, s_y, s_ang = s_data['x'], s_data['y'], s_data['angle']
                            
                            with position_lock:
                                dist_sq = (robot.x - s_x)**2 + (robot.y - s_y)**2
                                diff = normalize_angle_diff(s_ang, robot.heading)
                            
                            is_at_pos = dist_sq < DIST_THRESHOLD_MM**2
                            is_at_ang = abs(diff) < TURN_THRESHOLD_DEG

                            if IS_DEMO:
                                holding_table_id = pick_table(reserved_table_id)
                            elif (is_at_pos and is_at_ang):
                                holding_table_id = pick_table(reserved_table_id)
                            else:
                                move_to_target(planner, robot, bno, pmw, (s_x, s_y, s_ang))
                                holding_table_id = pick_table(reserved_table_id)
                            
                        except Exception as e:
                            print(f"Pick Table Error: {e}")
                            set_motor_speed(0, 0)
                    else:
                        print("Stocker not found")

                # Schedule Execution Logic
                if current_task == SCHEDULE_ACTION and scheduled_job_data is not None:
                    print(f"Executing Schedule: {scheduled_job_data['name']}")
                    target_tbl_id = scheduled_job_data['tableId']
                    target_point_name = scheduled_job_data['position'] # saved_pointsのnameを取得
                    
                    # 1. Check Table Status & Pick if needed
                    if holding_table_id != target_tbl_id:
                        print(f"Need table {target_tbl_id}, currently holding {holding_table_id}. Moving to Stocker.")
                        if os.path.exists(STOCKER_FILE):
                            try:
                                with open(STOCKER_FILE, 'r') as f:
                                    s_data = json.load(f)
                                    s_target = (s_data['x'], s_data['y'], s_data['angle'])
                                
                                # Move to Stocker
                                move_to_target(planner, robot, bno, pmw, s_target)
                                
                                # Pick Table
                                new_id = pick_table(target_tbl_id)
                                holding_table_id = new_id
                                
                            except Exception as e:
                                print(f"Schedule Pick Error: {e}")
                        else:
                            print("Stocker not configured, cannot pick table.")
                    
                    # 2. Get Coordinate from Saved Points
                    point_data = get_saved_point_by_name(target_point_name)
                    if point_data:
                        target_coords = (point_data['x'], point_data['y'], point_data['angle'])
                        print(f"Moving to schedule position '{target_point_name}': {target_coords}")
                        move_to_target(planner, robot, bno, pmw, target_coords)
                        print("Schedule Action Completed.")
                    else:
                        print(f"Error: Target point '{target_point_name}' not found in saved_points.dat")

    except KeyboardInterrupt:
        print("\n停止")
    finally:
        set_motor_speed(0, 0)
        GPIO.cleanup()
        arduino.close()

if __name__ == "__main__":
    main()