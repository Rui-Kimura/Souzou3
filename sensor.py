import RPi.GPIO as GPIO
import time
import board
import adafruit_bno055
import math
import json
import os
import sys
from pmw3901 import PMW3901

# ==========================================
# 設定（main.pyと同一）
# ==========================================
PROFILE_FILE = "bno_profile.json"
CONFIG_FILE = "robot_config.json"
SENSOR_HEIGHT_MM = 95.0

# ピン設定（初期化のためだけに使用、モーターは回しません）
L_EN = 19; IN1 = 21; IN2 = 20
R_EN = 26; IN3 = 16; IN4 = 12
LINEAR_IN1 = 5; LINEAR_IN2 = 6

# ==========================================
# ハードウェア初期化
# ==========================================
def setup_hardware():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    pins = [IN1, IN2, IN3, IN4, L_EN, R_EN, LINEAR_IN1, LINEAR_IN2]
    for pin in pins:
        GPIO.setup(pin, GPIO.OUT)
        GPIO.output(pin, GPIO.LOW) # 安全のためLow固定

def get_bno_heading(sensor):
    try:
        h = sensor.euler[0]
        return h if h is not None else 0.0
    except:
        return 0.0

# ==========================================
# キャリブレーションロジック
# ==========================================

def load_config():
    default_config = {
        "pmw_rotation_deg": 0.0,  # センサーの取付回転角度
        "pixel_to_mm": 0.0017 * SENSOR_HEIGHT_MM, # スケール
        "bno_offset_deg": 0.0     # 方位オフセット
    }
    if os.path.exists(CONFIG_FILE):
        try:
            with open(CONFIG_FILE, "r") as f:
                return json.load(f)
        except:
            pass
    return default_config

def save_config(config):
    with open(CONFIG_FILE, "w") as f:
        json.dump(config, f, indent=4)
    print(f"\n設定を {CONFIG_FILE} に保存しました。")

def calibrate_pmw_direction(pmw):
    print("\n=== PMW3901 取付向き補正 ===")
    print("ロボットを【手で前進（進行方向）】させてください。")
    print("準備ができたら Enter を押して測定開始（約3秒間動かし続けてください）")
    input()

    print("測定中...")
    total_dx = 0
    total_dy = 0
    start_time = time.time()
    
    while time.time() - start_time < 3.0:
        try:
            dx, dy = pmw.get_motion()
        except RuntimeError:
            dx, dy = 0, 0
        total_dx += dx
        total_dy += dy
        time.sleep(0.01)

    print(f"計測結果: Raw dX={total_dx}, Raw dY={total_dy}")

    if abs(total_dx) < 10 and abs(total_dy) < 10:
        print("エラー: 移動量が少なすぎます。もう一度やり直してください。")
        return None

    # ベクトルから角度を計算 (ForwardをY軸プラスとした場合の回転角)
    # atan2(y, x) だが、ここでは進行方向をY軸正としたい
    # センサー座標系で、前進したベクトルが (total_dx, total_dy)
    # これを (0, 1) に合わせるための回転角を求める
    
    # センサー上のベクトル角度
    angle_rad = math.atan2(total_dy, total_dx)
    # 理想的な前進ベクトル (0, 1) つまり 90度(pi/2)
    # 補正角 = 理想 - 計測
    correction_rad = (math.pi / 2) - angle_rad
    correction_deg = math.degrees(correction_rad)

    # 正規化 (-180 ~ 180)
    correction_deg = (correction_deg + 180) % 360 - 180

    print(f"検出されたセンサー回転角: {correction_deg:.1f} 度")
    return correction_deg

def calibrate_pmw_scale(pmw, rotation_deg):
    print("\n=== PMW3901 距離スケール補正 ===")
    dist_req = 500.0 # mm
    print(f"ロボットを定規に沿って正確に {dist_req}mm ({dist_req/10.0}cm) 前進させます。")
    print("準備ができたら Enter を押し、移動を開始してください。")
    print("移動が完了したら再度 Enter を押してください。")
    input("Enterで計測開始 >>")

    # 非同期で計測するためのフラグ
    print("計測中... 移動完了したらEnter")
    
    total_fwd_pixels = 0
    measuring = True
    
    # スレッド等は使わず簡易的に実装（Enter待ちがブロッキングするため、ここは時間指定にするか、KeyboardInterruptを使う）
    # ここでは簡易化のため「3秒待機→移動→3秒待機」ではなく、
    # ユーザーが移動させ、止まったらCtrl+Cまたはタイムアウト方式にします。
    # 確実なのは「Enterを押すまでループ」ですが、Pythonのinputはブロッキングします。
    # 簡易的に、一定時間（例えば10秒）の間に動かしてもらう方式にします。
    
    print("10秒間計測します。その間に500mm動かして停止してください。")
    time.sleep(1)
    
    start_time = time.time()
    rad = math.radians(rotation_deg)
    
    while time.time() - start_time < 10.0:
        try:
            dx, dy = pmw.get_motion()
        except:
            dx, dy = 0, 0
            
        # 回転補正を適用して「前進成分」のみ抽出
        # x' = x cos - y sin
        # y' = x sin + y cos (これが前進成分)
        # ※ 上記 calibrate_pmw_direction の計算と合わせる
        
        # 修正: 回転行列
        # Robot_X = dx * cos(th) - dy * sin(th)
        # Robot_Y = dx * sin(th) + dy * cos(th)
        
        fwd_pixel = dx * math.sin(rad) + dy * math.cos(rad)
        total_fwd_pixels += fwd_pixel
        
        sys.stdout.write(f"\r経過: {int(time.time() - start_time)}s / 累積Pixel: {total_fwd_pixels:.1f}")
        sys.stdout.flush()
        time.sleep(0.01)
    
    print("\n計測終了")
    
    if abs(total_fwd_pixels) < 100:
        print("移動量が少なすぎます。")
        return None
        
    new_scale = dist_req / total_fwd_pixels
    print(f"算出されたスケール係数: {new_scale:.6f} mm/pixel")
    return new_scale

def calibrate_bno_heading(bno):
    print("\n=== BNO055 方位ゼロ点補正 ===")
    print("ロボットをマップ上の「角度0度（北/上）」に向けたい方向に置いてください。")
    input("準備ができたら Enter >>")
    
    current_heading = get_bno_heading(bno)
    print(f"現在のセンサー生値: {current_heading:.1f} 度")
    print(f"この値をオフセットとして保存します。")
    return current_heading

def main():
    setup_hardware()
    
    # I2C / BNO Init
    try:
        i2c = board.I2C()
        bno = adafruit_bno055.BNO055_I2C(i2c, address=0x29)
        # プロファイル読み込み（あれば）
        if os.path.exists(PROFILE_FILE):
            with open(PROFILE_FILE, "r") as f:
                data = json.load(f)
            bno.offsets_accelerometer = tuple(data["accel"])
            bno.offsets_gyroscope = tuple(data["gyro"])
            bno.offsets_magnetometer = tuple(data["mag"])
            print("BNOプロファイル適用済み")
        bno.mode = adafruit_bno055.NDOF_MODE
        time.sleep(1)
    except Exception as e:
        print(f"BNO初期化エラー: {e}")
        return

    # PMW Init
    try:
        pmw = PMW3901()
    except Exception as e:
        print(f"PMW3901初期化エラー: {e}")
        return

    config = load_config()

    while True:
        print("\n==============================")
        print(f"現在の設定:")
        print(f"  PMW回転角: {config['pmw_rotation_deg']:.1f} deg")
        print(f"  PMWスケール: {config['pixel_to_mm']:.6f} mm/px")
        print(f"  BNOオフセット: {config['bno_offset_deg']:.1f} deg")
        print("==============================")
        print("1: PMW 取付向き自動補正 (手押しで前進)")
        print("2: PMW 距離スケール補正 (手押しで500mm)")
        print("3: BNO 方位ゼロ点設定 (現在の向きを0とする)")
        print("4: 設定を保存して終了")
        print("q: 保存せずに終了")
        
        c = input("選択 >> ").strip()
        
        if c == '1':
            res = calibrate_pmw_direction(pmw)
            if res is not None:
                config['pmw_rotation_deg'] = res
        elif c == '2':
            res = calibrate_pmw_scale(pmw, config['pmw_rotation_deg'])
            if res is not None:
                config['pixel_to_mm'] = res
        elif c == '3':
            res = calibrate_bno_heading(bno)
            if res is not None:
                config['bno_offset_deg'] = res
        elif c == '4':
            save_config(config)
            break
        elif c == 'q':
            break

    GPIO.cleanup()

if __name__ == "__main__":
    main()