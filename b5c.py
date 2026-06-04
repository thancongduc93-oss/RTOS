#!/usr/bin/env python3
"""
==========================================================
  USV BOAT — ADVANCED PID NAVIGATION
  b5c.py — Dieu khien muot ma bang thuat toan PID
==========================================================
Chay tren: RASPBERRY PI

DAC DIEM THUYEN (Theo cau hinh):
  - Trong luong: 30kg (Quan tinh rat lon)
  - Kich thuoc: 1m x 0.7m (Rong, de bi can nuoc, can luc be lai lon)
  - Thuật toán PID được thiết kế đặc biệt:
    + Kp cao để thắng sức ì ban đầu.
    + Kd rất cao (quan trọng nhất) để phanh đà xoay của 30kg, chống lố (overshoot).
    + Ki nhỏ để bù gió/dòng chảy tạt ngang mà không gây lắc lư.

Logic dieu khien:
  - Khong dung "vung goc" ma tinh toan tuyen tinh tung 1% toc do motor.
  - Khoang cach < 5m → Dung motor, tha troi quan tinh
  - Khoang cach < 20m → Giam toc dan

Cach chay:
  sudo ip link set can0 up type can bitrate 500000
  sudo python3 b5c.py
"""

import json
import math
import struct
import threading
import time

import can
import paho.mqtt.client as mqtt


# =============================================================
# CONFIG
# =============================================================

# ----- CAN Bus -----
CAN_CHANNEL   = "can0"
CAN_INTERFACE = "socketcan"
CAN_BITRATE   = 500000

# CAN IDs — Nhan tu ESP32
CAN_ID_LOCAL_GPS  = 0x101
CAN_ID_REMOTE_GPS = 0x102
CAN_ID_STATUS     = 0x103
CAN_ID_IMU        = 0x100

# CAN IDs — Gui toi ESP32
CAN_ID_MOTOR_CMD  = 0x201

# ----- MQTT -----
MQTT_BROKER     = "broker.emqx.io"
MQTT_PORT       = 1883
MQTT_CLIENT_ID  = "usv_pi_b5c_001"
TOPIC_TELEMETRY = "usv/boat1/telemetry"
TOPIC_STATUS    = "usv/boat1/status"
TOPIC_COMMAND   = "usv/boat1/command/#"

# ----- PID Parameters (Dành cho thuyền 20kg) -----
# 20kg có quán tính nhỏ hơn 30kg một chút, nên giảm các hệ số xuống.
PID_KP = 2.0           # Tỉ lệ: Cần lực bẻ lái vừa đủ để thắng sức ì của 20kg
PID_KI = 0.03          # Tích phân: Rất nhỏ để bù gió dạt ngang chậm rãi
PID_KD = 1.2           # Đạo hàm: Giảm lực phanh đà xoay xuống so với 30kg vì quán tính ít hơn

PID_INTEGRAL_MAX = 50.0  # Giới hạn bù gió (tránh cộng dồn quá nhiều)
PID_OUTPUT_MAX   = 180.0 # Giới hạn lực rẽ (PWM chênh lệch tối đa)

HEADING_DEADBAND = 3.0   # Sai lệch dưới 3 độ -> PID không can thiệp, đi thẳng tắp

# ----- Distance -----
COAST_DISTANCE_M  = 5.0     # < 5m → tha troi, dung motor
SLOWDOWN_DISTANCE = 20.0    # < 20m → giam toc dan

# ----- Heading Filter -----
HEADING_ALPHA     = 0.25    # EMA filter: 0=smooth, 1=raw

# ----- Motor -----
MOTOR_PWM_MAX = 255
MOTOR_PWM_MIN = -255

# ----- Modes -----
MODE_STOP   = 0
MODE_AUTO   = 1
MODE_MANUAL = 2

# ----- Loop -----
LOOP_RATE_HZ     = 20
DEBUG_PRINT      = True
DEBUG_INTERVAL_S = 0.5


# =============================================================
# PID CONTROLLER CLASS
# =============================================================
class PIDController:
    """PID Controller với chống windup và kẹp (clamp) đầu ra."""
    def __init__(self, Kp, Ki, Kd, integral_max=100.0, output_max=200.0):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.integral_max = integral_max
        self.output_max = output_max
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None

    def compute(self, error):
        now = time.time()
        if self._prev_time is None:
            self._prev_time = now
            self._prev_error = error
            return self._clamp(self.Kp * error)

        dt = now - self._prev_time
        if dt <= 0: dt = 0.001

        # P - Proportional (Lực bẻ lái tức thời)
        P = self.Kp * error

        # I - Integral (Bù gió dạt)
        self._integral += error * dt
        self._integral = max(-self.integral_max, min(self.integral_max, self._integral))
        I = self.Ki * self._integral

        # D - Derivative (Phanh hãm quán tính 30kg)
        derivative = (error - self._prev_error) / dt
        D = self.Kd * derivative

        self._prev_error = error
        self._prev_time = now

        return self._clamp(P + I + D)

    def _clamp(self, value):
        return max(-self.output_max, min(self.output_max, value))

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_time = None


# =============================================================
# NAVIGATION MATH
# =============================================================

def haversine_distance(lat1, lon1, lat2, lon2):
    """Tinh khoang cach giua 2 diem GPS (met)."""
    R = 6371000.0
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lam = math.radians(lon2 - lon1)
    a = math.sin(d_phi/2)**2 + math.cos(phi1)*math.cos(phi2)*math.sin(d_lam/2)**2
    return R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Tinh goc bearing tu diem 1 toi diem 2 (0-360)."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    d_lam = math.radians(lon2 - lon1)
    y = math.sin(d_lam) * math.cos(phi2)
    x = math.cos(phi1)*math.sin(phi2) - math.sin(phi1)*math.cos(phi2)*math.cos(d_lam)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def normalize_angle(angle):
    """Chuan hoa goc ve -180 den +180."""
    while angle > 180: angle -= 360
    while angle < -180: angle += 360
    return angle


def mix_motors(speed, turn):
    """
    Két hợp tốc độ tiến (speed) và lực rẽ PID (turn).
    Nếu turn > 0 (bẻ Phải): motor Trái phải chạy nhanh hơn, motor Phải chậm lại
    Nếu turn < 0 (bẻ Trái): motor Phải phải chạy nhanh hơn, motor Trái chậm lại
    """
    left_pwm  = speed + turn
    right_pwm = speed - turn
    
    # Đảm bảo không vượt quá giới hạn -255 đến 255
    left_pwm  = max(MOTOR_PWM_MIN, min(MOTOR_PWM_MAX, int(left_pwm)))
    right_pwm = max(MOTOR_PWM_MIN, min(MOTOR_PWM_MAX, int(right_pwm)))
    
    return left_pwm, right_pwm


# =============================================================
# SHARED DATA
# =============================================================

data = {
    # GPS
    "lat": None, "lon": None,
    "bracelet_lat": None, "bracelet_lon": None,
    "web_lat": None, "web_lon": None,
    "target_source": "BRACELET", # "BRACELET" hoac "WEB"
    "distance": None,
    "gps_ok": False, "lora_ok": False,

    # IMU
    "heading": 0.0,
    "heading_filtered": 0.0,
    "heading_init": False,
    "roll": 0.0, "pitch": 0.0,
    "imu_sys_cal": 0, "imu_mag_cal": 0,
    "imu_ok": False,

    # Navigation output
    "bearing": 0.0, "error": 0.0,
    "speed": 0, "left_pwm": 0, "right_pwm": 0,
    "left_percent": 0, "right_percent": 0,
    "nav_mode": MODE_STOP,
    "user_mode": MODE_STOP,
    "auto_speed": 150,
    "turn_val": 0.0, # Giá trị lực rẽ của PID
}

data_lock = threading.Lock()
bus = None


# =============================================================
# CAN BUS: PARSE + READ
# =============================================================

def parse_can_message(msg):
    with data_lock:
        if msg.arbitration_id == CAN_ID_LOCAL_GPS and msg.dlc >= 8:
            lat_raw = struct.unpack_from('<i', msg.data, 0)[0]
            lon_raw = struct.unpack_from('<i', msg.data, 4)[0]
            if lat_raw != 0 or lon_raw != 0:
                data["lat"] = lat_raw / 1_000_000.0
                data["lon"] = lon_raw / 1_000_000.0

        elif msg.arbitration_id == CAN_ID_REMOTE_GPS and msg.dlc >= 8:
            lat_raw = struct.unpack_from('<i', msg.data, 0)[0]
            lon_raw = struct.unpack_from('<i', msg.data, 4)[0]
            if lat_raw != 0 or lon_raw != 0:
                data["bracelet_lat"] = lat_raw / 1_000_000.0
                data["bracelet_lon"] = lon_raw / 1_000_000.0

        elif msg.arbitration_id == CAN_ID_STATUS and msg.dlc >= 6:
            dist_cm = struct.unpack_from('<I', msg.data, 0)[0]
            data["distance"] = round(dist_cm / 100.0, 2)
            data["gps_ok"] = (msg.data[4] == 1)
            data["lora_ok"] = (msg.data[5] == 1)

        elif msg.arbitration_id == CAN_ID_IMU and msg.dlc >= 8:
            heading = struct.unpack_from('<h', msg.data, 0)[0] / 100.0
            roll    = struct.unpack_from('<h', msg.data, 2)[0] / 100.0
            pitch   = struct.unpack_from('<h', msg.data, 4)[0] / 100.0
            data["heading"] = round(heading, 2)
            data["roll"]    = round(roll, 2)
            data["pitch"]   = round(pitch, 2)
            data["imu_sys_cal"] = msg.data[6]
            data["imu_mag_cal"] = msg.data[7]
            data["imu_ok"] = True

            if not data["heading_init"]:
                data["heading_filtered"] = heading
                data["heading_init"] = True
            else:
                diff = heading - data["heading_filtered"]
                while diff > 180: diff -= 360
                while diff < -180: diff += 360
                data["heading_filtered"] = (data["heading_filtered"] + HEADING_ALPHA * diff) % 360

        elif msg.arbitration_id == 0x104 and msg.dlc >= 4:
            left_fb  = struct.unpack_from('<h', msg.data, 0)[0]
            right_fb = struct.unpack_from('<h', msg.data, 2)[0]
            data["left_percent"]  = max(0, min(100, int((left_fb - 1000) / 10.0)))
            data["right_percent"] = max(0, min(100, int((right_fb - 1000) / 10.0)))


def can_reader_thread():
    global bus
    print("📡 [CAN-RX] Dang lang nghe CAN bus...")
    frame_count = 0
    last_report = time.time()

    while True:
        try:
            message = bus.recv(timeout=1.0)
            if message is not None:
                parse_can_message(message)
                frame_count += 1

            now = time.time()
            if now - last_report >= 5.0:
                if frame_count > 0:
                    print(f"📊 [CAN-RX] {frame_count} frames trong 5s")
                else:
                    print("⚠️  [CAN-RX] KHONG NHAN DUOC FRAME NAO!")
                frame_count = 0
                last_report = now
        except Exception as e:
            print(f"❌ [CAN-RX] Loi: {e}")


# =============================================================
# CAN BUS: SEND MOTOR COMMAND
# =============================================================

def send_motor_command(left_pwm, right_pwm, mode):
    global bus
    if bus is None:
        return
    cmd_data = struct.pack('<hhB', int(left_pwm), int(right_pwm), int(mode))
    msg = can.Message(
        arbitration_id=CAN_ID_MOTOR_CMD,
        data=cmd_data,
        is_extended_id=False,
    )
    try:
        bus.send(msg, timeout=0.01)
    except can.CanError:
        pass


# =============================================================
# NAVIGATION THREAD — PID CONTROL (20Hz)
# =============================================================

def navigation_thread():
    pid = PIDController(
        Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
        integral_max=PID_INTEGRAL_MAX,
        output_max=PID_OUTPUT_MAX
    )
    
    loop_period = 1.0 / LOOP_RATE_HZ
    last_debug = 0.0

    print(f"🧭 [NAV] Advanced PID Navigation started @ {LOOP_RATE_HZ}Hz")
    print(f"🧭 [NAV] PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD} (Tuned for 30kg boat)")
    print(f"🧭 [NAV] Coast distance: {COAST_DISTANCE_M}m")

    while True:
        loop_start = time.time()

        with data_lock:
            user_mode   = data["user_mode"]
            lat         = data["lat"]
            lon         = data["lon"]
            target_src  = data["target_source"]
            if target_src == "WEB":
                target_lat = data["web_lat"]
                target_lon = data["web_lon"]
            else:
                target_lat = data["bracelet_lat"]
                target_lon = data["bracelet_lon"]
            heading     = data["heading_filtered"]
            gps_ok      = data["gps_ok"]
            imu_ok      = data["imu_ok"] and data["heading_init"]
            web_speed   = data["auto_speed"]

        has_local  = lat is not None and lon is not None
        has_remote = target_lat is not None and target_lon is not None

        # ===== LUON TINH KHOANG CACH & GOC NEU CO 2 GPS =====
        distance = 0.0
        bearing = 0.0
        error = 0.0
        if has_local and has_remote:
            distance = haversine_distance(lat, lon, target_lat, target_lon)
            bearing = calculate_bearing(lat, lon, target_lat, target_lon)
            error   = normalize_angle(bearing - heading)
            with data_lock:
                data["distance"] = round(distance, 2)
                data["bearing"] = round(bearing, 1)
                data["error"] = round(error, 1)

        # ----- MANUAL: tay cam RC dieu khien -----
        if user_mode == MODE_MANUAL:
            pid.reset()
            send_motor_command(0, 0, MODE_MANUAL)
            with data_lock:
                data["nav_mode"] = MODE_MANUAL
                data["speed"] = 0; data["left_pwm"] = 0; data["right_pwm"] = 0
                data["turn_val"] = 0.0
            if DEBUG_PRINT and (time.time() - last_debug) >= DEBUG_INTERVAL_S:
                last_debug = time.time()
                print(f"🧭 [NAV] MANUAL (RC) | Head={heading:.1f}° | Dist={distance:.1f}m | Err={error:+.1f}°")
            elapsed = time.time() - loop_start
            if elapsed < loop_period: time.sleep(loop_period - elapsed)
            continue

        # ----- STOP: dung motor -----
        if user_mode == MODE_STOP:
            pid.reset()
            send_motor_command(0, 0, MODE_STOP)
            with data_lock:
                data["nav_mode"] = MODE_STOP
                data["speed"] = 0; data["left_pwm"] = 0; data["right_pwm"] = 0
                data["turn_val"] = 0.0
            if DEBUG_PRINT and (time.time() - last_debug) >= DEBUG_INTERVAL_S:
                last_debug = time.time()
                print(f"🧭 [NAV] STOP | Head={heading:.1f}° | Dist={distance:.1f}m | Err={error:+.1f}°")
            elapsed = time.time() - loop_start
            if elapsed < loop_period: time.sleep(loop_period - elapsed)
            continue

        # ----- AUTO: kiem tra du lieu -----
        nav_ready  = has_local and has_remote and imu_ok

        if not nav_ready:
            pid.reset()
            send_motor_command(0, 0, MODE_STOP)
            with data_lock:
                data["nav_mode"] = MODE_STOP
                data["speed"] = 0; data["left_pwm"] = 0; data["right_pwm"] = 0
                data["turn_val"] = 0.0
            if DEBUG_PRINT and (time.time() - last_debug) >= DEBUG_INTERVAL_S:
                last_debug = time.time()
                missing = []
                if not has_local:  missing.append("NO_GPS")
                if not has_remote: missing.append("NO_VICTIM")
                if not imu_ok:     missing.append("NO_IMU")
                print(f"⏳ [NAV] WAITING | {' + '.join(missing)} | Head={heading:.1f}°")
            elapsed = time.time() - loop_start
            if elapsed < loop_period: time.sleep(loop_period - elapsed)
            continue

        # ===== XU LY DIEU KHIEN PID (AUTO) =====
        
        # ----- Kiem tra khoang cach: < 5m → tha troi -----
        if distance < COAST_DISTANCE_M:
            left_pwm, right_pwm = 0, 0
            nav_mode = MODE_STOP
            turn_val = 0.0
            speed = 0
            pid.reset()
        else:
            # Giam toc khi gan (< 20m)
            if distance < SLOWDOWN_DISTANCE:
                ratio = (distance - COAST_DISTANCE_M) / (SLOWDOWN_DISTANCE - COAST_DISTANCE_M)
                # Giữ mức độ tối thiểu để thuyền vẫn có khả năng bẻ lái
                speed = max(60, int(web_speed * ratio))
            else:
                speed = web_speed

            # Tính PID (Chỉ can thiệp khi lệch > 3 độ)
            if abs(error) < HEADING_DEADBAND:
                turn_val = 0.0
                pid.reset() # Reset integral khi đã đi đúng hướng
            else:
                turn_val = pid.compute(error)

            # Tính PWM 2 motor kết hợp Speed + Lực PID
            left_pwm, right_pwm = mix_motors(speed, turn_val)
            nav_mode = MODE_AUTO

        # Gui lenh motor xuong dongco.ino
        send_motor_command(left_pwm, right_pwm, nav_mode)

        # Cap nhat shared data cho phan motor
        with data_lock:
            data["speed"]     = speed
            data["left_pwm"]  = left_pwm
            data["right_pwm"] = right_pwm
            data["nav_mode"]  = nav_mode
            data["turn_val"]  = turn_val

        # Debug output
        if DEBUG_PRINT and (time.time() - last_debug) >= DEBUG_INTERVAL_S:
            last_debug = time.time()
            dir_arrow = "→" if turn_val > 0 else ("←" if turn_val < 0 else "↑")
            print(
                f"🧭 [NAV] PID | "
                f"Dist={distance:5.1f}m | "
                f"Err={error:+6.1f}° | "
                f"Turn={turn_val:+6.1f} {dir_arrow} | "
                f"Spd={speed:3d} | "
                f"L={left_pwm:+4d} R={right_pwm:+4d}"
            )

        elapsed = time.time() - loop_start
        if elapsed < loop_period:
            time.sleep(loop_period - elapsed)


# =============================================================
# MQTT
# =============================================================

def handle_command(topic, payload):
    try:
        if topic.endswith("mode"):
            mode_str = payload.decode().strip().upper()
            print(f"📥 [MQTT] Mode: {mode_str}")
            with data_lock:
                if mode_str in ("0", "STOP"):
                    data["user_mode"] = MODE_STOP
                elif mode_str in ("1", "AUTO"):
                    data["user_mode"] = MODE_AUTO
                elif mode_str in ("2", "MANUAL"):
                    data["user_mode"] = MODE_MANUAL

        elif topic.endswith("speed"):
            spd = max(0, min(255, int(payload.decode().strip())))
            with data_lock:
                data["auto_speed"] = spd
            print(f"🏎️  [MQTT] Speed = {spd}/255")

        elif topic.endswith("mission"):
            mission_data = json.loads(payload.decode())
            mission = mission_data.get("mission", [])
            if len(mission) > 0:
                with data_lock:
                    data["web_lat"] = mission[0].get("lat")
                    data["web_lon"] = mission[0].get("lng")
                    data["target_source"] = "WEB"
                print(f"📍 [MQTT] Nhan diem den tu Web: Lat={data['web_lat']}, Lon={data['web_lon']}")
            else:
                with data_lock:
                    data["target_source"] = "BRACELET"
                print("📍 [MQTT] Xoa diem Web, tro ve bam theo Vong tay!")

    except Exception as e:
        print(f"❌ [MQTT] Error: {e}")


def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ [MQTT] Connected!")
        client.subscribe(TOPIC_COMMAND)
    else:
        print(f"❌ [MQTT] Connect failed: {rc}")


def on_disconnect(client, userdata, rc):
    print("⚠️  [MQTT] Disconnected!")


def on_message(client, userdata, msg):
    handle_command(msg.topic, msg.payload)


# =============================================================
# MAIN
# =============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  🚤 USV — ADVANCED PID NAVIGATION (b5c.py)")
    print("  Tuned for: 20kg Boat")
    print("=" * 60)

    # 1. CAN Bus
    try:
        bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE)
        print(f"✅ [CAN] Connected: {CAN_CHANNEL}")
    except Exception as e:
        print(f"❌ [CAN] Error: {e}")
        print(f"   Run: sudo ip link set {CAN_CHANNEL} up type can bitrate {CAN_BITRATE}")
        exit(1)

    # 2. CAN reader thread
    threading.Thread(target=can_reader_thread, daemon=True).start()

    # 3. Navigation thread
    threading.Thread(target=navigation_thread, daemon=True).start()

    # 4. MQTT
    mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message

    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
    except Exception as e:
        print(f"❌ [MQTT] Error: {e}")
        exit(1)

    # 5. Main loop: Publish MQTT moi 1 giay
    print("\n🏁 READY!\n")

    try:
        while True:
            with data_lock:
                d = dict(data)

            mode_names = {MODE_STOP: "STOP", MODE_AUTO: "AUTO", MODE_MANUAL: "MANUAL"}

            # Status
            status_msg = {
                "online": True,
                "gps_ok": d["gps_ok"],
                "lora_ok": d["lora_ok"],
                "imu_ok": d["imu_ok"],
                "imu_cal": f"S{d['imu_sys_cal']}M{d['imu_mag_cal']}",
                "nav_mode": mode_names.get(d["nav_mode"], "STOP"),
                "heading": d["heading"],
                "left_motor": d["left_percent"],
                "right_motor": d["right_percent"],
                "timestamp": int(time.time()),
            }
            mqtt_client.publish(TOPIC_STATUS, json.dumps(status_msg))

            # Telemetry
            if d["lat"] is not None and d["lon"] is not None:
                telemetry = {
                    "lat": d["lat"], "lon": d["lon"],
                    "heading": d["heading"],
                    "bearing": d["bearing"],
                    "error": d["error"],
                    "distance": d.get("distance"),
                    "speed": d["speed"],
                    "left_pwm": d["left_pwm"],
                    "right_pwm": d["right_pwm"],
                    "left_motor": d["left_percent"],
                    "right_motor": d["right_percent"],
                    "nav_mode": mode_names.get(d["nav_mode"], "STOP"),
                }
                with data_lock:
                    t_lat = data["web_lat"] if data["target_source"] == "WEB" else data["bracelet_lat"]
                    t_lon = data["web_lon"] if data["target_source"] == "WEB" else data["bracelet_lon"]
                
                if t_lat is not None:
                    telemetry["remote_lat"] = t_lat
                    telemetry["remote_lon"] = t_lon

                mqtt_client.publish(TOPIC_TELEMETRY, json.dumps(telemetry))

                print(
                    f"📡 [MQTT] Lat={d['lat']:.6f} Lon={d['lon']:.6f} | "
                    f"Head={d['heading']:.1f}° | "
                    f"Bear={d['bearing']:.1f}° | "
                    f"Err={d['error']:+.1f}° | "
                    f"Dist={d.get('distance', 0)}m | "
                    f"L={d['left_pwm']:+d} R={d['right_pwm']:+d} | "
                    f"GPS:{'OK' if d['gps_ok'] else 'NO'} "
                    f"LoRa:{'OK' if d['lora_ok'] else 'NO'} "
                    f"IMU:{'OK' if d['imu_ok'] else 'NO'}"
                )
            else:
                print("⏳ [MQTT] Cho GPS...")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\n🛑 DUNG...")
        send_motor_command(0, 0, MODE_STOP)
        time.sleep(0.1)
        send_motor_command(0, 0, MODE_STOP)
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        bus.shutdown()
        print("✅ Thoat an toan.")
