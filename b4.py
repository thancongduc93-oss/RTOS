#!/usr/bin/env python3
"""
==========================================================
  USV BOAT — ALL-IN-ONE: CAN + Navigation + PID + MQTT
==========================================================
Chay tren: RASPBERRY PI (tren tau)

Chuc nang:
  1. Nhan GPS thuyen + GPS vong tay tu CAN bus (ESP32C3 Receiver)
  2. Nhan heading tu BNO055 qua CAN bus (ESP32C3 Motor Controller)
  3. Tinh toan navigation: bearing, error, PID
  4. Gui lenh motor qua CAN bus → ESP32C3 Autopilot
  5. Day telemetry + status len MQTT → Web Dashboard
  6. Nhan lenh waypoint + start tu Web qua MQTT

CAN Protocol:
  0x101: GPS Thuyen     (lat, lon int32 x1M)        ← ESP32 Receiver
  0x102: GPS Vong Tay   (lat, lon int32 x1M)        ← ESP32 Receiver
  0x103: Status         (dist_cm, gps_ok, lora_ok)  ← ESP32 Receiver
  0x104: BNO055 IMU     (heading, roll, pitch x100)  ← ESP32 Motor
  0x201: Motor Command  (left_pwm, right_pwm, mode)  → ESP32 Motor

MQTT Topics:
  usv/boat1/telemetry        → Gui toa do + heading + nav data
  usv/boat1/status           → Gui trang thai (heartbeat)
  usv/boat1/command/mission  ← Nhan Waypoint tu Web
  usv/boat1/command/start    ← Nhan lenh chay tu Web

Cach chay:
  sudo ip link set can0 up type can bitrate 500000
  sudo python3 b4.py
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
CAN_ID_LOCAL_GPS  = 0x101   # GPS thuyen (lat, lon)
CAN_ID_REMOTE_GPS = 0x102   # GPS vong tay (lat, lon)
CAN_ID_STATUS     = 0x103   # Distance(cm) + gps_ok + lora_ok
CAN_ID_IMU        = 0x100   # Heading, Roll, Pitch (x100) + Cal

# CAN IDs — Gui toi ESP32
CAN_ID_MOTOR_CMD  = 0x201   # left_pwm + right_pwm + mode
CAN_ID_WAYPOINT   = 0x202   # Gui waypoint xuong ESP32 (du phong)
CAN_ID_CMD_START  = 0x203   # Gui lenh Start xuong ESP32 (du phong)

# ----- MQTT -----
MQTT_BROKER     = "broker.emqx.io"
MQTT_PORT       = 1883
MQTT_CLIENT_ID  = "usv_pi_allinone_001"
TOPIC_TELEMETRY = "usv/boat1/telemetry"
TOPIC_STATUS    = "usv/boat1/status"
TOPIC_COMMAND   = "usv/boat1/command/#"

# ----- PID -----
PID_KP = 2.0
PID_KI = 0.05
PID_KD = 1.0
PID_INTEGRAL_MAX = 100.0
PID_OUTPUT_MAX   = 200.0

# ----- Speed Zones (nguong_met, toc_do_PWM) -----
SPEED_ZONES = [
    (20.0, 200),   # Xa > 20m   → max
    (10.0, 150),   # 10-20m     → vua
    (5.0,  120),   # 5-10m      → giam
    (2.0,   80),   # 2-5m       → cham
    (0.0,    0),   # < 2m       → dung
]

# ----- Motor -----
MOTOR_PWM_MIN = -255
MOTOR_PWM_MAX = 255

# ----- Navigation -----
LOOP_RATE_HZ     = 20      # 20Hz = 50ms/loop
ARRIVAL_RADIUS_M = 2.0     # < 2m → coi la da toi
GPS_TIMEOUT_S    = 5.0     # Mat CAN > 5s → dung motor

# ----- Modes -----
MODE_STOP   = 0
MODE_AUTO   = 1
MODE_MANUAL = 2

# ----- Debug -----
DEBUG_PRINT      = True
DEBUG_INTERVAL_S = 0.5


# =============================================================
# PID CONTROLLER
# =============================================================

class PIDController:
    """
    PID Controller voi anti-windup va output clamping.
    Input:  error (sai lech goc, -180 den +180)
    Output: turn  (gia tri steering, -max den +max)
    """

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
        """Tinh output PID tu sai lech goc."""
        now = time.time()

        if self._prev_time is None:
            self._prev_time = now
            self._prev_error = error
            output = self.Kp * error
            return self._clamp(output)

        dt = now - self._prev_time
        if dt <= 0:
            dt = 0.001

        # Proportional
        P = self.Kp * error

        # Integral (anti-windup)
        self._integral += error * dt
        self._integral = max(-self.integral_max,
                             min(self.integral_max, self._integral))
        I = self.Ki * self._integral

        # Derivative
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
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_phi = math.radians(lat2 - lat1)
    d_lambda = math.radians(lon2 - lon1)

    a = (math.sin(d_phi / 2) ** 2 +
         math.cos(phi1) * math.cos(phi2) * math.sin(d_lambda / 2) ** 2)
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    return R * c


def calculate_bearing(lat1, lon1, lat2, lon2):
    """Tinh goc bearing tu diem 1 toi diem 2. Returns 0-360."""
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    d_lambda = math.radians(lon2 - lon1)

    y = math.sin(d_lambda) * math.cos(phi2)
    x = (math.cos(phi1) * math.sin(phi2) -
         math.sin(phi1) * math.cos(phi2) * math.cos(d_lambda))

    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360) % 360


def normalize_angle(angle):
    """Chuan hoa goc ve -180 den +180."""
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def get_speed_for_distance(distance):
    """Lay toc do PWM dua tren khoang cach."""
    for threshold, speed in SPEED_ZONES:
        if distance >= threshold:
            return speed
    return 0


def mix_motors(speed, turn):
    """
    Tinh PWM cho 2 motor (differential drive).
    speed: toc do tien (0-255)
    turn:  steering (am=trai, duong=phai)
    Returns: (left_pwm, right_pwm)
    """
    left_pwm = speed - turn
    right_pwm = speed + turn
    left_pwm = max(MOTOR_PWM_MIN, min(MOTOR_PWM_MAX, int(left_pwm)))
    right_pwm = max(MOTOR_PWM_MIN, min(MOTOR_PWM_MAX, int(right_pwm)))
    return left_pwm, right_pwm


# =============================================================
# SHARED DATA (giua cac thread)
# =============================================================

data = {
    # GPS tu ESP32 Receiver (CAN 0x101, 0x102, 0x103)
    "lat": None,
    "lon": None,
    "remote_lat": None,
    "remote_lon": None,
    "distance": None,
    "gps_ok": False,
    "lora_ok": False,

    # BNO055 tu ESP32 Motor (CAN 0x104)
    "heading": 0.0,
    "roll": 0.0,
    "pitch": 0.0,
    "imu_sys_cal": 0,
    "imu_mag_cal": 0,
    "imu_ok": False,

    # Navigation output
    "bearing": 0.0,
    "error": 0.0,
    "turn": 0.0,
    "speed": 0,
    "left_pwm": 0,
    "right_pwm": 0,
    "nav_mode": MODE_STOP,
    "user_mode": MODE_MANUAL,
}

data_lock = threading.Lock()
bus = None  # CAN Bus global


# =============================================================
# CAN BUS: PARSE + READ THREAD
# =============================================================

def parse_can_message(msg):
    """Giai ma cac goi tin CAN tu ESP32."""
    with data_lock:
        # 0x101: GPS Thuyen (lat, lon int32 x 1,000,000)
        if msg.arbitration_id == CAN_ID_LOCAL_GPS and msg.dlc >= 8:
            lat_raw = struct.unpack_from('<i', msg.data, 0)[0]
            lon_raw = struct.unpack_from('<i', msg.data, 4)[0]
            if lat_raw != 0 or lon_raw != 0:
                data["lat"] = lat_raw / 1_000_000.0
                data["lon"] = lon_raw / 1_000_000.0

        # 0x102: GPS Vong tay (lat, lon int32 x 1,000,000)
        elif msg.arbitration_id == CAN_ID_REMOTE_GPS and msg.dlc >= 8:
            lat_raw = struct.unpack_from('<i', msg.data, 0)[0]
            lon_raw = struct.unpack_from('<i', msg.data, 4)[0]
            if lat_raw != 0 or lon_raw != 0:
                data["remote_lat"] = lat_raw / 1_000_000.0
                data["remote_lon"] = lon_raw / 1_000_000.0

        # 0x103: Khoang cach (uint32 cm) + gps_ok + lora_ok
        elif msg.arbitration_id == CAN_ID_STATUS and msg.dlc >= 6:
            dist_cm = struct.unpack_from('<I', msg.data, 0)[0]
            data["distance"] = round(dist_cm / 100.0, 2)
            data["gps_ok"] = (msg.data[4] == 1)
            data["lora_ok"] = (msg.data[5] == 1)

        # 0x104: BNO055 IMU (heading, roll, pitch x100 + cal)
        elif msg.arbitration_id == CAN_ID_IMU and msg.dlc >= 8:
            heading = struct.unpack_from('<h', msg.data, 0)[0] / 100.0
            roll = struct.unpack_from('<h', msg.data, 2)[0] / 100.0
            pitch = struct.unpack_from('<h', msg.data, 4)[0] / 100.0
            data["heading"] = round(heading, 2)
            data["roll"] = round(roll, 2)
            data["pitch"] = round(pitch, 2)
            data["imu_sys_cal"] = msg.data[6]
            data["imu_mag_cal"] = msg.data[7]
            data["imu_ok"] = True


def can_reader_thread():
    """Thread doc lien tuc du lieu tu CAN Bus."""
    global bus
    print("📡 [CAN-RX] Dang lang nghe CAN bus...")
    while True:
        try:
            message = bus.recv(timeout=1.0)
            if message is not None:
                parse_can_message(message)
        except Exception:
            pass


# =============================================================
# CAN BUS: SEND MOTOR COMMAND
# =============================================================

def send_motor_command(left_pwm, right_pwm, mode):
    """Gui lenh motor qua CAN bus (ID: 0x201)."""
    global bus
    if bus is None:
        return

    # Pack: int16 left + int16 right + uint8 mode = 5 bytes
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
# NAVIGATION AUTOPILOT THREAD (20Hz)
# =============================================================

def navigation_thread():
    """
    Vong lap autopilot chinh:
    Doc GPS + heading → Tinh bearing → PID → Gui motor command
    Chay o 20Hz (50ms/loop)
    """
    pid = PIDController(
        Kp=PID_KP, Ki=PID_KI, Kd=PID_KD,
        integral_max=PID_INTEGRAL_MAX,
        output_max=PID_OUTPUT_MAX,
    )

    loop_period = 1.0 / LOOP_RATE_HZ
    last_can_time = time.time()
    last_debug_time = 0.0

    print(f"🧭 [NAV] Autopilot started @ {LOOP_RATE_HZ}Hz")
    print(f"🧭 [NAV] PID: Kp={PID_KP}, Ki={PID_KI}, Kd={PID_KD}")
    print(f"🧭 [NAV] Arrival radius: {ARRIVAL_RADIUS_M}m")

    while True:
        loop_start = time.time()

        with data_lock:
            user_mode = data["user_mode"]
            lat = data["lat"]
            lon = data["lon"]
            remote_lat = data["remote_lat"]
            remote_lon = data["remote_lon"]
            heading = data["heading"]
            gps_ok = data["gps_ok"]
            lora_ok = data["lora_ok"]
            imu_ok = data["imu_ok"]
            distance_from_esp = data["distance"]
            imu_cal = (data["imu_sys_cal"], data["imu_mag_cal"])

        # ----- Kiem tra che do cua user -----
        if user_mode == MODE_MANUAL:
            pid.reset()
            send_motor_command(0, 0, MODE_MANUAL)
            with data_lock:
                data["nav_mode"] = MODE_MANUAL
                data["bearing"] = 0.0
                data["error"] = 0.0
                data["turn"] = 0.0
                data["speed"] = 0
                data["left_pwm"] = 0
                data["right_pwm"] = 0
            if DEBUG_PRINT and (time.time() - last_debug_time) >= DEBUG_INTERVAL_S:
                last_debug_time = time.time()
                print(f"🧭 [NAV] MODE: MANUAL (RC Control) | Head={heading:.1f}°")
            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)
            continue

        elif user_mode == MODE_STOP:
            pid.reset()
            send_motor_command(0, 0, MODE_STOP)
            with data_lock:
                data["nav_mode"] = MODE_STOP
                data["bearing"] = 0.0
                data["error"] = 0.0
                data["turn"] = 0.0
                data["speed"] = 0
                data["left_pwm"] = 0
                data["right_pwm"] = 0
            if DEBUG_PRINT and (time.time() - last_debug_time) >= DEBUG_INTERVAL_S:
                last_debug_time = time.time()
                print(f"🧭 [NAV] MODE: STOP | Head={heading:.1f}°")
            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)
            continue

        # ----- Neu o che do AUTO, kiem tra du lieu hop le -----
        has_local = lat is not None and lon is not None
        has_remote = remote_lat is not None and remote_lon is not None
        nav_ready = has_local and has_remote and imu_ok and distance_from_esp is not None

        if not nav_ready:
            # Chua du du lieu → dung motor, reset PID
            pid.reset()
            send_motor_command(0, 0, MODE_STOP)

            with data_lock:
                data["nav_mode"] = MODE_STOP
                data["bearing"] = 0.0
                data["error"] = 0.0
                data["turn"] = 0.0
                data["speed"] = 0
                data["left_pwm"] = 0
                data["right_pwm"] = 0

            if DEBUG_PRINT and (time.time() - last_debug_time) >= DEBUG_INTERVAL_S:
                last_debug_time = time.time()
                missing = []
                if not has_local:
                    missing.append("NO_LOCAL_GPS")
                if not has_remote:
                    missing.append("NO_REMOTE_GPS")
                if not imu_ok:
                    missing.append("NO_IMU")
                if distance_from_esp is None:
                    missing.append("NO_DISTANCE")
                print(f"⏳ [NAV] WAITING | {' + '.join(missing)} | Head={heading:.1f}°")

            # Rate limiting
            elapsed = time.time() - loop_start
            if elapsed < loop_period:
                time.sleep(loop_period - elapsed)
            continue

        # ----- Tinh toan navigation (AUTO) -----
        distance = distance_from_esp

        bearing = calculate_bearing(lat, lon, remote_lat, remote_lon)
        error = normalize_angle(bearing - heading)
        turn = pid.compute(error)
        speed = get_speed_for_distance(distance)

        # Kiem tra da toi noi chua
        if distance < ARRIVAL_RADIUS_M:
            left_pwm, right_pwm = 0, 0
            nav_mode = MODE_STOP
            pid.reset()
        else:
            left_pwm, right_pwm = mix_motors(speed, turn)
            nav_mode = MODE_AUTO

        # ----- Gui lenh motor -----
        send_motor_command(left_pwm, right_pwm, nav_mode)

        # ----- Cap nhat shared data -----
        with data_lock:
            data["bearing"] = round(bearing, 1)
            data["error"] = round(error, 1)
            data["turn"] = round(turn, 1)
            data["speed"] = speed
            data["left_pwm"] = left_pwm
            data["right_pwm"] = right_pwm
            data["nav_mode"] = nav_mode

        # ----- Debug output -----
        if DEBUG_PRINT and (time.time() - last_debug_time) >= DEBUG_INTERVAL_S:
            last_debug_time = time.time()
            dir_arrow = "→R" if error > 5 else ("←L" if error < -5 else "↑OK")
            print(
                f"🧭 [NAV] AUTO | "
                f"Dist={distance:6.1f}m | "
                f"Bear={bearing:5.1f}° | "
                f"Head={heading:5.1f}° | "
                f"Err={error:+6.1f}° {dir_arrow} | "
                f"Spd={speed:3d} | "
                f"L={left_pwm:+4d} R={right_pwm:+4d} | "
                f"Cal=S{imu_cal[0]}M{imu_cal[1]}"
            )

        # ----- Rate limiting -----
        elapsed = time.time() - loop_start
        if elapsed < loop_period:
            time.sleep(loop_period - elapsed)


# =============================================================
# MQTT: NHAN LENH TU WEB
# =============================================================

def handle_command(topic, payload):
    """Xu ly lenh nhan tu Web qua MQTT."""
    try:
        if topic.endswith("start"):
            print("🚀 [MQTT] Nhan lenh START tu Web!")
            # Gui lenh start xuong ESP32 (du phong)
            if bus is not None:
                msg = can.Message(
                    arbitration_id=CAN_ID_CMD_START,
                    data=b'\x01',
                    is_extended_id=False,
                )
                bus.send(msg)

        elif topic.endswith("mission"):
            mission_data = json.loads(payload.decode())
            mission = mission_data.get("mission", [])
            print(f"📥 [MQTT] Nhan duoc {len(mission)} diem tu Web!")
            # Gui waypoints xuong ESP32 (du phong)
            send_waypoints_to_esp32(mission)

        elif topic.endswith("mode"):
            mode_str = payload.decode().strip().upper()
            print(f"📥 [MQTT] Nhan lenh doi MODE tu Web: {mode_str}")
            with data_lock:
                if mode_str in ("0", "STOP"):
                    data["user_mode"] = MODE_STOP
                elif mode_str in ("1", "AUTO"):
                    data["user_mode"] = MODE_AUTO
                elif mode_str in ("2", "MANUAL"):
                    data["user_mode"] = MODE_MANUAL

    except Exception as e:
        print(f"❌ [MQTT] Command error: {e}")


def send_waypoints_to_esp32(mission):
    """Gui danh sach waypoint xuong ESP32 qua CAN Bus (du phong)."""
    global bus
    if bus is None:
        return

    for seq, wp in enumerate(mission):
        lat = int(wp["lat"] * 1e6)
        lon = int(wp["lng"] * 1e6)

        data1 = struct.pack('<hI', seq, lat & 0xFFFFFFFF) + b'\x01\x00'
        msg1 = can.Message(arbitration_id=CAN_ID_WAYPOINT,
                           data=data1, is_extended_id=False)
        bus.send(msg1)
        time.sleep(0.05)

        data2 = struct.pack('<hI', seq, lon & 0xFFFFFFFF) + b'\x02\x00'
        msg2 = can.Message(arbitration_id=CAN_ID_WAYPOINT,
                           data=data2, is_extended_id=False)
        bus.send(msg2)

        print(f"  ✅ WP {seq+1}: lat={wp['lat']:.6f}, lon={wp['lng']:.6f}")
        time.sleep(0.1)

    print(f"🎉 [MQTT] Da gui xong {len(mission)} waypoint!")


# =============================================================
# MQTT SETUP
# =============================================================

def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("✅ [MQTT] Da ket noi Broker thanh cong!")
        client.subscribe(TOPIC_COMMAND)
        print(f"✅ [MQTT] Dang lang nghe lenh tai: {TOPIC_COMMAND}")
    else:
        print(f"❌ [MQTT] Ket noi that bai, ma loi: {rc}")


def on_disconnect(client, userdata, rc):
    print("⚠️  [MQTT] Mat ket noi Broker! Dang thu ket noi lai...")


def on_message(client, userdata, msg):
    handle_command(msg.topic, msg.payload)


# =============================================================
# MAIN
# =============================================================

if __name__ == "__main__":
    print("=" * 60)
    print("  🚤 USV BOAT — ALL-IN-ONE")
    print("  CAN Bus + Navigation + PID + MQTT")
    print("=" * 60)
    print()

    # --- 1. Ket noi CAN Bus ---
    try:
        bus = can.interface.Bus(channel=CAN_CHANNEL, interface=CAN_INTERFACE)
        print(f"✅ [CAN] Da ket noi '{CAN_CHANNEL}' thanh cong!")
    except Exception as e:
        print(f"❌ [CAN] Loi ket noi: {e}")
        print(f"   Chay: sudo ip link set {CAN_CHANNEL} up type can bitrate {CAN_BITRATE}")
        exit(1)

    # --- 2. Khoi dong CAN reader thread ---
    threading.Thread(target=can_reader_thread, daemon=True).start()

    # --- 3. Khoi dong Navigation autopilot thread ---
    threading.Thread(target=navigation_thread, daemon=True).start()

    # --- 4. Khoi dong MQTT ---
    mqtt_client = mqtt.Client(client_id=MQTT_CLIENT_ID)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_disconnect = on_disconnect
    mqtt_client.on_message = on_message

    try:
        mqtt_client.connect(MQTT_BROKER, MQTT_PORT, 60)
        mqtt_client.loop_start()
    except Exception as e:
        print(f"❌ [MQTT] Khong the ket noi Broker: {e}")
        print("   Kiem tra ket noi Internet (WiFi hoac 4G)")
        exit(1)

    # --- 5. Vong lap chinh: Publish MQTT moi 1 giay ---
    print("\n🏁 HE THONG SAN SANG! Navigation + MQTT dang chay...\n")

    try:
        while True:
            with data_lock:
                d = dict(data)  # Copy snapshot

            # === Gui heartbeat status ===
            mode_names = {MODE_STOP: "STOP", MODE_AUTO: "AUTO", MODE_MANUAL: "MANUAL"}
            status_msg = {
                "online": True,
                "gps_ok": d["gps_ok"],
                "lora_ok": d["lora_ok"],
                "imu_ok": d["imu_ok"],
                "imu_cal": f"S{d['imu_sys_cal']}M{d['imu_mag_cal']}",
                "nav_mode": mode_names.get(d["nav_mode"], "STOP"),
                "heading": d["heading"],
                "timestamp": int(time.time()),
            }
            mqtt_client.publish(TOPIC_STATUS, json.dumps(status_msg))

            # === Gui Telemetry (toa do + heading + nav data) ===
            if d["lat"] is not None and d["lon"] is not None:
                telemetry = {
                    "lat": d["lat"],
                    "lon": d["lon"],
                    "heading": d["heading"],
                    "bearing": d["bearing"],
                    "error": d["error"],
                    "distance": d["distance"],
                    "speed": d["speed"],
                    "left_pwm": d["left_pwm"],
                    "right_pwm": d["right_pwm"],
                    "nav_mode": mode_names.get(d["nav_mode"], "STOP"),
                }

                if d["remote_lat"] is not None:
                    telemetry["remote_lat"] = d["remote_lat"]
                    telemetry["remote_lon"] = d["remote_lon"]

                mqtt_client.publish(TOPIC_TELEMETRY, json.dumps(telemetry))

                print(
                    f"📡 [MQTT] Lat={d['lat']:.6f} Lon={d['lon']:.6f} | "
                    f"Head={d['heading']:.1f}° | "
                    f"Bear={d['bearing']:.1f}° | "
                    f"Err={d['error']:+.1f}° | "
                    f"Dist={d['distance']}m | "
                    f"L={d['left_pwm']:+d} R={d['right_pwm']:+d} | "
                    f"GPS:{'OK' if d['gps_ok'] else 'NO'} "
                    f"LoRa:{'OK' if d['lora_ok'] else 'NO'} "
                    f"IMU:{'OK' if d['imu_ok'] else 'NO'}"
                )
            else:
                print("⏳ [MQTT] Dang cho GPS tu ESP32...")

            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n🛑 DUNG HE THONG...")
        # Safety: dung motor truoc khi thoat
        send_motor_command(0, 0, MODE_STOP)
        time.sleep(0.1)
        send_motor_command(0, 0, MODE_STOP)
        mqtt_client.loop_stop()
        mqtt_client.disconnect()
        bus.shutdown()
        print("✅ Da thoat an toan.")
