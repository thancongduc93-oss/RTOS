#!/usr/bin/env python3
"""
==========================================================
  USV BOAT — SIMPLE NAVIGATION (No PID)
  b4c.py — Dieu khien don gian theo vung goc
==========================================================
Chay tren: RASPBERRY PI

Logic dieu khien:
  - |error| < 20°  → Chay thang (2 motor bang nhau)
  - 20° ≤ |error| < 50° → Queo nhe (motor trong cham hon)
  - |error| ≥ 50°  → Queo manh (motor trong cham han)
  - Khoang cach < 5m → Dung motor, tha troi quan tinh

CAN Protocol (dong bo voi dongco.ino):
  0x101: GPS Thuyen     (lat, lon int32 x1M)        ← ESP32 Receiver
  0x102: GPS Vong Tay   (lat, lon int32 x1M)        ← ESP32 Receiver
  0x103: Status         (dist_cm, gps_ok, lora_ok)  ← ESP32 Receiver
  0x100: BNO055 IMU     (heading, roll, pitch x100)  ← ESP32 Motor
  0x104: Motor Feedback (left_us, right_us)          ← ESP32 Motor
  0x201: Motor Command  (left_pwm, right_pwm, mode)  → ESP32 Motor

Cach chay:
  sudo ip link set can0 up type can bitrate 500000
  sudo python3 b4c.py
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
MQTT_CLIENT_ID  = "usv_pi_b4c_001"
TOPIC_TELEMETRY = "usv/boat1/telemetry"
TOPIC_STATUS    = "usv/boat1/status"
TOPIC_COMMAND   = "usv/boat1/command/#"

# ----- Navigation Zones -----
DEADBAND_DEG      = 20.0    # < 20° → chay thang
MILD_TURN_DEG     = 50.0    # 20-50° → queo nhe
                             # >= 50° → queo manh

# ----- Turn Strength (he so giam toc motor trong) -----
MILD_TURN_FACTOR  = 0.55    # Queo nhe: motor trong = 55% toc do
HARD_TURN_FACTOR  = 0.15    # Queo manh: motor trong = 15% toc do

# ----- Distance -----
COAST_DISTANCE_M  = 5.0     # < 5m → tha troi, dung motor
SLOWDOWN_DISTANCE = 20.0    # < 20m → giam toc dan

# ----- Heading Filter -----
HEADING_ALPHA     = 0.25    # EMA filter: 0=smooth, 1=raw

# ----- Motor -----
MOTOR_PWM_MAX = 255

# ----- Modes -----
MODE_STOP   = 0
MODE_AUTO   = 1
MODE_MANUAL = 2

# ----- Loop -----
LOOP_RATE_HZ     = 20
DEBUG_PRINT      = True
DEBUG_INTERVAL_S = 0.5


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
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle


def compute_motor_pwm(error, speed):
    """
    Tinh PWM cho 2 motor dua tren goc lech (error).
    error > 0 → can queo PHAI → motor TRAI manh hon
    error < 0 → can queo TRAI → motor PHAI manh hon

    Returns: (left_pwm, right_pwm)
    """
    abs_err = abs(error)

    if abs_err < DEADBAND_DEG:
        # ===== VUNG 1: Chay thang — 2 motor bang nhau =====
        return speed, speed

    elif abs_err < MILD_TURN_DEG:
        # ===== VUNG 2: Queo nhe — motor trong giam nhe =====
        inner_speed = int(speed * MILD_TURN_FACTOR)
        if error > 0:
            # Queo phai → motor trai manh, motor phai giam
            return speed, inner_speed
        else:
            # Queo trai → motor phai manh, motor trai giam
            return inner_speed, speed

    else:
        # ===== VUNG 3: Queo manh — motor trong giam nhieu =====
        inner_speed = int(speed * HARD_TURN_FACTOR)
        if error > 0:
            # Queo phai → motor trai manh, motor phai giam manh
            return speed, inner_speed
        else:
            # Queo trai → motor phai manh, motor trai giam manh
            return inner_speed, speed


# =============================================================
# SHARED DATA
# =============================================================

data = {
    # GPS
    "lat": None, "lon": None,
    "remote_lat": None, "remote_lon": None,
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
    "turn_zone": "STRAIGHT",
}

data_lock = threading.Lock()
bus = None


# =============================================================
# CAN BUS: PARSE + READ
# =============================================================

def parse_can_message(msg):
    """Giai ma cac goi tin CAN tu ESP32."""
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
                data["remote_lat"] = lat_raw / 1_000_000.0
                data["remote_lon"] = lon_raw / 1_000_000.0

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

            # EMA filter heading (chong nhay vot)
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
    """Thread doc lien tuc CAN bus."""
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
    """Gui lenh motor qua CAN bus (ID: 0x201) → dongco.ino."""
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
# NAVIGATION THREAD — SIMPLE ZONE-BASED (20Hz)
# =============================================================

def navigation_thread():
    """
    Vong lap dieu khien don gian:
    - Tinh bearing va error
    - Dua tren vung goc, chon toc do 2 motor
    - Khong dung PID
    """
    loop_period = 1.0 / LOOP_RATE_HZ
    last_debug = 0.0

    print(f"🧭 [NAV] Simple Navigation started @ {LOOP_RATE_HZ}Hz")
    print(f"🧭 [NAV] Zones: STRAIGHT <{DEADBAND_DEG}° | MILD <{MILD_TURN_DEG}° | HARD ≥{MILD_TURN_DEG}°")
    print(f"🧭 [NAV] Coast distance: {COAST_DISTANCE_M}m")

    while True:
        loop_start = time.time()

        with data_lock:
            user_mode   = data["user_mode"]
            lat         = data["lat"]
            lon         = data["lon"]
            remote_lat  = data["remote_lat"]
            remote_lon  = data["remote_lon"]
            heading     = data["heading_filtered"]
            gps_ok      = data["gps_ok"]
            imu_ok      = data["imu_ok"] and data["heading_init"]
            dist_esp    = data["distance"]
            web_speed   = data["auto_speed"]

        has_local  = lat is not None and lon is not None
        has_remote = remote_lat is not None and remote_lon is not None

        # ===== LUON TINH KHOANG CACH & GOC NEU CO 2 GPS =====
        distance = 0.0
        bearing = 0.0
        error = 0.0
        if has_local and has_remote:
            distance = haversine_distance(lat, lon, remote_lat, remote_lon)
            bearing = calculate_bearing(lat, lon, remote_lat, remote_lon)
            error   = normalize_angle(bearing - heading)
            with data_lock:
                data["distance"] = round(distance, 2)
                data["bearing"] = round(bearing, 1)
                data["error"] = round(error, 1)

        # ----- MANUAL: tay cam RC dieu khien -----
        if user_mode == MODE_MANUAL:
            send_motor_command(0, 0, MODE_MANUAL)
            with data_lock:
                data["nav_mode"] = MODE_MANUAL
                data["speed"] = 0; data["left_pwm"] = 0; data["right_pwm"] = 0
                data["turn_zone"] = "RC"
            if DEBUG_PRINT and (time.time() - last_debug) >= DEBUG_INTERVAL_S:
                last_debug = time.time()
                print(f"🧭 [NAV] MANUAL (RC) | Head={heading:.1f}° | Dist={distance:.1f}m | Err={error:+.1f}°")
            elapsed = time.time() - loop_start
            if elapsed < loop_period: time.sleep(loop_period - elapsed)
            continue

        # ----- STOP: dung motor -----
        if user_mode == MODE_STOP:
            send_motor_command(0, 0, MODE_STOP)
            with data_lock:
                data["nav_mode"] = MODE_STOP
                data["speed"] = 0; data["left_pwm"] = 0; data["right_pwm"] = 0
                data["turn_zone"] = "STOP"
            if DEBUG_PRINT and (time.time() - last_debug) >= DEBUG_INTERVAL_S:
                last_debug = time.time()
                print(f"🧭 [NAV] STOP | Head={heading:.1f}° | Dist={distance:.1f}m | Err={error:+.1f}°")
            elapsed = time.time() - loop_start
            if elapsed < loop_period: time.sleep(loop_period - elapsed)
            continue

        # ----- AUTO: kiem tra du lieu -----
        nav_ready  = has_local and has_remote and imu_ok

        if not nav_ready:
            send_motor_command(0, 0, MODE_STOP)
            with data_lock:
                data["nav_mode"] = MODE_STOP
                data["speed"] = 0; data["left_pwm"] = 0; data["right_pwm"] = 0
                data["turn_zone"] = "WAIT"
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
        # ===== XU LY DIEU KHIEN (AUTO) =====
        # ----- Kiem tra khoang cach: < 5m → tha troi -----
        if distance < COAST_DISTANCE_M:
            left_pwm, right_pwm = 0, 0
            nav_mode = MODE_STOP
            turn_zone = "COAST"
            speed = 0
        else:
            # Giam toc khi gan (< 20m)
            if distance < SLOWDOWN_DISTANCE:
                ratio = (distance - COAST_DISTANCE_M) / (SLOWDOWN_DISTANCE - COAST_DISTANCE_M)
                speed = max(60, int(web_speed * ratio))
            else:
                speed = web_speed

            # Tinh PWM 2 motor dua tren vung goc
            left_pwm, right_pwm = compute_motor_pwm(error, speed)
            nav_mode = MODE_AUTO

            # Xac dinh ten vung de debug
            abs_err = abs(error)
            if abs_err < DEADBAND_DEG:
                turn_zone = "STRAIGHT"
            elif abs_err < MILD_TURN_DEG:
                turn_zone = "MILD_" + ("R" if error > 0 else "L")
            else:
                turn_zone = "HARD_" + ("R" if error > 0 else "L")

        # Gui lenh motor xuong dongco.ino
        send_motor_command(left_pwm, right_pwm, nav_mode)

        # Cap nhat shared data cho phan motor
        with data_lock:
            data["speed"]     = speed
            data["left_pwm"]  = left_pwm
            data["right_pwm"] = right_pwm
            data["nav_mode"]  = nav_mode
            data["turn_zone"] = turn_zone

        # Debug output
        if DEBUG_PRINT and (time.time() - last_debug) >= DEBUG_INTERVAL_S:
            last_debug = time.time()
            print(
                f"🧭 [NAV] {turn_zone:10s} | "
                f"Dist={distance:5.1f}m | "
                f"Bear={bearing:5.1f}° | "
                f"Head={heading:5.1f}° | "
                f"Err={error:+6.1f}° | "
                f"Spd={speed:3d} | "
                f"L={left_pwm:3d} R={right_pwm:3d}"
            )

        elapsed = time.time() - loop_start
        if elapsed < loop_period:
            time.sleep(loop_period - elapsed)


# =============================================================
# MQTT
# =============================================================

def handle_command(topic, payload):
    """Xu ly lenh tu Web."""
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
    print("=" * 50)
    print("  🚤 USV — Simple Zone Navigation (b4c.py)")
    print("=" * 50)

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
                    "turn_zone": d["turn_zone"],
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
                    f"Dist={d.get('distance', 0)}m | "
                    f"L={d['left_pwm']:+d} R={d['right_pwm']:+d} | "
                    f"Zone={d['turn_zone']} | "
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
