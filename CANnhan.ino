#include <ESP32Servo.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include "driver/twai.h"

// ============================================================
// ESP32-C3 — Motor Controller + BNO055 + CAN Bus
// Chế độ: MANUAL (RC) + AUTO (nhận lệnh từ Pi qua CAN)
// ============================================================
//
// CAN Protocol (đồng bộ với b4.py):
//   SEND:
//     0x100 — IMU: heading(2B) + roll(2B) + pitch(2B) + sys_cal(1B) + mag_cal(1B)
//             int16 × 100, little-endian
//   RECEIVE:
//     0x201 — Motor: left_pwm(2B) + right_pwm(2B) + mode(1B)
//             int16 little-endian, mode: 0=STOP 1=AUTO 2=MANUAL
//
// Phần cứng:
//   CAN:    TX=GPIO2, RX=GPIO5 (qua MCP2551/TJA1050)
//   ESC:    Left=GPIO6, Right=GPIO7
//   RC:     Throttle=GPIO3, Steering=GPIO4
//   I2C:    SDA=GPIO10, SCL=GPIO20 (BNO055 addr 0x29)
// ============================================================

// ---------------- PIN CONFIG ----------------
constexpr uint8_t THROTTLE_PIN  = 3;
constexpr uint8_t STEERING_PIN  = 4;

constexpr uint8_t ESC_LEFT_PIN  = 6;
constexpr uint8_t ESC_RIGHT_PIN = 7;

// CAN Bus pins
#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_5

// I2C pins for BNO055
#define I2C_SDA_PIN 10
#define I2C_SCL_PIN 20

// ---------------- RC ----------------
constexpr int RC_MIN_US = 500;
constexpr int RC_MID_US = 1500;
constexpr int RC_MAX_US = 2000;

// ---------------- ESC ----------------
constexpr int ESC_ARM_US        = 1000;
constexpr int ESC_OUTPUT_MIN_US = 900;
constexpr int ESC_OUTPUT_MAX_US = 2000;

constexpr uint32_t ESC_ARM_DELAY_MS = 5000;

// ---------------- MOTOR TUNE ----------------
constexpr int LEFT_BOOST_US  = 0;
constexpr int RIGHT_BOOST_US = 30;

constexpr int LEFT_START_US  = 1080;
constexpr int RIGHT_START_US = 1080;

constexpr int LEFT_GAIN_PERCENT  = 100;
constexpr int RIGHT_GAIN_PERCENT = 100;

constexpr int STRAIGHT_TRIM_US = 0;

// ---------------- CONTROL ----------------
constexpr bool REVERSE_THROTTLE = false;
constexpr bool REVERSE_STEERING = false;

constexpr int THROTTLE_DEADBAND_US = 20;
constexpr int STEERING_DEADBAND_US = 20;

constexpr int STEERING_MIX_LIMIT = 700;

constexpr int STEERING_FILTER_NUM = 5;
constexpr int STEERING_FILTER_DEN = 8;

constexpr int START_MIX_THRESHOLD = 40;

constexpr int RAMP_STEP_US = 40;

constexpr uint32_t CONTROL_PERIOD_MS = 20;
constexpr uint32_t SIGNAL_TIMEOUT_US = 100000;
constexpr uint32_t PRINT_PERIOD_MS = 200;

// ---------------- AUTO CAL ----------------
constexpr bool AUTO_CALIBRATE_STEERING_CENTER = true;
constexpr uint32_t STEERING_CALIBRATE_MS = 1200;
constexpr int STEERING_TRIM_US = 0;

// ================== CAN BUS PROTOCOL ==================
// *** ĐỒNG BỘ VỚI b4.py ***
#define CAN_ID_IMU        0x100  // Gửi → Pi: Heading + Roll + Pitch + Cal
#define CAN_ID_MOTOR_CMD  0x201  // Nhận ← Pi: L_pwm + R_pwm + Mode

#define MODE_STOP    0
#define MODE_AUTO    1
#define MODE_MANUAL  2

constexpr uint32_t CAN_TIMEOUT_MS = 1000;
constexpr uint32_t IMU_SEND_PERIOD_MS = 50; // 20Hz

// ============================================================

Servo escLeft;
Servo escRight;

// BNO055
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
bool bnoOK = false;

volatile uint32_t throttleRiseUs = 0;
volatile uint32_t steeringRiseUs = 0;

volatile uint16_t throttlePulseUs = RC_MID_US;
volatile uint16_t steeringPulseUs = RC_MID_US;

volatile uint32_t throttleLastUpdateUs = 0;
volatile uint32_t steeringLastUpdateUs = 0;

int leftUs = ESC_ARM_US;
int rightUs = ESC_ARM_US;

int steeringCenterUs = RC_MID_US;
int steeringFilteredUs = RC_MID_US;

int throttleFiltered = RC_MID_US;

// Auto Mode State Variables
int16_t autoLeftPWM = 0;
int16_t autoRightPWM = 0;
uint8_t currentMode = MODE_MANUAL;
unsigned long lastCANReceived = 0;
unsigned long lastIMUSent = 0;

// ============================================================
// HELPER FUNCTIONS
// ============================================================

int rampToTarget(int cur, int target, int step)
{
    if(cur < target){
        cur += step;
        if(cur > target) cur = target;
    }
    else if(cur > target){
        cur -= step;
        if(cur < target) cur = target;
    }
    return cur;
}

int applyReverse(int value, bool rev)
{
    if(!rev) return value;
    return RC_MIN_US + RC_MAX_US - value;
}

int applyGain(int mix, int gain)
{
    mix = (mix * gain) / 100;
    return constrain(mix, 0, 1000);
}

int mixToEsc(int mix, int startUs)
{
    mix = constrain(mix, 0, 1000);
    if(mix < START_MIX_THRESHOLD)
        return ESC_ARM_US;
    return map(mix, START_MIX_THRESHOLD, 1000, startUs, ESC_OUTPUT_MAX_US);
}

int applyBoost(int value, int boost)
{
    if(value <= ESC_ARM_US)
        return ESC_ARM_US;
    value += boost;
    return constrain(value, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
}

// Convert Pi PWM (-255~255) → ESC microseconds (1000~2000)
int autoPwmToMicroseconds(int16_t pwmValue) {
    if (pwmValue == 0) return ESC_ARM_US;
    int us = 1500 + ((float)pwmValue / 255.0) * 500.0;
    return constrain(us, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
}

// ============================================================
// INTERRUPTS (RC Receiver)
// ============================================================

void IRAM_ATTR onThrottleChange()
{
    uint32_t now = micros();
    if(digitalRead(THROTTLE_PIN)) {
        throttleRiseUs = now;
    } else {
        uint32_t pw = now - throttleRiseUs;
        if(pw > 750 && pw < 2250) {
            throttlePulseUs = pw;
            throttleLastUpdateUs = now;
        }
    }
}

void IRAM_ATTR onSteeringChange()
{
    uint32_t now = micros();
    if(digitalRead(STEERING_PIN)) {
        steeringRiseUs = now;
    } else {
        uint32_t pw = now - steeringRiseUs;
        if(pw > 750 && pw < 2250) {
            steeringPulseUs = pw;
            steeringLastUpdateUs = now;
        }
    }
}

// ============================================================
// STEERING CALIBRATION
// ============================================================

void calibrateSteering()
{
    if(!AUTO_CALIBRATE_STEERING_CENTER) {
        steeringCenterUs = RC_MID_US;
        return;
    }

    long sum = 0;
    int cnt = 0;
    uint32_t start = millis();

    while(millis() - start < STEERING_CALIBRATE_MS) {
        uint16_t sRaw;
        uint32_t sTime;
        noInterrupts();
        sRaw = steeringPulseUs;
        sTime = steeringLastUpdateUs;
        interrupts();

        if(micros() - sTime < SIGNAL_TIMEOUT_US && sRaw >= RC_MIN_US && sRaw <= RC_MAX_US) {
            sum += sRaw;
            cnt++;
        }
        delay(2);
    }

    steeringCenterUs = (cnt > 0) ? (sum / cnt) + STEERING_TRIM_US : RC_MID_US;
    Serial.print("Center=");
    Serial.println(steeringCenterUs);
}

// ============================================================
// CAN BUS FUNCTIONS
// ============================================================

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
        CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL
    );
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        if (twai_start() == ESP_OK) {
            Serial.println("[CAN] OK — TX:0x100 IMU | RX:0x201 Motor");
        } else {
            Serial.println("[CAN] Start FAILED");
        }
    } else {
        Serial.println("[CAN] Install FAILED");
    }
}

// Nhận lệnh motor từ Pi (CAN 0x201)
void receiveCANCommand() {
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        if (msg.identifier == CAN_ID_MOTOR_CMD && msg.data_length_code >= 5) {
            memcpy(&autoLeftPWM,  &msg.data[0], 2);
            memcpy(&autoRightPWM, &msg.data[2], 2);
            currentMode = msg.data[4];
            lastCANReceived = millis();
        }
    }
}

// Gửi BNO055 heading lên Pi (CAN 0x100)
void sendIMUOverCAN() {
    if (!bnoOK) return;

    sensors_event_t event;
    bno.getEvent(&event);

    uint8_t sys, gyro, accel, mag;
    bno.getCalibration(&sys, &gyro, &accel, &mag);

    // Nhân ×100 → int16 little-endian (khớp với b4.py parse '<h' / 100.0)
    int16_t heading_scaled = (int16_t)(event.orientation.x * 100.0);
    int16_t roll_scaled    = (int16_t)(event.orientation.y * 100.0);
    int16_t pitch_scaled   = (int16_t)(event.orientation.z * 100.0);

    twai_message_t msg;
    msg.identifier = CAN_ID_IMU;   // 0x100
    msg.extd = 0;
    msg.rtr = 0;
    msg.data_length_code = 8;

    memcpy(&msg.data[0], &heading_scaled, 2);  // byte 0-1: heading (LE)
    memcpy(&msg.data[2], &roll_scaled, 2);     // byte 2-3: roll (LE)
    memcpy(&msg.data[4], &pitch_scaled, 2);    // byte 4-5: pitch (LE)
    msg.data[6] = sys;                         // byte 6: sys_cal
    msg.data[7] = mag;                         // byte 7: mag_cal

    twai_transmit(&msg, pdMS_TO_TICKS(10));
}

// ============================================================
// SETUP
// ============================================================

void setup()
{
    Serial.begin(115200);

    pinMode(THROTTLE_PIN, INPUT);
    pinMode(STEERING_PIN, INPUT);

    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), onThrottleChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_PIN), onSteeringChange, CHANGE);

    escLeft.setPeriodHertz(50);
    escRight.setPeriodHertz(50);

    escLeft.attach(ESC_LEFT_PIN, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
    escRight.attach(ESC_RIGHT_PIN, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);

    escLeft.writeMicroseconds(ESC_ARM_US);
    escRight.writeMicroseconds(ESC_ARM_US);

    delay(ESC_ARM_DELAY_MS);

    calibrateSteering();
    steeringFilteredUs = steeringCenterUs;

    // CAN Bus
    setupCAN();

    // BNO055 I2C
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    delay(100);
    if (bno.begin()) {
        bno.setMode(OPERATION_MODE_NDOF);
        bnoOK = true;
        Serial.println("[IMU] BNO055 OK (0x29)");
    } else {
        Serial.println("[IMU] BNO055 NOT DETECTED!");
    }

    lastCANReceived = millis();
    Serial.println("=== SYSTEM READY ===");
}

// ============================================================
// LOOP
// ============================================================

void loop()
{
    static uint32_t lastControl = 0;
    static uint32_t lastPrint = 0;

    // 1. Nhận lệnh motor từ Pi qua CAN (0x201)
    receiveCANCommand();

    // 2. Gửi BNO055 heading lên Pi qua CAN (0x100) — 20Hz
    if (bnoOK && (millis() - lastIMUSent >= IMU_SEND_PERIOD_MS)) {
        lastIMUSent = millis();
        sendIMUOverCAN();
    }

    // 3. Vòng lặp điều khiển ESC (20ms)
    if(millis() - lastControl < CONTROL_PERIOD_MS)
        return;
    lastControl = millis();

    // Safety: Mất CAN > 1s khi AUTO → về MANUAL (vẫn lái tay được)
    if (currentMode == MODE_AUTO && (millis() - lastCANReceived > CAN_TIMEOUT_MS)) {
        Serial.println("[SAFETY] CAN Timeout → MANUAL (RC)!");
        currentMode = MODE_MANUAL;
    }

    int leftTarget = ESC_ARM_US;
    int rightTarget = ESC_ARM_US;

    if (currentMode == MODE_AUTO) {
        // --- CHẾ ĐỘ TỰ ĐỘNG (AUTO) — Lệnh từ Pi ---
        leftTarget  = autoPwmToMicroseconds(autoLeftPWM);
        rightTarget = autoPwmToMicroseconds(autoRightPWM);
    }
    else if (currentMode == MODE_STOP) {
        // --- CHẾ ĐỘ DỪNG (STOP) ---
        leftTarget  = ESC_ARM_US;
        rightTarget = ESC_ARM_US;
    }
    else {
        // --- CHẾ ĐỘ THỦ CÔNG (MANUAL) — RC Receiver ---
        uint16_t throttleRaw;
        uint16_t steeringRaw;
        uint32_t tTime;
        uint32_t sTime;

        noInterrupts();
        throttleRaw = throttlePulseUs;
        steeringRaw = steeringPulseUs;
        tTime = throttleLastUpdateUs;
        sTime = steeringLastUpdateUs;
        interrupts();

        bool signalOk = (micros()-tTime < SIGNAL_TIMEOUT_US) && (micros()-sTime < SIGNAL_TIMEOUT_US);

        if(signalOk) {
            throttleRaw = applyReverse(constrain(throttleRaw, RC_MIN_US, RC_MAX_US), REVERSE_THROTTLE);
            steeringRaw = applyReverse(constrain(steeringRaw, RC_MIN_US, RC_MAX_US), REVERSE_STEERING);

            steeringFilteredUs = ((steeringFilteredUs*STEERING_FILTER_NUM) + steeringRaw*(STEERING_FILTER_DEN-STEERING_FILTER_NUM)) / STEERING_FILTER_DEN;
            throttleFiltered = (throttleFiltered*3 + throttleRaw) / 4;

            int throttleMix = 0;
            if(throttleFiltered > RC_MID_US + THROTTLE_DEADBAND_US) {
                throttleMix = map(throttleFiltered, RC_MID_US + THROTTLE_DEADBAND_US, RC_MAX_US, 0, 1000);
            }

            int steering = steeringFilteredUs - steeringCenterUs;
            steering = constrain(steering, -500, 500);

            float s = (float)steering / 500.0;
            s = s * abs(s);

            int steeringCmd = s * STEERING_MIX_LIMIT;
            if(abs(steeringCmd) < 20) steeringCmd = 0;

            int leftMix  = constrain(throttleMix + steeringCmd, 0, 1000);
            int rightMix = constrain(throttleMix - steeringCmd, 0, 1000);

            leftMix  = applyGain(leftMix, LEFT_GAIN_PERCENT);
            rightMix = applyGain(rightMix, RIGHT_GAIN_PERCENT);

            leftTarget  = applyBoost(mixToEsc(leftMix, LEFT_START_US), LEFT_BOOST_US);
            rightTarget = applyBoost(mixToEsc(rightMix, RIGHT_START_US), RIGHT_BOOST_US);

            if(steeringCmd == 0 && throttleMix > START_MIX_THRESHOLD) {
                leftTarget  += STRAIGHT_TRIM_US;
                rightTarget -= STRAIGHT_TRIM_US;
                leftTarget  = constrain(leftTarget,  ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
                rightTarget = constrain(rightTarget, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
            }
        }
    }

    // Tăng tốc mịn (Ramping)
    leftUs  = rampToTarget(leftUs,  leftTarget,  RAMP_STEP_US);
    rightUs = rampToTarget(rightUs, rightTarget, RAMP_STEP_US);

    escLeft.writeMicroseconds(leftUs);
    escRight.writeMicroseconds(rightUs);

    // Debug output
    if(millis() - lastPrint > PRINT_PERIOD_MS) {
        lastPrint = millis();
        Serial.print("Mode=");
        Serial.print(currentMode == MODE_AUTO ? "AUTO" : (currentMode == MODE_STOP ? "STOP" : "MANUAL"));
        Serial.print(" | L=");
        Serial.print(leftUs);
        Serial.print(" R=");
        Serial.print(rightUs);

        if (bnoOK) {
            sensors_event_t event;
            bno.getEvent(&event);
            Serial.print(" | Yaw=");
            Serial.print(event.orientation.x, 1);
        }
        Serial.println();
    }
}