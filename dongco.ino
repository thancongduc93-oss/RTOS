#include <ESP32Servo.h>
#include "driver/twai.h"
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <math.h>

// ============================================================
// ESP32 MAIN: MOTOR (MANUAL + AUTO) + BNO055 IMU + CAN BUS
// (Thay the cho CANgui.ino cu, khong con Web Server nua)
// ============================================================

// ---------------- PIN ----------------
constexpr uint8_t THROTTLE_PIN  = 3;
constexpr uint8_t STEERING_PIN  = 4;
constexpr uint8_t ESC_LEFT_PIN  = 6;
constexpr uint8_t ESC_RIGHT_PIN = 7;

#define CAN_TX_PIN  GPIO_NUM_2
#define CAN_RX_PIN  GPIO_NUM_5

// ---------------- CAN IDs ----------------
#define CAN_ID_IMU        0x100
#define CAN_ID_MOTOR_CMD  0x201
#define CAN_ID_MOTOR_FB   0x104  // Gui toc do motor hien tai (L, R) ve Pi

// ---------------- BNO055 ----------------
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29, &Wire);
bool bnoOk = false;
unsigned long lastBnoRetry = 0;
float headingDeg = 0, rollDeg = 0, pitchDeg = 0;
uint8_t sysCal = 0, magCal = 0, gyroCal = 0, accelCal = 0;
unsigned long lastImuSent = 0;

// ---------------- RC ----------------
constexpr int RC_MIN_US = 500;
constexpr int RC_MID_US = 1500;
constexpr int RC_MAX_US = 2000;

// ---------------- ESC ----------------
constexpr int ESC_ARM_US        = 1000;
constexpr int ESC_OUTPUT_MIN_US = 900;
constexpr int ESC_OUTPUT_MAX_US = 2000;
constexpr uint32_t ESC_ARM_DELAY_MS = 5000;
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
constexpr uint32_t PRINT_PERIOD_MS = 100;

constexpr bool AUTO_CALIBRATE_STEERING_CENTER = true;
constexpr uint32_t STEERING_CALIBRATE_MS = 1200;
constexpr int STEERING_TRIM_US = 0;

Servo escLeft;
Servo escRight;

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

// Lệnh từ Raspberry Pi
#define MODE_STOP    0
#define MODE_AUTO    1
#define MODE_MANUAL  2
uint8_t currentMode = MODE_MANUAL;  // Mac dinh MANUAL (RC) cho an toan
int16_t piLeftPwm = 0;
int16_t piRightPwm = 0;
unsigned long lastCANReceived = 0;
constexpr uint32_t CAN_CMD_TIMEOUT_MS = 2000; // Mat CAN 2s khi AUTO → dung motor

// ============================================================
// HELPER FUNCTIONS (TU CANO2)
// ============================================================

int rampToTarget(int cur, int target, int step) {
    if(cur < target) { cur += step; if(cur > target) cur = target; }
    else if(cur > target) { cur -= step; if(cur < target) cur = target; }
    return cur;
}

int applyReverse(int value,bool rev) {
    if(!rev) return value;
    return RC_MIN_US + RC_MAX_US - value;
}

int applyGain(int mix,int gain) {
    mix = (mix * gain) / 100;
    return constrain(mix,0,1000);
}

int mixToEsc(int mix,int startUs) {
    mix = constrain(mix,0,1000);
    if(mix < START_MIX_THRESHOLD) return ESC_ARM_US;
    return map(mix, START_MIX_THRESHOLD, 1000, startUs, ESC_OUTPUT_MAX_US);
}

int applyBoost(int value,int boost) {
    if(value <= ESC_ARM_US) return ESC_ARM_US;
    value += boost;
    return constrain(value, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
}

void IRAM_ATTR onThrottleChange() {
    uint32_t now = micros();
    if(digitalRead(THROTTLE_PIN)) { throttleRiseUs = now; }
    else {
        uint32_t pw = now - throttleRiseUs;
        if(pw > 750 && pw < 2250) { throttlePulseUs = pw; throttleLastUpdateUs = now; }
    }
}

void IRAM_ATTR onSteeringChange() {
    uint32_t now = micros();
    if(digitalRead(STEERING_PIN)) { steeringRiseUs = now; }
    else {
        uint32_t pw = now - steeringRiseUs;
        if(pw > 750 && pw < 2250) { steeringPulseUs = pw; steeringLastUpdateUs = now; }
    }
}

void calibrateSteering() {
    if(!AUTO_CALIBRATE_STEERING_CENTER) { steeringCenterUs = RC_MID_US; return; }
    long sum = 0; int cnt = 0; uint32_t start = millis();
    while(millis() - start < STEERING_CALIBRATE_MS) {
        uint16_t sRaw; uint32_t sTime;
        noInterrupts(); sRaw = steeringPulseUs; sTime = steeringLastUpdateUs; interrupts();
        if(micros() - sTime < SIGNAL_TIMEOUT_US && sRaw >= RC_MIN_US && sRaw <= RC_MAX_US) {
            sum += sRaw; cnt++;
        }
        delay(2);
    }
    steeringCenterUs = (cnt > 0) ? (sum / cnt) + STEERING_TRIM_US : RC_MID_US;
}

// ============================================================
// CAN BUS & IMU
// ============================================================

bool canOk = false;
unsigned long lastCanCheck = 0;
uint32_t canTxFail = 0;
uint32_t canRxCount = 0;

void setupCAN() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 5;       // Giam TX queue de khong bi nghen
    g_config.rx_queue_len = 10;      // Tang RX queue de khong mat frame
    twai_timing_config_t  t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    esp_err_t install = twai_driver_install(&g_config, &t_config, &f_config);
    if (install != ESP_OK) {
        Serial.printf("CAN: Install FAIL (err=%d)\n", install);
        canOk = false;
        return;
    }
    esp_err_t start = twai_start();
    if (start != ESP_OK) {
        Serial.printf("CAN: Start FAIL (err=%d)\n", start);
        canOk = false;
        return;
    }
    canOk = true;
    Serial.println("CAN: OK (500kbps) TX=GPIO2 RX=GPIO5");
}

// Kiem tra va phuc hoi CAN bus khi bi bus-off
void checkCANHealth() {
    if (!canOk) return;
    if (millis() - lastCanCheck < 1000) return; // Kiem tra moi 1 giay
    lastCanCheck = millis();

    twai_status_info_t status;
    if (twai_get_status_info(&status) == ESP_OK) {
        // In trang thai CAN de debug
        static uint32_t lastStatusPrint = 0;
        if (millis() - lastStatusPrint > 5000) {
            lastStatusPrint = millis();
            Serial.printf("[CAN] state=%d tx_err=%lu rx_err=%lu tx_fail=%lu rx_miss=%lu tx_q=%lu rx_q=%lu rxCnt=%lu\n",
                status.state, status.tx_error_counter, status.rx_error_counter,
                status.tx_failed_count, status.rx_missed_count,
                status.msgs_to_tx, status.msgs_to_rx, canRxCount);
        }

        // Neu CAN bi bus-off hoac recovery → restart
        if (status.state == TWAI_STATE_BUS_OFF) {
            Serial.println("[CAN] BUS-OFF! Dang khoi dong lai...");
            twai_stop();
            twai_driver_uninstall();
            delay(100);
            setupCAN();
            canTxFail = 0;
        }
        else if (status.state == TWAI_STATE_RECOVERING) {
            Serial.println("[CAN] RECOVERING...");
            // Doi CAN tu phuc hoi, khong gui gi
        }
    }
}

void receiveCAN() {
    if (!canOk) return;
    twai_message_t msg;
    while (twai_receive(&msg, 0) == ESP_OK) {
        canRxCount++;
        if (msg.identifier == CAN_ID_MOTOR_CMD && msg.data_length_code >= 5) {
            memcpy(&piLeftPwm, &msg.data[0], 2);
            memcpy(&piRightPwm, &msg.data[2], 2);
            uint8_t mode = msg.data[4];
            currentMode = mode;  // 0=STOP, 1=AUTO, 2=MANUAL
            lastCANReceived = millis();
            Serial.printf("[CAN-RX] Motor CMD: L=%d R=%d Mode=%d\n", piLeftPwm, piRightPwm, mode);
        }
    }
}

bool canTransmit(twai_message_t* msg) {
    if (!canOk) return false;
    esp_err_t ret = twai_transmit(msg, pdMS_TO_TICKS(5)); // Timeout 5ms thay vi 0
    if (ret != ESP_OK) {
        canTxFail++;
        if (canTxFail % 100 == 1) { // Khong spam serial
            Serial.printf("[CAN-TX] FAIL id=0x%03X err=%d (total=%lu)\n", 
                msg->identifier, ret, canTxFail);
        }
        return false;
    }
    return true;
}

void sendIMU() {
    if (!bnoOk) return; // KHONG gui khi BNO055 chua san sang!
    if (millis() - lastImuSent < 50) return; // 20Hz
    lastImuSent = millis();
    int16_t hdg = (int16_t)lroundf(headingDeg * 100.0f);
    int16_t rol = (int16_t)lroundf(rollDeg    * 100.0f);
    int16_t pit = (int16_t)lroundf(pitchDeg   * 100.0f);
    twai_message_t msg;
    msg.identifier = CAN_ID_IMU; msg.extd = 0; msg.rtr = 0; msg.data_length_code = 8;
    msg.data[0] = hdg & 0xFF; msg.data[1] = (hdg >> 8) & 0xFF;
    msg.data[2] = rol & 0xFF; msg.data[3] = (rol >> 8) & 0xFF;
    msg.data[4] = pit & 0xFF; msg.data[5] = (pit >> 8) & 0xFF;
    msg.data[6] = sysCal; msg.data[7] = magCal;
    canTransmit(&msg);
}

void sendMotorFB() {
    static uint32_t lastFb = 0;
    if (millis() - lastFb < 100) return; // 10Hz
    lastFb = millis();
    twai_message_t msg;
    msg.identifier = CAN_ID_MOTOR_FB; msg.extd = 0; msg.rtr = 0; msg.data_length_code = 4;
    // Doi L/R cho khop voi hien thi tren Web
    int16_t l = rightUs; int16_t r = leftUs;
    memcpy(&msg.data[0], &l, 2);
    memcpy(&msg.data[2], &r, 2);
    canTransmit(&msg);
}

// ============================================================
// SETUP & LOOP
// ============================================================

void setup() {
    Serial.begin(115200);
    delay(500);
    Serial.println("\n========================================");
    Serial.println("  ESP32-C3 Motor Controller + BNO055 + CAN");
    Serial.println("========================================");

    // 1. RC Input
    pinMode(THROTTLE_PIN,INPUT);
    pinMode(STEERING_PIN,INPUT);
    attachInterrupt(digitalPinToInterrupt(THROTTLE_PIN), onThrottleChange, CHANGE);
    attachInterrupt(digitalPinToInterrupt(STEERING_PIN), onSteeringChange, CHANGE);
    Serial.println("[RC] Interrupt attached: Throttle=GPIO3, Steering=GPIO4");

    // 2. ESC Arm
    escLeft.setPeriodHertz(50);
    escRight.setPeriodHertz(50);
    escLeft.attach(ESC_LEFT_PIN, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
    escRight.attach(ESC_RIGHT_PIN, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
    escLeft.writeMicroseconds(ESC_ARM_US);
    escRight.writeMicroseconds(ESC_ARM_US);
    Serial.println("[ESC] Arming...");
    delay(ESC_ARM_DELAY_MS);
    Serial.println("[ESC] Armed OK");

    // 3. CAN Bus — PHAI KHOI TAO TRUOC I2C de tranh xung dot GPIO matrix
    setupCAN();

    // 4. I2C BNO055 — KHOI TAO SAU CAN
    Wire.begin(18, 19);
    delay(100);
    int bnoRetry = 0;
    while (!bno.begin()) {
        Serial.printf("[IMU] BNO055 FAIL (retry %d/5)\n", bnoRetry + 1);
        delay(500);
        if (++bnoRetry >= 5) { 
            Serial.println("[IMU] BNO055: KHONG TIM THAY — se thu lai trong loop"); 
            break; 
        }
    }
    if(bnoRetry < 5) {
        bno.setMode(OPERATION_MODE_NDOF);
        bnoOk = true;
        Serial.println("[IMU] BNO055 OK (addr=0x29, SDA=18, SCL=19)");
    }

    // 5. Steering calibration
    calibrateSteering();
    steeringFilteredUs = steeringCenterUs;

    Serial.println("========================================");
    Serial.printf("CAN: %s | BNO055: %s | Mode: MANUAL\n", 
        canOk ? "OK" : "FAIL", bnoOk ? "OK" : "FAIL");
    Serial.println("READY - Cho lenh tu Pi qua CAN 0x201");
    Serial.println("========================================");
}

void loop() {
    // 0. Kiem tra suc khoe CAN bus (phuc hoi bus-off)
    checkCANHealth();

    // 1. Doc BNO055 va gui len Pi
    if (bnoOk) {
        imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
        headingDeg = euler.x(); rollDeg = euler.y(); pitchDeg = euler.z();
        bno.getCalibration(&sysCal, &gyroCal, &accelCal, &magCal);
    } else {
        // Thu ket noi lai BNO055 moi 5 giay
        if (millis() - lastBnoRetry > 5000) {
            lastBnoRetry = millis();
            Wire.begin(18, 19);
            if (bno.begin()) {
                bno.setMode(OPERATION_MODE_NDOF);
                bnoOk = true;
                Serial.println("[IMU] BNO055 da ket noi lai OK!");
            }
        }
    }
    sendIMU(); // Chi gui khi bnoOk = true (da check trong ham)

    // 2. Nhận lenh dieu khien (AUTO) tu Pi
    receiveCAN();

    // 3. Chu ky dieu khien motor (50Hz = 20ms)
    static uint32_t lastControl = 0;
    static uint32_t lastPrint = 0;
    if(millis() - lastControl < CONTROL_PERIOD_MS) return;
    lastControl = millis();

    uint16_t throttleRaw; uint16_t steeringRaw;
    uint32_t tTime; uint32_t sTime;
    noInterrupts();
    throttleRaw = throttlePulseUs; steeringRaw = steeringPulseUs;
    tTime = throttleLastUpdateUs; sTime = steeringLastUpdateUs;
    interrupts();

    bool signalOk = (micros()-tTime < SIGNAL_TIMEOUT_US) && (micros()-sTime < SIGNAL_TIMEOUT_US);

    // Safety: Mat CAN > 2s khi dang AUTO → tu dong dung motor
    if (currentMode == MODE_AUTO && (millis() - lastCANReceived > CAN_CMD_TIMEOUT_MS)) {
        currentMode = MODE_STOP;
        Serial.println("[SAFETY] CAN Timeout → STOP!");
    }

    int leftTarget = ESC_ARM_US;
    int rightTarget = ESC_ARM_US;

    if (currentMode == MODE_STOP) {
        // ================= STOP: MOTOR DUNG, TAY CAM VO HIEU HOA =================
        leftTarget = ESC_ARM_US;
        rightTarget = ESC_ARM_US;

    } else if (currentMode == MODE_AUTO) {
        // ================= AUTO: PI DIEU KHIEN, TAY CAM VO HIEU HOA =================
        // Pi gui PWM tu 0 den 255. Ta map len 1080 -> 2000
        if (piLeftPwm > 0) {
            leftTarget = map(constrain(piLeftPwm, 0, 255), 0, 255, LEFT_START_US, 2000);
        } else {
            leftTarget = ESC_ARM_US;
        }
        if (piRightPwm > 0) {
            rightTarget = map(constrain(piRightPwm, 0, 255), 0, 255, RIGHT_START_US, 2000);
        } else {
            rightTarget = ESC_ARM_US;
        }

    } else if (currentMode == MODE_MANUAL && signalOk) {
        // ================= MANUAL: TAY CAM RC DIEU KHIEN =================
        throttleRaw = applyReverse(constrain(throttleRaw,RC_MIN_US,RC_MAX_US), REVERSE_THROTTLE);
        steeringRaw = applyReverse(constrain(steeringRaw,RC_MIN_US,RC_MAX_US), REVERSE_STEERING);
        steeringFilteredUs = ((steeringFilteredUs*STEERING_FILTER_NUM)+ steeringRaw*(STEERING_FILTER_DEN-STEERING_FILTER_NUM)) / STEERING_FILTER_DEN;
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

        int leftMix = constrain(throttleMix + steeringCmd, 0, 1000);
        int rightMix = constrain(throttleMix - steeringCmd, 0, 1000);

        leftMix = applyGain(leftMix, LEFT_GAIN_PERCENT);
        rightMix = applyGain(rightMix, RIGHT_GAIN_PERCENT);

        leftTarget = applyBoost(mixToEsc(leftMix,LEFT_START_US), LEFT_BOOST_US);
        rightTarget = applyBoost(mixToEsc(rightMix,RIGHT_START_US), RIGHT_BOOST_US);

        if(steeringCmd == 0 && throttleMix > START_MIX_THRESHOLD) {
            leftTarget += STRAIGHT_TRIM_US;
            rightTarget -= STRAIGHT_TRIM_US;
            leftTarget = constrain(leftTarget, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
            rightTarget = constrain(rightTarget, ESC_OUTPUT_MIN_US, ESC_OUTPUT_MAX_US);
        }
    }
    // else: MODE_MANUAL nhung mat tin hieu RC → leftTarget/rightTarget = ESC_ARM_US (da set o tren)

    // Lam muot toc do motor
    leftUs = rampToTarget(leftUs, leftTarget, RAMP_STEP_US);
    rightUs = rampToTarget(rightUs, rightTarget, RAMP_STEP_US);

    escLeft.writeMicroseconds(leftUs);
    escRight.writeMicroseconds(rightUs);

    // 4. Gui Feedback toc do thuc te len Pi
    sendMotorFB();

    if(millis()-lastPrint > PRINT_PERIOD_MS) {
        lastPrint = millis();
        const char* modeStr = (currentMode == MODE_AUTO) ? "AUTO" : (currentMode == MODE_STOP ? "STOP" : "MANUAL");
        Serial.printf("[%s] L=%d R=%d Hdg=%.1f\n", modeStr, leftUs, rightUs, headingDeg);
    }
}
