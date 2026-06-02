#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <math.h>
#include "driver/twai.h"

// =====================================================
// GPS
// =====================================================
HardwareSerial GPS(1);
TinyGPSPlus gps;

// =====================================================
// LORA
// =====================================================
HardwareSerial LoRa(0);

// =====================================================
// LCD
// =====================================================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// =====================================================
// REMOTE GPS
// =====================================================
bool remoteValid = false;
unsigned long lastLoRaReceived = 0;
const unsigned long LORA_TIMEOUT = 5000; // mất 5s không nhận → invalid

// LoRa receive buffer (non-blocking)
String loraBuffer = "";
const int LORA_BUF_MAX = 128; // giới hạn buffer tránh tràn RAM

// =====================================================
// LCD SCREEN
// =====================================================
int lcdScreen = 0;
unsigned long lastLCDChange = 0;
const unsigned long LCD_INTERVAL = 3000; // giảm từ 5000 → 3000ms

// =====================================================
// EMA FILTER + STATIC AVERAGING (thay thế Median filter)
// =====================================================
#define EMA_ALPHA 0.5       // 0.0 = rất smooth, 1.0 = không filter
#define DEADZONE_METERS 3.0 // Ngưỡng chuyển từ static → moving mode

double localLatFiltered = 0.0;
double localLonFiltered = 0.0;
double remoteLatFiltered = 0.0;
double remoteLonFiltered = 0.0;

bool localFilterInit = false;
bool remoteFilterInit = false;
double cachedDistance = 0.0;

// Static averaging: khi đứng yên, trung bình nhiều mẫu để giảm sai số GPS
// Sai số giảm theo công thức: error/√N (ví dụ: 50 mẫu → sai số giảm 7 lần)
double localLatSum = 0.0, localLonSum = 0.0;
unsigned long localAvgCount = 0;

double remoteLatSum = 0.0, remoteLonSum = 0.0;
unsigned long remoteAvgCount = 0;

// Bộ đếm kiểm chứng di chuyển chống nhiễu nhảy vọt (xe máy đi qua)
unsigned int localMoveSamples = 0;
unsigned int remoteMoveSamples = 0;

unsigned long lastCANSend = 0;
const unsigned long CAN_SEND_INTERVAL = 200; // Gửi liên tục 5Hz (200ms)

// =====================================================
// CAN BUS (TWAI) — GPIO 2 & 3
// =====================================================
#define CAN_TX_PIN GPIO_NUM_2
#define CAN_RX_PIN GPIO_NUM_3

void sendGPSOverCAN() {
  twai_message_t msg;
  msg.extd = 0; // Standard 11-bit ID
  msg.rtr = 0;

  // 1. Gửi dữ liệu LOCAL GPS (ID: 0x101) - Gửi 0 nếu chưa có GPS
  int32_t localLatScaled = 0;
  int32_t localLonScaled = 0;
  if (localFilterInit) {
    localLatScaled = localLatFiltered * 1000000.0;
    localLonScaled = localLonFiltered * 1000000.0;
  }
  
  msg.identifier = 0x101;
  msg.data_length_code = 8;
  memcpy(&msg.data[0], &localLatScaled, 4);
  memcpy(&msg.data[4], &localLonScaled, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(10));

  // 2. Gửi dữ liệu REMOTE GPS (ID: 0x102) - Gửi 0 nếu chưa có LoRa
  int32_t remoteLatScaled = 0;
  int32_t remoteLonScaled = 0;
  if (remoteValid) {
    remoteLatScaled = remoteLatFiltered * 1000000.0;
    remoteLonScaled = remoteLonFiltered * 1000000.0;
  }
  
  msg.identifier = 0x102;
  msg.data_length_code = 8;
  memcpy(&msg.data[0], &remoteLatScaled, 4);
  memcpy(&msg.data[4], &remoteLonScaled, 4);
  twai_transmit(&msg, pdMS_TO_TICKS(10));

  // 3. Gửi dữ liệu khoảng cách và trạng thái valid (ID: 0x103)
  int32_t distScaled = 0;
  if (localFilterInit && remoteValid) {
    distScaled = cachedDistance * 100.0; // đổi sang cm để giữ 2 số thập phân
  }
  
  msg.identifier = 0x103;
  msg.data_length_code = 6;
  memcpy(&msg.data[0], &distScaled, 4);
  msg.data[4] = localFilterInit ? 1 : 0;
  msg.data[5] = remoteValid ? 1 : 0;
  twai_transmit(&msg, pdMS_TO_TICKS(10));
}

// =====================================================
// GPS 5Hz rate command (không reset để giữ Hot/Warm Start)
// =====================================================
const uint8_t setRate5Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
                              0xC8, 0x00, // 200ms
                              0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};


// =====================================================
// HAVERSINE DISTANCE
// =====================================================
double calculateDistance(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double phi1 = lat1 * M_PI / 180.0;
  double phi2 = lat2 * M_PI / 180.0;
  double dPhi = (lat2 - lat1) * M_PI / 180.0;
  double dLambda = (lon2 - lon1) * M_PI / 180.0;

  double a = sin(dPhi / 2.0) * sin(dPhi / 2.0) +
             cos(phi1) * cos(phi2) * sin(dLambda / 2.0) * sin(dLambda / 2.0);

  double c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));
  return R * c;
}

// =====================================================
// UPDATE LOCAL GPS FILTER (Static Averaging + EMA)
// =====================================================
void updateLocalGPSFilter() {
  if (!gps.location.isValid())
    return;
  if (!gps.location.isUpdated())
    return;

  double newLat = gps.location.lat();
  double newLon = gps.location.lng();

  if (!localFilterInit) {
    // Lần đầu: gán trực tiếp + khởi tạo averaging
    localLatFiltered = newLat;
    localLonFiltered = newLon;
    localLatSum = newLat;
    localLonSum = newLon;
    localAvgCount = 1;
    localFilterInit = true;
  } else {
    double moved =
        calculateDistance(localLatFiltered, localLonFiltered, newLat, newLon);

    if (moved < DEADZONE_METERS) {
      // Đứng yên: tích lũy và trung bình để giảm sai số GPS
      // Quy luật: sai số giảm theo 1/√N
      localLatSum += newLat;
      localLonSum += newLon;
      localAvgCount++;
      localLatFiltered = localLatSum / localAvgCount;
      localLonFiltered = localLonSum / localAvgCount;
      localMoveSamples = 0; // reset bộ đếm kiểm chứng
    } else {
      localMoveSamples++;
      if (localMoveSamples >= 4) {
        // Xác nhận di chuyển thực tế (lệch liên tiếp 4 mẫu ~0.8 giây)
        localLatFiltered =
            EMA_ALPHA * newLat + (1.0 - EMA_ALPHA) * localLatFiltered;
        localLonFiltered =
            EMA_ALPHA * newLon + (1.0 - EMA_ALPHA) * localLonFiltered;
        localLatSum = localLatFiltered;
        localLonSum = localLonFiltered;
        localAvgCount = 1;
      } else {
        // Nghi ngờ nhiễu nhảy vọt (xe máy qua): Tiếp tục giữ lọc tĩnh ổn định
        localLatSum += localLatFiltered;
        localLonSum += localLonFiltered;
        localAvgCount++;
      }
    }
  }

  // Tính lại distance ngay khi local GPS update
  if (remoteFilterInit) {
    cachedDistance = calculateDistance(localLatFiltered, localLonFiltered,
                                       remoteLatFiltered, remoteLonFiltered);
  }
}

// =====================================================
// RECEIVE LORA (Non-blocking + EMA + Dead-zone)
// =====================================================
void updateLoRa() {
  // Timeout: nếu lâu không nhận được → đánh dấu invalid
  if (remoteValid && millis() - lastLoRaReceived > LORA_TIMEOUT) {
    remoteValid = false;
  }

  // Đọc từng byte, KHÔNG blocking — tránh chặn loop()
  while (LoRa.available()) {
    char c = LoRa.read();

    if (c == '\n') {
      // Đã nhận đủ 1 dòng → xử lý
      loraBuffer.trim();

      if (loraBuffer.startsWith("GPS:")) {
        String data = loraBuffer.substring(4);
        int comma = data.indexOf(',');

        if (comma > 0) {
          double rawLat = data.substring(0, comma).toDouble();
          double rawLon = data.substring(comma + 1).toDouble();

          // Validate tọa độ Việt Nam
          if (rawLat >= 8.0 && rawLat <= 24.0 && rawLon >= 102.0 &&
              rawLon <= 110.0) {

            if (!remoteFilterInit) {
              remoteLatFiltered = rawLat;
              remoteLonFiltered = rawLon;
              remoteLatSum = rawLat;
              remoteLonSum = rawLon;
              remoteAvgCount = 1;
              remoteFilterInit = true;
            } else {
              double moved = calculateDistance(
                  remoteLatFiltered, remoteLonFiltered, rawLat, rawLon);

              if (moved < DEADZONE_METERS) {
                // Đứng yên: static averaging
                remoteLatSum += rawLat;
                remoteLonSum += rawLon;
                remoteAvgCount++;
                remoteLatFiltered = remoteLatSum / remoteAvgCount;
                remoteLonFiltered = remoteLonSum / remoteAvgCount;
                remoteMoveSamples = 0; // reset bộ đếm kiểm chứng
              } else {
                remoteMoveSamples++;
                if (remoteMoveSamples >= 4) {
                  // Xác nhận di chuyển thực tế
                  remoteLatFiltered =
                      EMA_ALPHA * rawLat + (1.0 - EMA_ALPHA) * remoteLatFiltered;
                  remoteLonFiltered =
                      EMA_ALPHA * rawLon + (1.0 - EMA_ALPHA) * remoteLonFiltered;
                  remoteLatSum = remoteLatFiltered;
                  remoteLonSum = remoteLonFiltered;
                  remoteAvgCount = 1;
                } else {
                  // Nghi ngờ nhiễu nhảy vọt: Giữ lọc tĩnh ổn định
                  remoteLatSum += remoteLatFiltered;
                  remoteLonSum += remoteLonFiltered;
                  remoteAvgCount++;
                }
              }
            }

            remoteValid = true;
            lastLoRaReceived = millis();

            if (localFilterInit) {
              cachedDistance =
                  calculateDistance(localLatFiltered, localLonFiltered,
                                    remoteLatFiltered, remoteLonFiltered);
            }
          }
        }
      }

      loraBuffer = ""; // Reset buffer cho dòng tiếp theo
    } else {
      // Tích lũy ký tự, giới hạn buffer tránh tràn RAM
      if (loraBuffer.length() < LORA_BUF_MAX) {
        loraBuffer += c;
      } else {
        loraBuffer = ""; // Buffer overflow → bỏ dòng lỗi
      }
    }
  }
}

// =====================================================
// LCD
// =====================================================
void updateLCD() {
  if (millis() - lastLCDChange < LCD_INTERVAL)
    return;
  lastLCDChange = millis();
  lcd.clear();

  if (lcdScreen == 0) {
    if (localFilterInit) {
      lcd.setCursor(0, 0);
      lcd.print("L:");
      lcd.print(localLatFiltered, 4);
      lcd.setCursor(0, 1);
      lcd.print("O:");
      lcd.print(localLonFiltered, 4);
    } else {
      lcd.setCursor(0, 0);
      lcd.print("LocalGPS:");
      lcd.setCursor(0, 1);
      lcd.print("Waiting");
    }
  }

  else if (lcdScreen == 1) {
    if (remoteValid) {
      lcd.setCursor(0, 0);
      lcd.print("R:");
      lcd.print(remoteLatFiltered, 4);
      lcd.setCursor(0, 1);
      lcd.print("O:");
      lcd.print(remoteLonFiltered, 4);
    } else {
      lcd.setCursor(0, 0);
      lcd.print("RemoteGPS:");
      lcd.setCursor(0, 1);
      lcd.print("No Sender");
    }
  }

  else if (lcdScreen == 2) {
    lcd.setCursor(0, 0);
    lcd.print("Distance:");
    lcd.setCursor(0, 1);
    if (localFilterInit && remoteValid) {
      double lcdDist = cachedDistance;
      if (lcdDist < 0.0) lcdDist = 0.0;
      lcd.print(lcdDist, 2);
      lcd.print(" m");
    } else {
      lcd.print("No Distance");
    }
  }

  lcdScreen++;
  if (lcdScreen > 2)
    lcdScreen = 0;
}


// =====================================================
// SETUP
// =====================================================
void setup() {
  Wire.begin(6, 7);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Starting...");

  // Khởi động GPS trực tiếp (giữ Hot/Warm Start) + cấu hình 5Hz
  GPS.begin(9600, SERIAL_8N1, 4, 5);
  delay(200);
  GPS.write(setRate5Hz, sizeof(setRate5Hz)); // set 5Hz
  delay(200);

  // Khởi động cổng CAN Bus (TWAI) - GPIO 2 & 3, tốc độ 500kbps
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config  = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config  = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
  }

  LoRa.begin(9600, SERIAL_8N1, 8, 9);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("System Ready!");
  delay(1000);
}

// =====================================================
// LOOP
// =====================================================
void loop() {

  while (GPS.available())
    gps.encode(GPS.read());

  updateLocalGPSFilter();
  updateLoRa();
  updateLCD();

  // Gửi dữ liệu qua CAN Bus liên tục 5Hz (200ms) - Gửi 0.000000 nếu chưa bắt được GPS
  if (millis() - lastCANSend >= CAN_SEND_INTERVAL) {
    lastCANSend = millis();
    sendGPSOverCAN();
  }
}