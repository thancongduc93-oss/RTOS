#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <HardwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>

HardwareSerial GPS(1);
HardwareSerial LoRa(0);
TinyGPSPlus gps;

// =====================================================
// OLED — SDA=6, SCL=7
// =====================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
#define OLED_ADDR 0x3C

Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define BTN_PIN 20

bool lastBtnState = HIGH;
unsigned long lastDebounce = 0;
const unsigned long DEBOUNCE_MS = 150;

bool sending = false;
unsigned long lastSend = 0;
const unsigned long SEND_INTERVAL = 500; // Gửi mỗi 500ms để cập nhật nhanh hơn

unsigned long lastOLED = 0;
const unsigned long OLED_INTERVAL =
    500; // Cập nhật OLED mỗi 500ms (tránh gọi 5 lần/s ở 5Hz)

// ================= GPS 5Hz RATE =================
const uint8_t setRate5Hz[] = {0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
                              0xC8, 0x00, // 200ms = 5Hz
                              0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A};

// ================= OLED =================
void showOLED() {
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);

  if (gps.location.isValid()) {
    // Tiêu đề
    oled.setTextSize(1);
    oled.setCursor(0, 0);
    oled.print("=== GPS SENDER ===");

    // Lat / Lon chữ vừa
    oled.setCursor(0, 16);
    oled.print("Lat: ");
    oled.print(String(gps.location.lat(), 6));

    oled.setCursor(0, 30);
    oled.print("Lon: ");
    oled.print(String(gps.location.lng(), 6));

    // Trạng thái TX to, dễ nhìn
    oled.setTextSize(2);
    oled.setCursor(0, 46);
    oled.print("TX:");
    oled.print(sending ? "ON " : "OFF");
  } else {
    oled.setTextSize(1);
    oled.setCursor(0, 10);
    oled.print("Waiting for GPS...");

    oled.setTextSize(2);
    oled.setCursor(0, 36);
    oled.print("TX:");
    oled.print(sending ? "ON " : "OFF");
  }

  oled.display();
}

// ================= SETUP =================
void setup() {
  pinMode(BTN_PIN, INPUT_PULLUP);

  // SDA=6, SCL=7
  Wire.begin(7, 6);

  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    while (true)
      delay(1000); // treo nếu OLED lỗi
  }

  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setCursor(0, 0);
  oled.print("Starting...");
  oled.display();

  GPS.begin(9600, SERIAL_8N1, 4, 5);
  delay(200);
  GPS.write(setRate5Hz, sizeof(setRate5Hz)); // Set GPS 5Hz
  delay(200);

  LoRa.begin(9600, SERIAL_8N1, 9, 8);

  oled.clearDisplay();
  oled.setCursor(0, 10);
  oled.print("Waiting for GPS...");
  oled.setTextSize(2);
  oled.setCursor(0, 36);
  oled.print("TX:OFF");
  oled.display();
}

// ================= LOOP =================
void loop() {

  // ================= READ GPS =================
  while (GPS.available()) {
    gps.encode(GPS.read());
  }

  // Cập nhật OLED theo interval — tránh gọi 5 lần/s gây tốn I2C
  if (millis() - lastOLED >= OLED_INTERVAL) {
    lastOLED = millis();
    showOLED();
  }

  // ================= BUTTON =================
  static bool buttonHandled = false;
  bool btnState = digitalRead(BTN_PIN);

  if (btnState != lastBtnState) {
    lastDebounce = millis();
    lastBtnState = btnState;
  }

  if ((millis() - lastDebounce) > DEBOUNCE_MS) {
    if (btnState == LOW && !buttonHandled) {
      sending = !sending;
      showOLED();
      buttonHandled = true;
    }
    if (btnState == HIGH) {
      buttonHandled = false;
    }
  }

  // ================= SEND LORA =================
  if (sending && gps.location.isValid() &&
      (millis() - lastSend >= SEND_INTERVAL)) {
    lastSend = millis();
    String payload = "GPS:" + String(gps.location.lat(), 6) + "," +
                     String(gps.location.lng(), 6);
    LoRa.println(payload);
  }
}