/*
 * MAGNETIC HAPTIC RECEIVER – DEBUG VERSION
 * Board: Seeed Studio XIAO ESP32C3
 *
 * Listens for ESP-NOW messages:
 *   - "M1", "M2", ..., "M5"
 *   - or "SPIKE detected at motor 1..5"
 *
 * On a valid motor index:
 *   - Prints debug
 *   - Drives that motor pin at 50% duty for 200 ms
 */

// ------------ CONFIG -----------------
#include <WiFi.h>
#include <esp_now.h>

const int NUM_MOTORS = 5;

// Motor mapping (change if your wiring differs)
const int MOTOR_PINS[NUM_MOTORS] = {
  D5, // Motor 1
  D4, // Motor 2
  D3, // Motor 3
  D2, // Motor 4
  D1  // Motor 5
};

const int PWM_50   = 2555;      // 50% duty on 0..255
const unsigned long ON_TIME_MS = 200;

// Per-motor off times; 0 means OFF
unsigned long motorOffTime[NUM_MOTORS] = {0};

// ------------ ESP-NOW RECV CALLBACK (new core signature) ------------

void OnDataRecv(const esp_now_recv_info_t *info,
                const uint8_t *data,
                int len) {
  if (len <= 0) return;

  char msg[64];
  int copyLen = len;
  if (copyLen >= (int)sizeof(msg)) copyLen = sizeof(msg) - 1;
  memcpy(msg, data, copyLen);
  msg[copyLen] = '\0';

  Serial.print("[RX] Raw message: ");
  Serial.println(msg);

  // 1) Try "M1".."M5" format
  int motorIndex = -1; // 0-based index into MOTOR_PINS

  if (msg[0] == 'M' && copyLen >= 2 && msg[1] >= '1' && msg[1] <= '5') {
    motorIndex = (msg[1] - '0') - 1;  // "M1" -> 0, "M5" -> 4
  } else {
    // 2) Try "SPIKE detected at motor X" style: search for first digit 1..5
    for (int i = 0; i < copyLen; i++) {
      if (msg[i] >= '1' && msg[i] <= '5') {
        motorIndex = (msg[i] - '0') - 1;
        break;
      }
    }
  }

  if (motorIndex < 0 || motorIndex >= NUM_MOTORS) {
    Serial.println("[RX] No valid motor index in message.");
    return;
  }

  int motorNumber = motorIndex + 1;
  int pin = MOTOR_PINS[motorIndex];

  Serial.print("[RX] Activating motor ");
  Serial.print(motorNumber);
  Serial.print(" on pin ");
  Serial.println(pin);

  analogWrite(pin, PWM_50);
  motorOffTime[motorIndex] = millis() + ON_TIME_MS;
}

// ---------------- SETUP -----------------

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println();
  Serial.println("=== MAGNETIC HAPTIC RECEIVER – DEBUG ===");

  // Configure motor pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(MOTOR_PINS[i], OUTPUT);
    analogWrite(MOTOR_PINS[i], 0);
    Serial.print("[SETUP] Motor ");
    Serial.print(i + 1);
    Serial.print(" -> pin ");
    Serial.println(MOTOR_PINS[i]);
  }

  // WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) { delay(100); }
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("[SETUP] Ready to receive ESP-NOW messages.");
}

// ---------------- LOOP ------------------

void loop() {
  unsigned long now = millis();

  // Turn off motors whose on-time expired
  for (int i = 0; i < NUM_MOTORS; i++) {
    if (motorOffTime[i] != 0 && now >= motorOffTime[i]) {
      analogWrite(MOTOR_PINS[i], 0);
      motorOffTime[i] = 0;

      Serial.print("[RX] Motor ");
      Serial.print(i + 1);
      Serial.println(" OFF");
    }
  }

  delay(1);
}
