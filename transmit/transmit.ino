/*
 * HAPTIC TRANSMITTER – 5x BOARDS × 3x MLX90393 EACH
 * -------------------------------------------------
 * - 5 sensor boards
 * - Each board has 3 MLX90393 at I2C addresses: 0x0C, 0x0D, 0x0F
 * - Boards are separated by different SDA pins, same SCL pin
 *
 * Behavior:
 * - Continuously reads X/Y/Z for all 5×3 sensors using your working MLX code.
 * - For each board:
 *      If ANY of its 3 sensors has |ΔX|, |ΔY|, or |ΔZ| > TRIGGER_THRESHOLD
 *      vs. previous reading, we:
 *        - Send "SPIKE detected at motor N" via ESP-NOW
 *        - Print same line on Serial
 *      where N = boardIndex + 1 (1..5)
 */

#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>

// ===================== CONFIG =========================

// MAC address of the RECEIVE (haptic) XIAO ESP32C3
// >>> CHANGE THIS TO YOUR RECEIVER'S WiFi MAC <<<
uint8_t RECEIVER_MAC[] = { 0x58, 0x8C, 0x81, 0xAE, 0x04, 0xB4 };

// Number of sensor boards
const int NUM_BOARDS = 5;

// One board per SDA pin (shared SCL)
const int SDA_PINS[NUM_BOARDS] = {
  D0,  // Board 1
  D1,  // Board 2
  D2,  // Board 3
  D3,  // Board 4 (your single-board test used D3)
  D4   // Board 5
};

// Common SCL for all boards
const int I2C_SCL = D5;

// MLX90393 addresses on each board
const byte MLX_ADDR[3] = { 0x0C, 0x0D, 0x0F };
const int SENSORS_PER_BOARD = 3;

// Spike detection parameters
const int16_t TRIGGER_THRESHOLD = 1000;      // tune this
const unsigned long DEBOUNCE_MS = 150;      // per board
const unsigned long SAMPLE_INTERVAL_MS = 20;

// =====================================================

// Previous values for delta detection
int16_t prevX[NUM_BOARDS][SENSORS_PER_BOARD];
int16_t prevY[NUM_BOARDS][SENSORS_PER_BOARD];
int16_t prevZ[NUM_BOARDS][SENSORS_PER_BOARD];
bool havePrev[NUM_BOARDS][SENSORS_PER_BOARD];

// Per-board debounce timer
unsigned long lastSpikeTime[NUM_BOARDS] = {0};

// Sample timer
unsigned long lastSampleTime = 0;

// ============ I2C / MLX SUPPORT (FROM YOUR WORKING CODE, ADAPTED) ==========

// Manually clear stuck bus for a specific SDA pin
void fixBusStuck(int sdaPin) {
  pinMode(I2C_SCL, OUTPUT);
  pinMode(sdaPin, INPUT_PULLUP);

  digitalWrite(I2C_SCL, HIGH);
  delay(1);

  for (int i = 0; i < 10; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(10);
  }

  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin, LOW);
  delayMicroseconds(10);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(sdaPin, HIGH);
  delay(10);
}

// Select which board we’re talking to by switching SDA
void selectBoard(int boardIndex) {
  int sdaPin = SDA_PINS[boardIndex];
  fixBusStuck(sdaPin);
  Wire.begin(sdaPin, I2C_SCL, 100000);
}

// Your MLX90393 reader, unchanged except for globals
bool readMlxSensor(int sensorIndex, int16_t &valX, int16_t &valY, int16_t &valZ) {
  byte address = MLX_ADDR[sensorIndex];
  byte error;

  // STEP 1: FORCE EXIT (Reset State)
  Wire.beginTransmission(address);
  Wire.write(0x80); // EXIT
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)address, (uint8_t)1); // Clear buffer
  delay(5);

  // STEP 2: START MEASUREMENT
  Wire.beginTransmission(address);
  Wire.write(0x3E); // SM | ZYX enabled
  error = Wire.endTransmission();

  if (error != 0) {
    return false;
  }

  // STEP 3: STATUS HANDSHAKE
  Wire.requestFrom((uint8_t)address, (uint8_t)1);

  // STEP 4: WAIT
  delay(20);

  // STEP 5: READ COMMAND
  Wire.beginTransmission(address);
  Wire.write(0x4E); // RM
  error = Wire.endTransmission();

  if (error != 0) return false;

  // STEP 6: GET DATA
  if (Wire.requestFrom((uint8_t)address, (uint8_t)7) == 7) {
    byte status = Wire.read();

    uint8_t x_h = Wire.read(); uint8_t x_l = Wire.read();
    uint8_t y_h = Wire.read(); uint8_t y_l = Wire.read();
    uint8_t z_h = Wire.read(); uint8_t z_l = Wire.read();

    valX = (x_h << 8) | x_l;
    valY = (y_h << 8) | y_l;
    valZ = (z_h << 8) | z_l;
    return true;
  }

  return false;
}

// ================== ESP-NOW SUPPORT ===================

// New ESP32 core: send callback uses wifi_tx_info_t*
void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

// Send spike message for a board (mapped to motor index)
void sendSpike(int boardIndex) {
  uint8_t motorIndex = boardIndex + 1; // 0..4 -> 1..5

  char msg[40];
  snprintf(msg, sizeof(msg), "SPIKE detected at motor %u", motorIndex);

  Serial.println(msg);  // on TRANSMIT serial monitor

  esp_err_t result = esp_now_send(RECEIVER_MAC, (uint8_t*)msg, strlen(msg) + 1);
  if (result != ESP_OK) {
    Serial.print("esp_now_send error: ");
    Serial.println(result);
  }
}

// ===================== SETUP ==========================

void setup() {
  Serial.begin(115200);
  while (!Serial) { delay(10); }

  Serial.println();
  Serial.println("=== HAPTIC TRANSMITTER – 5x BOARDS × 3x MLX90393 ===");

  // Init I2C on first board just to get bus sane
  selectBoard(0);

  // Clear prev flags
  for (int b = 0; b < NUM_BOARDS; b++) {
    for (int s = 0; s < SENSORS_PER_BOARD; s++) {
      havePrev[b][s] = false;
    }
  }

  // Wi-Fi + ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    while (true) { delay(100); }
  }

  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, RECEIVER_MAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add ESP-NOW peer");
    while (true) { delay(100); }
  }

  Serial.println("Setup complete.");
}

// ====================== LOOP ==========================

void loop() {
  unsigned long now = millis();
  if (now - lastSampleTime < SAMPLE_INTERVAL_MS) return;
  lastSampleTime = now;

  // For each board
  for (int b = 0; b < NUM_BOARDS; b++) {
    bool boardTriggered = false;

    // Switch I2C to this board
    selectBoard(b);

    // For each sensor on that board
    for (int s = 0; s < SENSORS_PER_BOARD; s++) {
      int16_t x, y, z;
      bool ok = readMlxSensor(s, x, y, z);
      if (!ok) {
        // Failed read, skip; don't update prev
        continue;
      }

      if (!havePrev[b][s]) {
        prevX[b][s] = x;
        prevY[b][s] = y;
        prevZ[b][s] = z;
        havePrev[b][s] = true;
        continue;
      }

      int16_t dx = abs(x - prevX[b][s]);
      int16_t dy = abs(y - prevY[b][s]);
      int16_t dz = abs(z - prevZ[b][s]);

      if (dx > TRIGGER_THRESHOLD || dy > TRIGGER_THRESHOLD || dz > TRIGGER_THRESHOLD) {
        boardTriggered = true;
      }

      prevX[b][s] = x;
      prevY[b][s] = y;
      prevZ[b][s] = z;
    }

    if (boardTriggered && (now - lastSpikeTime[b] > DEBOUNCE_MS)) {
      sendSpike(b);
      lastSpikeTime[b] = now;
    }
  }
}
