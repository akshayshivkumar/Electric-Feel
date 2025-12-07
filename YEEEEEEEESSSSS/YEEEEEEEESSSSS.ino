/*
 * I2C SENSORS - SERIAL PLOTTER READY (BYTE ORDER FIXED)
 * -----------------------------------------------------
 * Hardware: 3x MLX90393 Magnetometers (Addresses 0x0C, 0x0D, 0x0F)
 *
 * SERIAL PLOTTER INSTRUCTIONS:
 * 1. Upload this code.
 * 2. Close the Serial Monitor.
 * 3. Go to Tools > Serial Plotter (Ctrl+Shift+L).
 * 4. Ensure baud rate is set to 115200.
 */

#include <Wire.h>

// --- PIN DEFINITIONS (XIAO ESP32-C3) ---
#define I2C_SDA D3
#define I2C_SCL D5

// --- MLX90393 CONFIGURATION ---
const byte MLX_ADDR[] = {0x0C, 0x0D, 0x0F};
// Short names for the plotter legend
const char* PLOT_LABELS[] = {"S1", "S2", "S3"}; 

// --- HELPER: MANUALLY CLEAR STUCK BUS ---
void fixBusStuck() {
  pinMode(I2C_SCL, OUTPUT);
  pinMode(I2C_SDA, INPUT_PULLUP);
  
  digitalWrite(I2C_SCL, HIGH);
  delay(1);
  
  for(int i=0; i<10; i++) {
    digitalWrite(I2C_SCL, LOW);
    delayMicroseconds(10);
    digitalWrite(I2C_SCL, HIGH);
    delayMicroseconds(10);
  }
  
  digitalWrite(I2C_SDA, LOW);
  delayMicroseconds(10);
  digitalWrite(I2C_SCL, HIGH);
  delayMicroseconds(10);
  digitalWrite(I2C_SDA, HIGH);
  delay(10);
}

// --- MLX90393 READER (Modified for Plotter) ---
// Now accepts pointers to return X, Y, Z values instead of printing directly
bool readMlxSensor(int id, int16_t &valX, int16_t &valY, int16_t &valZ) {
  byte address = MLX_ADDR[id];
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
    // Return false so we can handle the error (plot 0 or last value)
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
    
    // CORRECTION: Read HIGH byte first, then LOW byte
    // The sensor sends MSB first.
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

void setup() {
  Serial.begin(115200);
  while(!Serial) delay(10); 

  // Initial debug prints are fine in setup (plotter usually ignores startup text)
  fixBusStuck();
  Wire.begin(I2C_SDA, I2C_SCL, 100000); 
  delay(100);
}

void loop() {
  // Variables to hold data for this frame
  int16_t dataX[3] = {0, 0, 0};
  int16_t dataY[3] = {0, 0, 0};
  int16_t dataZ[3] = {0, 0, 0};

  // Collect data from all 3 sensors
  for(int i=0; i<3; i++) {
    bool success = readMlxSensor(i, dataX[i], dataY[i], dataZ[i]);
    if (!success) {
      // If read fails, values stay 0 (visually obvious on plotter)
    }
  }

  // PRINT FOR SERIAL PLOTTER
  // Format: "Label:Value, Label:Value, ..."
  // All on ONE line per timestamp
  
  for(int i=0; i<3; i++) {
    Serial.print(PLOT_LABELS[i]); Serial.print("_X:");
    Serial.print(dataX[i]); Serial.print(",");
    
    Serial.print(PLOT_LABELS[i]); Serial.print("_Y:");
    Serial.print(dataY[i]); Serial.print(",");
    
    Serial.print(PLOT_LABELS[i]); Serial.print("_Z:");
    Serial.print(dataZ[i]); 
    
    // Add comma if not the last sensor
    if (i < 2) Serial.print(",");
  }
  
  // End the line to push the frame to the plotter
  Serial.println();

  delay(100); // Plotter refresh rate
}