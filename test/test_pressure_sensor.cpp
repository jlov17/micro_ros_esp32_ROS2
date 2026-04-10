#include <Arduino.h>
#include <Wire.h>

// I2C Configuration
const uint8_t I2C_SDA = 8;
const uint8_t I2C_SCL = 9;

// Multiplexer Configuration
#define TCAADDR 0x70  // TCA9548A I2C multiplexer address
#define PressureCh 0  // channel on MUX of Pressure sensor

// Pressure Sensor Configuration
#define PRE_SENSOR_ADDR 0x28
#define OUTPUT_MIN 1677722
#define OUTPUT_MAX 15099494
#define PRESSURE_MIN 0.0     // psi
#define PRESSURE_MAX 60.0    // psi

char printBuffer[200];

// Select channel on TCA9548A multiplexer
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read pressure from sensor on specified MUX channel
float readPressure(uint8_t channel) {
  if (channel > 4) { channel = 4; }
  
  tcaSelect(channel); // select channel from MUX
  uint8_t data[7]; // holds output data
  uint8_t cmd[3] = {0xAA, 0x00, 0x00}; // command to be sent
  float press_counts = 0; // digital pressure reading [counts]
  
  Wire.beginTransmission(PRE_SENSOR_ADDR);
  int stat = Wire.write(cmd, 3); // write command to the sensor
  stat |= Wire.endTransmission();
  delay(10); // check datasheet for this, this used to be 20.
  Wire.requestFrom(PRE_SENSOR_ADDR, 7); // read back Sensor data 7 bytes
  int i = 0;
  for (i = 0; i < 7; i++) {
    data[i] = Wire.read();
  }
  press_counts = data[3] + data[2] * 256 + data[1] * 65536; // calculate digital pressure counts
  float percentage = (press_counts / 16777215) * 100; // calculate pressure as percentage of full scale
  float pressure = ((press_counts - OUTPUT_MIN) * (PRESSURE_MAX - PRESSURE_MIN)) / (OUTPUT_MAX - OUTPUT_MIN) + PRESSURE_MIN;
  
  pressure = pressure * 6.89475729; // psi to kPa
  
  return pressure;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);  // 400kHz
  
  Serial.println("\n=== Pressure Sensor Test with TCA9548A Multiplexer ===");
  Serial.println("Testing pressure sensor on MUX channel " + String(PressureCh));
  
  // Test multiplexer communication
  Wire.beginTransmission(TCAADDR);
  int mux_error = Wire.endTransmission();
  if (mux_error == 0) {
    Serial.println("✓ TCA9548A multiplexer detected at 0x70");
  } else {
    Serial.println("⚠️  WARNING: Multiplexer not detected!");
  }
  
  // Test pressure sensor through multiplexer
  tcaSelect(PressureCh);
  Wire.beginTransmission(PRE_SENSOR_ADDR);
  int sensor_error = Wire.endTransmission();
  if (sensor_error == 0) {
    Serial.println("✓ Pressure sensor detected at 0x28 (through MUX ch" + String(PressureCh) + ")");
  } else {
    Serial.println("⚠️  WARNING: Pressure sensor not detected!");
  }
  
  Serial.println("\nPressure readings (kPa):\n");
}

void loop() {
  float pressure_kPa = readPressure(PressureCh);
  
  Serial.print("Channel ");
  Serial.print(PressureCh);
  Serial.print(": ");
  Serial.print(pressure_kPa, 3);
  Serial.println(" kPa");
  
  delay(1000);  // 1 Hz reading rate
}
