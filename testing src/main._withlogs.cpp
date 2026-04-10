#if 0  // Disabled 
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <TMCStepper.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/bool.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pin definitions - Both drivers on same SPI bus
constexpr uint8_t CS_PIN = 7;      // Chip Select for Driver 1 (M0, M1)
constexpr uint8_t CS_PIN2 = 18;     // Chip Select for Driver 2 (M2, M3)
constexpr uint8_t EN_PIN = 3;      // Enable pin (shared by both TMC5272 chips)
constexpr float RSENSE = 0.075f;

// I2C pins for pressure sensor
constexpr uint8_t I2C_SDA = 10;
constexpr uint8_t I2C_SCL = 8;

// Pressure sensor definitions
const float PRESSURE_MIN = 0.0;   // psi
const float PRESSURE_MAX = 60;  // psi
const int OUTPUT_MIN = 1677722;      // 
const int OUTPUT_MAX = 15099494;     //
const uint8_t PRE_SENSOR_ADDR = 0x28;    // Address of ABP2LANT060PG2A3XX sensor
const uint8_t TCA9548A_ADDR = 0x70;  // Default I2C address of the TCA9548A

// Number of syringes/axes (2 motors per TMC5272 chip × 2 chips)
const uint8_t NUM_SYRINGES = 4;

// Create TMC5272 drivers (inherits SPI from TMC2130)
TMC5272Stepper driver(CS_PIN, RSENSE);      // Driver 1: M0, M1
TMC5272Stepper driver2(CS_PIN2, RSENSE);    // Driver 2: M2, M3

// Micro-ROS variables
rcl_publisher_t motor_pos_publisher;
rcl_publisher_t encoder_pos_publisher;
rcl_publisher_t pressure_publisher;
rcl_publisher_t pump_status_publisher;
rcl_subscription_t pump_home_subscriber;
rcl_subscription_t motor_cmd_subscriber;
std_msgs__msg__Int32MultiArray motor_pos_msg;
std_msgs__msg__Int32MultiArray encoder_pos_msg;
std_msgs__msg__Float32MultiArray pressure_msg;
std_msgs__msg__Int32MultiArray motor_cmd_msg;
std_msgs__msg__Bool pump_home_msg;
std_msgs__msg__Bool pump_status_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;
rcl_timer_t pressure_timer;

// Storage for array data (4 motors total: M0, M1, M2, M3)
int32_t motor_pos_data[NUM_SYRINGES];
int32_t encoder_pos_data[NUM_SYRINGES];
float pressure_data[NUM_SYRINGES];
int32_t motor_cmd_data[NUM_SYRINGES];

// Mutex for thread-safe pressure data access
SemaphoreHandle_t pressure_mutex;

// Command flags
volatile bool homing_requested = false;

// Forward declarations
float readPressure(uint8_t channel);

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Update motor and encoder positions for all syringes (4 motors total)
    for (uint8_t i = 0; i < NUM_SYRINGES; i++) {
      // Read motor position and encoder from appropriate driver
      if (i == 0) {
        motor_pos_data[i] = driver.M0_XACTUAL();
        encoder_pos_data[i] = driver.M0_X_ENC();
      } else if (i == 1) {
        motor_pos_data[i] = driver.M1_XACTUAL();
        encoder_pos_data[i] = driver.M1_X_ENC();
      } else if (i == 2) {
        motor_pos_data[i] = driver2.M0_XACTUAL();
        encoder_pos_data[i] = driver2.M0_X_ENC();
      } else if (i == 3) {
        motor_pos_data[i] = driver2.M1_XACTUAL();
        encoder_pos_data[i] = driver2.M1_X_ENC();
      }
    }
    
    // Publish motor positions
    RCSOFTCHECK(rcl_publish(&motor_pos_publisher, &motor_pos_msg, NULL));

    // Publish encoder positions
    RCSOFTCHECK(rcl_publish(&encoder_pos_publisher, &encoder_pos_msg, NULL));
  }
}

void pressure_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read pressure data (updated by background task)
    if (xSemaphoreTake(pressure_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      // Data is already updated by pressure task - just publish
      RCSOFTCHECK(rcl_publish(&pressure_publisher, &pressure_msg, NULL));
      xSemaphoreGive(pressure_mutex);
    }
  }
}

// Background task to continuously read pressure sensors
void pressureTask(void *parameter) {
  while (true) {
    // Read all 4 pressure sensors
    float temp_pressure[NUM_SYRINGES];
    for (uint8_t i = 0; i < NUM_SYRINGES; i++) {
      temp_pressure[i] = readPressure(i);
    }
    
    // Update shared data with mutex protection
    if (xSemaphoreTake(pressure_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      for (uint8_t i = 0; i < NUM_SYRINGES; i++) {
        pressure_data[i] = temp_pressure[i];
      }
      xSemaphoreGive(pressure_mutex);
    }
    
    vTaskDelay(pdMS_TO_TICKS(50)); // Read every 50ms (20Hz)
  }
}

// Callback for /pump/home subscriber
void pump_home_callback(const void * msgin) {
  const std_msgs__msg__Bool * msg = (const std_msgs__msg__Bool *)msgin;
  if (msg->data) {
    homing_requested = true;
  }
}

// Callback for motor command subscriber
void motor_cmd_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  // Apply commands to each motor (4 motors total)
  for (size_t i = 0; i < msg->data.size && i < NUM_SYRINGES; i++) {
    if (i == 0) {
      driver.M0_XTARGET(msg->data.data[i]);
    } else if (i == 1) {
      driver.M1_XTARGET(msg->data.data[i]);
    } else if (i == 2) {
      driver2.M0_XTARGET(msg->data.data[i]);
    } else if (i == 3) {
      driver2.M1_XTARGET(msg->data.data[i]);
    }
  }
}

// Simultaneous homing for all 4 motors (2 per TMC5272 driver)
void performHoming() {
  Serial.println("\n=== STARTING SIMULTANEOUS HOMING FOR ALL 4 MOTORS ===");
  
  // State tracking for each motor (M0-M3)
  enum HomingState {
    HOME_CHECK_INITIAL,
    HOME_RELEASE_FORWARD,
    HOME_APPROACH_BACKWARD,
    HOME_BACKOFF_FORWARD,
    HOME_COMPLETE
  };
  
  HomingState motor_state[4] = {HOME_CHECK_INITIAL, HOME_CHECK_INITIAL, HOME_CHECK_INITIAL, HOME_CHECK_INITIAL};
  
  // SW_MODE configurations (same for all motors)
  uint32_t sw_mode_approach = 0;
  sw_mode_approach |= (1 << 1);   // stop_r_enable
  sw_mode_approach |= (1 << 3);   // pol_stop_r = 1 (active LOW)
  sw_mode_approach |= (1 << 7);   // latch_r_active
  sw_mode_approach |= (1 << 11);  // en_softstop
  
  uint32_t sw_mode_backoff = 0;
  sw_mode_backoff |= (1 << 3);    // pol_stop_r = 1 (active LOW)
  sw_mode_backoff |= (1 << 7);    // latch_r_active
  sw_mode_backoff |= (1 << 11);   // en_softstop
  
  // Configure all motors
  driver.M0_SW_MODE(sw_mode_approach);
  driver.M1_SW_MODE(sw_mode_approach);
  driver2.M0_SW_MODE(sw_mode_approach);
  driver2.M1_SW_MODE(sw_mode_approach);
  delay(2);
  
  // Clear status
  driver.M0_RAMP_STAT();
  driver.M1_RAMP_STAT();
  driver2.M0_RAMP_STAT();
  driver2.M1_RAMP_STAT();
  delay(2);
  
  // Read initial switch states
  uint32_t ramp_stat[4];
  bool switch_state[4];
  
  ramp_stat[0] = driver.M0_RAMP_STAT();
  ramp_stat[1] = driver.M1_RAMP_STAT();
  ramp_stat[2] = driver2.M0_RAMP_STAT();
  ramp_stat[3] = driver2.M1_RAMP_STAT();
  
  for (int i = 0; i < 4; i++) {
    switch_state[i] = (ramp_stat[i] & (1 << 1));
    
    if (switch_state[i]) {
      Serial.print("M"); Serial.print(i); Serial.println(": Switch pressed → releasing...");
      if (i == 0) driver.M0_SW_MODE(sw_mode_backoff);
      else if (i == 1) driver.M1_SW_MODE(sw_mode_backoff);
      else if (i == 2) driver2.M0_SW_MODE(sw_mode_backoff);
      else if (i == 3) driver2.M1_SW_MODE(sw_mode_backoff);
      motor_state[i] = HOME_RELEASE_FORWARD;
    } else {
      Serial.print("M"); Serial.print(i); Serial.println(": Starting approach...");
      motor_state[i] = HOME_APPROACH_BACKWARD;
    }
  }
  
  // Set initial velocities
  for (int i = 0; i < 4; i++) {
    uint32_t vmax = (motor_state[i] == HOME_RELEASE_FORWARD) ? 10000 : 15000;
    if (i == 0) driver.M0_VMAX(vmax);
    else if (i == 1) driver.M1_VMAX(vmax);
    else if (i == 2) driver2.M0_VMAX(vmax);
    else if (i == 3) driver2.M1_VMAX(vmax);
  }
  
  // Start motors simultaneously (RAMPMODE: [3:2]=M1, [1:0]=M0 per driver)
  uint8_t m0_mode = (motor_state[0] == HOME_RELEASE_FORWARD) ? 1 : 2;
  uint8_t m1_mode = (motor_state[1] == HOME_RELEASE_FORWARD) ? 1 : 2;
  uint32_t rampmode_driver1 = (m1_mode << 2) | m0_mode;
  driver.RAMPMODE(rampmode_driver1);
  
  uint8_t m2_mode = (motor_state[2] == HOME_RELEASE_FORWARD) ? 1 : 2;
  uint8_t m3_mode = (motor_state[3] == HOME_RELEASE_FORWARD) ? 1 : 2;
  uint32_t rampmode_driver2 = (m3_mode << 2) | m2_mode;
  driver2.RAMPMODE(rampmode_driver2);
  
  uint32_t lastPrint = 0;
  
  // Main state machine loop - homing all 4 motors simultaneously
  bool all_complete = false;
  while (!all_complete) {
    all_complete = true;
    
    // Process each motor
    for (int i = 0; i < 4; i++) {
      if (motor_state[i] == HOME_COMPLETE) continue;
      all_complete = false;
      
      // Get current ramp status and switch state
      if (i == 0) ramp_stat[i] = driver.M0_RAMP_STAT();
      else if (i == 1) ramp_stat[i] = driver.M1_RAMP_STAT();
      else if (i == 2) ramp_stat[i] = driver2.M0_RAMP_STAT();
      else if (i == 3) ramp_stat[i] = driver2.M1_RAMP_STAT();
      
      switch_state[i] = (ramp_stat[i] & (1 << 1));
      
      switch (motor_state[i]) {
        case HOME_RELEASE_FORWARD:
          if (!switch_state[i]) {
            Serial.print("M"); Serial.print(i); Serial.println(": Switch released → complete");
            if (i == 0) {
              driver.M0_VMAX(0);
              while (driver.M0_VACTUAL() != 0) delay(5);
              driver.M0_XACTUAL(0);
            } else if (i == 1) {
              driver.M1_VMAX(0);
              while (driver.M1_VACTUAL() != 0) delay(5);
              driver.M1_XACTUAL(0);
            } else if (i == 2) {
              driver2.M0_VMAX(0);
              while (driver2.M0_VACTUAL() != 0) delay(5);
              driver2.M0_XACTUAL(0);
            } else if (i == 3) {
              driver2.M1_VMAX(0);
              while (driver2.M1_VACTUAL() != 0) delay(5);
              driver2.M1_XACTUAL(0);
            }
            motor_state[i] = HOME_COMPLETE;
          }
          break;
          
        case HOME_APPROACH_BACKWARD:
          if (switch_state[i]) {
            Serial.print("M"); Serial.print(i); Serial.println(": Switch hit → backing off...");
            
            if (i == 0) {
              driver.M0_VMAX(0);
              while (driver.M0_VACTUAL() != 0) delay(5);
              driver.M0_SW_MODE(sw_mode_backoff);
              delay(2);
              driver.M0_VMAX(10000);
            } else if (i == 1) {
              driver.M1_VMAX(0);
              while (driver.M1_VACTUAL() != 0) delay(5);
              driver.M1_SW_MODE(sw_mode_backoff);
              delay(2);
              driver.M1_VMAX(10000);
            } else if (i == 2) {
              driver2.M0_VMAX(0);
              while (driver2.M0_VACTUAL() != 0) delay(5);
              driver2.M0_SW_MODE(sw_mode_backoff);
              delay(2);
              driver2.M0_VMAX(10000);
            } else if (i == 3) {
              driver2.M1_VMAX(0);
              while (driver2.M1_VACTUAL() != 0) delay(5);
              driver2.M1_SW_MODE(sw_mode_backoff);
              delay(2);
              driver2.M1_VMAX(10000);
            }
            
            motor_state[i] = HOME_BACKOFF_FORWARD;
            
            // Update RAMPMODE for changed motor
            if (i < 2) {
              uint8_t new_m0_mode = (i == 0) ? 1 : ((motor_state[0] == HOME_COMPLETE) ? 0 : ((motor_state[0] == HOME_RELEASE_FORWARD || motor_state[0] == HOME_BACKOFF_FORWARD) ? 1 : 2));
              uint8_t new_m1_mode = (i == 1) ? 1 : ((motor_state[1] == HOME_COMPLETE) ? 0 : ((motor_state[1] == HOME_RELEASE_FORWARD || motor_state[1] == HOME_BACKOFF_FORWARD) ? 1 : 2));
              driver.RAMPMODE((new_m1_mode << 2) | new_m0_mode);
            } else {
              uint8_t new_m2_mode = (i == 2) ? 1 : ((motor_state[2] == HOME_COMPLETE) ? 0 : ((motor_state[2] == HOME_RELEASE_FORWARD || motor_state[2] == HOME_BACKOFF_FORWARD) ? 1 : 2));
              uint8_t new_m3_mode = (i == 3) ? 1 : ((motor_state[3] == HOME_COMPLETE) ? 0 : ((motor_state[3] == HOME_RELEASE_FORWARD || motor_state[3] == HOME_BACKOFF_FORWARD) ? 1 : 2));
              driver2.RAMPMODE((new_m3_mode << 2) | new_m2_mode);
            }
          }
          break;
          
        case HOME_BACKOFF_FORWARD:
          if (!switch_state[i]) {
            Serial.print("M"); Serial.print(i); Serial.println(": Released → homing complete");
            if (i == 0) {
              driver.M0_VMAX(0);
              while (driver.M0_VACTUAL() != 0) delay(5);
              driver.M0_XACTUAL(0);
            } else if (i == 1) {
              driver.M1_VMAX(0);
              while (driver.M1_VACTUAL() != 0) delay(5);
              driver.M1_XACTUAL(0);
            } else if (i == 2) {
              driver2.M0_VMAX(0);
              while (driver2.M0_VACTUAL() != 0) delay(5);
              driver2.M0_XACTUAL(0);
            } else if (i == 3) {
              driver2.M1_VMAX(0);
              while (driver2.M1_VACTUAL() != 0) delay(5);
              driver2.M1_XACTUAL(0);
            }
            motor_state[i] = HOME_COMPLETE;
          }
          break;
          
        default:
          break;
      }
    }
    
    // Progress update every 100ms
    if (millis() - lastPrint > 100) {
      Serial.print("M0: "); Serial.print(driver.M0_XACTUAL());
      Serial.print(" | M1: "); Serial.print(driver.M1_XACTUAL());
      Serial.print(" | M2: "); Serial.print(driver2.M0_XACTUAL());
      Serial.print(" | M3: "); Serial.println(driver2.M1_XACTUAL());
      lastPrint = millis();
    }
    
    delay(10);
  }
  
  // Stop all motors by setting RAMPMODE to 0 (position mode)
  driver.RAMPMODE(0);
  driver2.RAMPMODE(0);
  
  Serial.println("\n✓ ALL 4 MOTORS HOMED SUCCESSFULLY");
  Serial.print("M0 Home: "); Serial.print(driver.M0_XACTUAL());
  Serial.print(" | M1 Home: "); Serial.print(driver.M1_XACTUAL());
  Serial.print(" | M2 Home: "); Serial.print(driver2.M0_XACTUAL());
  Serial.print(" | M3 Home: "); Serial.println(driver2.M1_XACTUAL());
  Serial.println("=== SIMULTANEOUS HOMING COMPLETE ===\n");
  
  // Publish homing complete status
  pump_status_msg.data = true;
  RCSOFTCHECK(rcl_publish(&pump_status_publisher, &pump_status_msg, NULL));
}

// Select channel on TCA9548A multiplexer
void selectMuxChannel(uint8_t channel) {
  if (channel > 7) return; // Valid channels are 0-7
  
  Wire.beginTransmission(TCA9548A_ADDR);
  Wire.write(1 << channel);  // Send byte to select channel
  Wire.endTransmission();
}

float readPressure(uint8_t channel) {
  // Select the appropriate multiplexer channel
  selectMuxChannel(channel);
  delay(10);  // Longer delay for mux to switch

  uint8_t data[7];                 // holds output data
  uint8_t cmd[3] = {0xAA, 0x00, 0x00};  // command to be sent
  float press_counts = 0;          // digital pressure reading [counts]

  Wire.beginTransmission(PRE_SENSOR_ADDR);
  int stat = Wire.write(cmd, 3);   // write command to the sensor
  int end_stat = Wire.endTransmission();
  
  if (end_stat != 0) {
    Serial.print("[Pressure Ch");
    Serial.print(channel);
    Serial.print("] Write error: ");
    Serial.print(end_stat);
    Serial.print(" (wrote ");
    Serial.print(stat);
    Serial.println(" bytes)");
    return NAN;
  }

  delay(10);

  int n = Wire.requestFrom((int)PRE_SENSOR_ADDR, 7); // read back 7 bytes
  if (n != 7) {
    Serial.print("[Pressure] Expected 7 bytes, got ");
    Serial.println(n);
    return NAN; // sensor not responding
  }

  int i = 0;
  for (i = 0; i < 7; i++) {
    data[i] = Wire.read();
  }

  press_counts = data[3] + data[2] * 256 + data[1] * 65536; // calculate digital pressure counts

  float pressure = ((press_counts - OUTPUT_MIN) * (PRESSURE_MAX - PRESSURE_MIN)) /
                   (OUTPUT_MAX - OUTPUT_MIN) + PRESSURE_MIN; // psi

  pressure = pressure * 6.89475729; // psi to kPa

  return pressure;
}


void setup() {
  Serial.begin(115200);
  delay(500);
  
  Serial.println("\n\n=== TMC5272 Global Registers Test (0x00-0x09) ===");

  // Initialize I2C for pressure sensor
  Wire.begin(I2C_SDA, I2C_SCL);
  Serial.print("I2C initialized for pressure sensor (SDA=");
  Serial.print(I2C_SDA);
  Serial.print(", SCL=");
  Serial.print(I2C_SCL);
  Serial.println(")");

  // Scan I2C bus for devices
  Serial.println("\n=== Scanning I2C Bus ===");
  uint8_t devices_found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    uint8_t error = Wire.endTransmission();
    if (error == 0) {
      Serial.print("Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      devices_found++;
    }
  }
  if (devices_found == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(devices_found);
    Serial.println(" I2C device(s)");
  }
  Serial.println("=== I2C Scan Complete ===\n");

  // Test multiplexer channel switching and scan for sensors
  Serial.println("=== Testing Multiplexer Channels ===");
  for (uint8_t ch = 0; ch < 4; ch++) {
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(": ");
    
    // Select channel
    Wire.beginTransmission(TCA9548A_ADDR);
    Wire.write(1 << ch);
    if (Wire.endTransmission() != 0) {
      Serial.println("ERROR - Cannot select channel");
      continue;
    }
    delay(10);
    
    // Scan for sensor on this channel
    Wire.beginTransmission(PRE_SENSOR_ADDR);
    if (Wire.endTransmission() == 0) {
      Serial.println("✓ Sensor found at 0x28");
    } else {
      Serial.println("✗ No sensor at 0x28");
    }
  }
  Serial.println("=== Channel Test Complete ===\n");

  // Test multiplexer and pressure sensors
  Serial.println("=== Testing Multiplexer and Pressure Sensors ===");
  for (uint8_t ch = 0; ch < 4; ch++) {
    Serial.print("Channel ");
    Serial.print(ch);
    Serial.print(": ");
    float pressure = readPressure(ch);
    if (isnan(pressure)) {
      Serial.println("ERROR - No sensor detected");
    } else {
      Serial.print(pressure);
      Serial.println(" kPa");
    }
    delay(100);
  }
  Serial.println("=== Pressure Test Complete ===\n");

  // Initialize SPI
  SPI.begin();
  Serial.println("SPI initialized with default settings");

  // Increase SPI clock for faster register reads
  driver.setSPISpeed(8000000); // 8 MHz
  
  // Setup pins
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  Serial.println("CS_PIN set HIGH");
  
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);  // SLEEPn must be HIGH to wake TMC5272
  Serial.print("SLEEPn (EN_PIN GPIO");
  Serial.print(EN_PIN);
  Serial.println(digitalRead(EN_PIN) ? "HIGH ✓" : "LOW ✗");
  delay(100);
  
  delay(500);  // Longer delay for chip power-up
  Serial.println("\nWaiting 500ms for chip power-up and initialization...");
  
  // Initialize driver
  Serial.println("\n--- Initializing TMC5272 Driver ---");
  driver.begin();
  

  // Disable PWM mode - use pure SpreadCycle for testing
  // Bit 0: M0_en_pwm_mode=0 (SpreadCycle only)
  // Bit 1: M0_multistep_filt=1 (keep default)
  // Bit 14: M0_drv_enn=0 (enable driver, active low)
  uint32_t gconf_init = 0x00020002;  // Both motors: SpreadCycle + multistep filter
  driver.GCONF(gconf_init);
  delay(50);
  
  Serial.print("GCONF written: 0x"); Serial.println(gconf_init, HEX);
  uint32_t gconf_read = driver.GCONF();
  Serial.print("GCONF readback: 0x"); Serial.println(gconf_read, HEX);
  driver.defaults();
  Serial.println("Driver initialized");
  delay(100);

  
  uint32_t ioin = driver.IOIN();
  uint8_t status = driver.status_response;
  Serial.print("IOIN = 0x"); Serial.print(ioin, HEX);
  Serial.print(" | Status byte = 0x"); Serial.println(status, HEX);

  
  // Check if VERSION is readable (should be non-zero if chip responds)
  Serial.println("\nReading VERSION field...");
  uint8_t ver = driver.version();
  uint8_t status_ver = driver.status_response;
  Serial.print("VERSION: 0x"); Serial.print(ver, HEX);
  Serial.print(" | Status: 0x"); Serial.println(status_ver, HEX);
  if (ver == 0) {
    Serial.println(" ⚠️  WARNING: VERSION is 0!");
    Serial.println("   NO SPI communication!");
  } else {
    Serial.print(" ✓ Chip detected! TMC5272 version: ");
    Serial.println(ver);
  }

  
  // Read and clear GSTAT
  uint32_t gstat = driver.GSTAT();
  if (gstat != 0) {
    Serial.print("GSTAT: 0x"); Serial.print(gstat, HEX);
    Serial.println(" - Clearing error flags...");
    driver.GSTAT(0x0000003F);  // Clear all GSTAT bits
    delay(10);
  }
  
  // Configure essential global registers
  driver.GCONF(0x00000000);           // Global configuration
  driver.NODECONF(0x00000000);        // Node configuration (UART)
  driver.DRV_CONF(0x0000CFCF);        // Driver config: equal drive strength for both motors
  driver.GLOBALSCALER(0x00000000);    // No global scaling
  driver.RAMPMODE(0x00000000);        // Position mode
  driver.MSLUT_ADDR(0x00);            // Microstep table address
  driver.MSLUT_DATA(0x00000000);      // Microstep table data
  
  Serial.println("\n=== Register Setup Complete ===");
  Serial.print("DRV_CONF readback: 0x"); Serial.println(driver.DRV_CONF(), HEX);
  
  // Configure GLOBALSCALER for full current
  driver.GLOBALSCALER(0x00000000);  // 0 = Full scale (256/256)
  
  // Configure Motor 0 current settings (reduced for less heat)
  // IHOLD=8 (~25% hold), IRUN=16 (~50% run), IHOLDDELAY=10
  driver.M0_IHOLD_IRUN(0x000A1008);
  
  // Enable Motor 0 driver with CHOPCONF (TOFF=3, MRES=0 for 256 microsteps)
  driver.M0_CHOPCONF(0x00000003);
  Serial.println("\n*** DRIVER ENABLED! ***");
  
  // Configure Motion Control Registers
  driver.RAMPMODE(0);  // Position mode
  
  // Set start/stop velocities (VSTART must be <= VSTOP)
  driver.M0_VSTART(500);
  driver.M0_VSTOP(700);
  Serial.println("VSTART=500, VSTOP=700");
  
  // Set acceleration profile
  uint32_t a1_val = 3000;   // Initial acceleration (reduced)
  uint32_t amax_val = 50000; // Maximum acceleration (reduced)
  driver.M0_A1(a1_val);
  driver.M0_AMAX(amax_val);
  Serial.print("A1="); Serial.print(a1_val);
  Serial.print(", AMAX="); Serial.println(amax_val);
  
  // Set deceleration profile
  uint32_t d1_val = 3000;    // Initial deceleration (reduced)
  uint32_t dmax_val = 50000; // Maximum deceleration (reduced)
  driver.M0_D1(d1_val);
  driver.M0_DMAX(dmax_val);
  Serial.print("D1="); Serial.print(d1_val);
  Serial.print(", DMAX="); Serial.println(dmax_val);
  
  // Set velocity profile
  uint32_t v1_val = 10000;    // First velocity threshold (reduced)
  uint32_t vmax_val = 50000;  // Maximum velocity (reduced to 25% of original)
  driver.M0_V1(v1_val);
  driver.M0_VMAX(vmax_val);

  // Verify motor is holding position at startup
  int32_t current_pos = driver.M0_XACTUAL();
  driver.M0_XTARGET(current_pos);  // Set target to current position (no movement)
  
  Serial.println("\n✓ Motor M0 configured");
  
  // ========== Configure M0 Encoder ==========
  // Use working ENCMODE from reference code
  // ENCMODE = 0x00000900 enables ABN encoder interface properly
  driver.M0_ENCMODE(0x00000900);
  
  // Set encoder constant - use 12.8 in U16.16 fixed-point format
  // This scales encoder counts to match motor steps
  driver.M0_ENC_CONST(0x000C1F40);  // 12.8 fixed-point
  
  // Reset encoder position
  driver.M0_X_ENC(0);
  
  // ========== Configure Motor 1 (M1) ==========
  Serial.println("\n=== Configuring Motor M1 ===");
  
  // Configure Motor 1 current settings BEFORE enabling driver
  driver.M1_IHOLD_IRUN(0x000A1008);  // IHOLD=8, IRUN=16, IHOLDDELAY=10 (match M0)
  delay(10);
  
  // Enable Motor 1 driver with CHOPCONF
  // TOFF=3, MRES=0 for 256 microsteps, freewheel=0 (normal operation)
  driver.M1_CHOPCONF(0x00000003);
  delay(50);
  
  // Write IHOLD_IRUN again after CHOPCONF to ensure it takes
  driver.M1_IHOLD_IRUN(0x000A1008);
  delay(10);
  driver.M1_IHOLD_IRUN(0x000A1008);  // Write third time
  delay(50);
  
  Serial.print("M1_CHOPCONF written: 0x3, readback: 0x");
  Serial.println(driver.M1_CHOPCONF(), HEX);
  Serial.print("M1_IHOLD_IRUN written: 0xA1008, readback: 0x");
  Serial.println(driver.M1_IHOLD_IRUN(), HEX);
  
  // Check DRV_STATUS for M1
  uint32_t m1_drv_status = driver.M1_DRV_STATUS();
  Serial.print("M1_DRV_STATUS: 0x");
  Serial.println(m1_drv_status, HEX);
  
  // Set start/stop velocities for M1
  driver.M1_VSTART(500);
  driver.M1_VSTOP(700);
  
  // Set acceleration profile for M1
  driver.M1_A1(3000);
  driver.M1_AMAX(50000);
  
  // Set deceleration profile for M1
  driver.M1_D1(3000);
  driver.M1_DMAX(50000);
  
  // Set velocity profile for M1
  driver.M1_V1(10000);
  driver.M1_VMAX(50000);
  
  // Set M1 to hold current position at startup
  int32_t m1_current_pos = driver.M1_XACTUAL();
  driver.M1_XTARGET(m1_current_pos);
  
  Serial.println("✓ Motor M1 configured");
  
  // ========== Configure M1 Encoder ==========
  // Same configuration as M0
  driver.M1_ENCMODE(0x00000900);
  driver.M1_ENC_CONST(0x000C1F40);  // 12.8 fixed-point
  driver.M1_X_ENC(0);
  
  Serial.println("\n=== Configuring Second TMC5272 Driver (M2, M3) ===\n");
  delay(100);
  
  // Setup CS_PIN2 for second driver
  pinMode(CS_PIN2, OUTPUT);
  digitalWrite(CS_PIN2, HIGH);
  Serial.println("CS_PIN2 set HIGH");
  
  // EN_PIN is shared with driver 1 (already configured in setup)
  Serial.println("EN_PIN shared with driver 1 ✓");
  delay(100);
  
  delay(500);  // Longer delay for chip power-up
  Serial.println("Waiting 500ms for second chip power-up and initialization...");
  
  // Initialize driver2
  Serial.println("\n--- Initializing Second TMC5272 Driver ---");
  driver2.begin();
  
  // Disable PWM mode - use pure SpreadCycle for testing
  uint32_t gconf_init2 = 0x00020002;  // Both motors: SpreadCycle + multistep filter
  driver2.GCONF(gconf_init2);
  delay(50);
  
  Serial.print("GCONF2 written: 0x"); Serial.println(gconf_init2, HEX);
  uint32_t gconf_read2 = driver2.GCONF();
  Serial.print("GCONF2 readback: 0x"); Serial.println(gconf_read2, HEX);
  driver2.defaults();
  Serial.println("Driver2 initialized");
  delay(100);
  
  uint32_t ioin2 = driver2.IOIN();
  uint8_t status2 = driver2.status_response;
  Serial.print("IOIN2 = 0x"); Serial.print(ioin2, HEX);
  Serial.print(" | Status byte = 0x"); Serial.println(status2, HEX);
  
  // Check VERSION for driver2
  Serial.println("Reading VERSION field...");
  uint8_t ver2 = driver2.version();
  uint8_t status_ver2 = driver2.status_response;
  Serial.print("VERSION2: 0x"); Serial.print(ver2, HEX);
  Serial.print(" | Status: 0x"); Serial.println(status_ver2, HEX);
  if (ver2 == 0) {
    Serial.println(" ⚠️  WARNING: VERSION2 is 0!");
    Serial.println("   NO SPI communication!");
  } else {
    Serial.print(" ✓ Second chip detected! TMC5272 version: ");
    Serial.println(ver2);
  }
  
  // Read and clear GSTAT for driver2
  uint32_t gstat2 = driver2.GSTAT();
  if (gstat2 != 0) {
    Serial.print("GSTAT2: 0x"); Serial.print(gstat2, HEX);
    Serial.println(" - Clearing error flags...");
    driver2.GSTAT(0x0000003F);  // Clear all GSTAT bits
    delay(10);
  }
  
  // Configure essential global registers for driver2
  driver2.GCONF(0x00000000);           // Global configuration
  driver2.NODECONF(0x00000000);        // Node configuration (UART)
  driver2.DRV_CONF(0x0000CFCF);        // Driver config: equal drive strength for both motors
  driver2.GLOBALSCALER(0x00000000);    // No global scaling
  driver2.RAMPMODE(0x00000000);        // Position mode
  
  Serial.println("\n=== Register Setup Complete (Driver2) ===");
  Serial.print("DRV_CONF2 readback: 0x"); Serial.println(driver2.DRV_CONF(), HEX);
  
  // Configure GLOBALSCALER for full current
  driver2.GLOBALSCALER(0x00000000);  // 0 = Full scale (256/256)
  
  // ========== Configure Motor 2 (M2) ==========
  Serial.println("\n=== Configuring Motor M2 (Driver2.M0) ===");
  
  // Configure Motor 2 current settings (reduced for less heat)
  driver2.M0_IHOLD_IRUN(0x000A1008);
  
  // Enable Motor 2 driver with CHOPCONF (TOFF=3, MRES=0 for 256 microsteps)
  driver2.M0_CHOPCONF(0x00000003);
  Serial.println("\n*** DRIVER2 M0 ENABLED! ***");
  
  // Set start/stop velocities for M2
  driver2.M0_VSTART(500);
  driver2.M0_VSTOP(700);
  Serial.println("M2: VSTART=500, VSTOP=700");
  
  // Set acceleration profile for M2
  driver2.M0_A1(3000);
  driver2.M0_AMAX(50000);
  
  // Set deceleration profile for M2
  driver2.M0_D1(3000);
  driver2.M0_DMAX(50000);
  
  // Set velocity profile for M2
  driver2.M0_V1(10000);
  driver2.M0_VMAX(50000);
  
  // Verify motor M2 is holding position at startup
  int32_t m2_current_pos = driver2.M0_XACTUAL();
  driver2.M0_XTARGET(m2_current_pos);
  
  Serial.println("✓ Motor M2 configured");
  
  // Configure M2 Encoder
  driver2.M0_ENCMODE(0x00000900);
  driver2.M0_ENC_CONST(0x000C1F40);  // 12.8 fixed-point
  driver2.M0_X_ENC(0);
  
  // ========== Configure Motor 3 (M3) ==========
  Serial.println("\n=== Configuring Motor M3 (Driver2.M1) ===");
  
  // Configure Motor 3 current settings
  driver2.M1_IHOLD_IRUN(0x000A1008);
  delay(10);
  
  // Enable Motor 3 driver with CHOPCONF
  driver2.M1_CHOPCONF(0x00000003);
  delay(50);
  
  // Write IHOLD_IRUN again after CHOPCONF
  driver2.M1_IHOLD_IRUN(0x000A1008);
  delay(10);
  driver2.M1_IHOLD_IRUN(0x000A1008);
  delay(50);
  
  Serial.print("M3_CHOPCONF written: 0x3, readback: 0x");
  Serial.println(driver2.M1_CHOPCONF(), HEX);
  Serial.print("M3_IHOLD_IRUN written: 0xA1008, readback: 0x");
  Serial.println(driver2.M1_IHOLD_IRUN(), HEX);
  
  // Check DRV_STATUS for M3
  uint32_t m3_drv_status = driver2.M1_DRV_STATUS();
  Serial.print("M3_DRV_STATUS: 0x");
  Serial.println(m3_drv_status, HEX);
  
  // Set start/stop velocities for M3
  driver2.M1_VSTART(500);
  driver2.M1_VSTOP(700);
  
  // Set acceleration profile for M3
  driver2.M1_A1(3000);
  driver2.M1_AMAX(50000);
  
  // Set deceleration profile for M3
  driver2.M1_D1(3000);
  driver2.M1_DMAX(50000);
  
  // Set velocity profile for M3
  driver2.M1_V1(10000);
  driver2.M1_VMAX(50000);
  
  // Set M3 to hold current position at startup
  int32_t m3_current_pos = driver2.M1_XACTUAL();
  driver2.M1_XTARGET(m3_current_pos);
  
  Serial.println("✓ Motor M3 configured");
  
  // Configure M3 Encoder
  driver2.M1_ENCMODE(0x00000900);
  driver2.M1_ENC_CONST(0x000C1F40);  // 12.8 fixed-point
  driver2.M1_X_ENC(0);
  
  Serial.println("\n✓ All 4 motors ready - waiting for ROS2 commands\n");

  // Create mutex for pressure data
  pressure_mutex = xSemaphoreCreateMutex();
  
  // Create pressure reading task on Core 0 (Core 1 runs Arduino loop)
  xTaskCreatePinnedToCore(
    pressureTask,      // Task function
    "PressureTask",    // Task name
    4096,              // Stack size (bytes)
    NULL,              // Task parameter
    1,                 // Priority (1 = low)
    NULL,              // Task handle
    0                  // Core 0
  );
  Serial.println("✓ Pressure reading task started on Core 0\n");

  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "tmc5272_motor_node", "", &support));
  
  // Create motor position publisher
  RCCHECK(rclc_publisher_init_default(
    &motor_pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_pos"));
  
  // Create encoder position publisher
  RCCHECK(rclc_publisher_init_default(
    &encoder_pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/encoder_pos"));
  
  // Create pressure publisher
  RCCHECK(rclc_publisher_init_default(
    &pressure_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "/pressure"));
  
  // Create pump status publisher
  RCCHECK(rclc_publisher_init_default(
    &pump_status_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/pump/status"));
  
  // Create pump home subscriber
  RCCHECK(rclc_subscription_init_default(
    &pump_home_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
    "/pump/home"));
  
  // Create motor command subscriber
  RCCHECK(rclc_subscription_init_default(
    &motor_cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_cmd"));
  
  // Create timer (motor positions publish every 10ms = 100Hz)
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create pressure timer (publish every 100ms = 10Hz)
  const unsigned int pressure_timer_timeout = 100;
  RCCHECK(rclc_timer_init_default(
    &pressure_timer,
    &support,
    RCL_MS_TO_NS(pressure_timer_timeout),
    pressure_timer_callback));
  
  // Create executor (2 timers + 2 subscribers: 1 home + 1 motor command)
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_timer(&executor, &pressure_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &pump_home_subscriber, &pump_home_msg, &pump_home_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_cmd_subscriber, &motor_cmd_msg, &motor_cmd_callback, ON_NEW_DATA));
  
  // Initialize array messages
  motor_pos_msg.data.data = motor_pos_data;
  motor_pos_msg.data.size = NUM_SYRINGES;
  motor_pos_msg.data.capacity = NUM_SYRINGES;
  
  encoder_pos_msg.data.data = encoder_pos_data;
  encoder_pos_msg.data.size = NUM_SYRINGES;
  encoder_pos_msg.data.capacity = NUM_SYRINGES;
  
  pressure_msg.data.data = pressure_data;
  pressure_msg.data.size = NUM_SYRINGES;
  pressure_msg.data.capacity = NUM_SYRINGES;
  
  motor_cmd_msg.data.data = motor_cmd_data;
  motor_cmd_msg.data.size = NUM_SYRINGES;
  motor_cmd_msg.data.capacity = NUM_SYRINGES;
  
  for (uint8_t i = 0; i < NUM_SYRINGES; i++) {
    motor_pos_data[i] = 0;
    encoder_pos_data[i] = 0;
    pressure_data[i] = 0.0f;
    motor_cmd_data[i] = 0;
  }
  pump_status_msg.data = false;
  
  Serial.println("✓ micro-ROS ready!\n");
  Serial.println("Hardware interface listening on:");
  Serial.println("  Subscribe: /pump/home (Bool) - Trigger homing");
  Serial.println("  Subscribe: /motor_cmd (Int32MultiArray) - Motor commands for all syringes");
  Serial.println("  Publish:   /motor_pos (Int32MultiArray) - Motor positions @ 100Hz");
  Serial.println("  Publish:   /encoder_pos (Int32MultiArray) - Encoder positions @ 100Hz");
  Serial.println("  Publish:   /pressure (Float32MultiArray) - Pressure readings @ 10Hz");
  Serial.println("  Publish:   /pump/status (Bool) - Homing status\n");
}

void loop() {
  // Execute homing if requested via ROS2
  if (homing_requested) {
    homing_requested = false;
    pump_status_msg.data = false;  // Homing in progress
    RCSOFTCHECK(rcl_publish(&pump_status_publisher, &pump_status_msg, NULL));
    performHoming();
  }
  
  // Spin micro-ROS executor - handle all subscriptions and publish timer
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}


#endif  // End disabled section