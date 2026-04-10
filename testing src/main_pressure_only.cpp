#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// I2C pins for pressure sensor
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;

// Pressure sensor definitions
const uint8_t PRESSURE_SENSOR_ADDR = 0x28;
const double outputmax = 15099494.0;  // output at maximum pressure [counts]
const double outputmin = 1677722.0;   // output at minimum pressure [counts]
const double pmax = 60.0;             // maximum value of pressure range [psi]
const double pmin = 0.0;              // minimum value of pressure range [psi]

// Pressure sensor variables
uint8_t pressure_data[7];                      // holds output data
uint8_t pressure_cmd[3] = {0xAA, 0x00, 0x00};  // command to be sent
double press_counts = 0.0;                     // digital pressure reading [counts]
double temp_counts = 0.0;                      // digital temperature reading [counts]
double pressure_value = 0.0;                   // pressure reading [bar]
double temperature_value = 0.0;                // temperature reading in deg C
double percentage = 0.0;                       // percentage of full scale pressure

// Micro-ROS variables
rcl_publisher_t pressure_publisher;
std_msgs__msg__Float32 pressure_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop() {
  while(1) {
    delay(100);
  }
}

// Read pressure sensor value (full datasheet implementation)
float readPressure() {
  Wire.beginTransmission(PRESSURE_SENSOR_ADDR);
  int stat = Wire.write(pressure_cmd, 3);  // write command to the sensor
  stat |= Wire.endTransmission();
  
  // No delay - I2C at 400kHz is fast enough (~200μs for 7 bytes)
  
  Wire.requestFrom(PRESSURE_SENSOR_ADDR, 7);  // read back Sensor data 7 bytes
  int i = 0;
  for (i = 0; i < 7; i++) {
    pressure_data[i] = Wire.read();
  }
  
  // Calculate digital pressure counts (24-bit)
  press_counts = (double)((int32_t)pressure_data[3] + 
                          (int32_t)pressure_data[2] * (int32_t)256 + 
                          (int32_t)pressure_data[1] * (int32_t)65536);
  
  // Calculate digital temperature counts (24-bit)
  temp_counts = (double)((int32_t)pressure_data[6] + 
                         (int32_t)pressure_data[5] * (int32_t)256 + 
                         (int32_t)pressure_data[4] * (int32_t)65536);
  
  // Calculate temperature in deg C
  temperature_value = (temp_counts * 200.0 / 16777215.0) - 50.0;
  
  // Calculate pressure as percentage of full scale
  percentage = (press_counts / 16777215.0) * 100.0;
  
  // Calculate pressure value according to equation 2 of datasheet
  double pressure_psi = ((press_counts - outputmin) * (pmax - pmin)) / 
                        (outputmax - outputmin) + pmin;
  
  // Convert from psi to kPa
  pressure_value = pressure_psi * 6.89476;
  
  return (float)pressure_value;
}

// Timer callback - publish pressure at 100Hz
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pressure_msg.data = readPressure();
    RCSOFTCHECK(rcl_publish(&pressure_publisher, &pressure_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=== Pressure Sensor Only Node ===");
  
  // Initialize I2C for pressure sensor
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);  // 400kHz I2C
  delay(10);
  
  Serial.println("✓ I2C initialized for pressure sensor");
  
  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pressure_sensor_node", "", &support));
  
  // Create pressure publisher
  RCCHECK(rclc_publisher_init_default(
    &pressure_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/pressure"));
  
  // Create timer for 100Hz (10ms period)
  const unsigned int timer_timeout = 10;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // Create executor (1 timer only)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  Serial.println("✓ Pressure sensor node ready - publishing to /pressure at 100Hz");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
