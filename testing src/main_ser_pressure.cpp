#include <Arduino.h>
#include <Wire.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// I2C pins
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;

// TCA9548A multiplexer
constexpr uint8_t TCAADDR = 0x70;

// Active MUX channels (2–7, channels 0 and 1 unused)
constexpr uint8_t MUX_CHANNELS[]  = {2, 3, 4, 5, 6, 7};
constexpr uint8_t NUM_SENSORS     = sizeof(MUX_CHANNELS);

// Pressure sensor I2C address and calibration constants
constexpr uint8_t PRESSURE_SENSOR_ADDR = 0x28;
const double outputmax = 15099494.0;  // counts at maximum pressure
const double outputmin = 1677722.0;   // counts at minimum pressure
const double pmax = 60.0;             // maximum pressure [psi]
const double pmin = 0.0;              // minimum pressure [psi]

// Sensor raw buffers (reused per read)
uint8_t pressure_data[7];
uint8_t pressure_cmd[3] = {0xAA, 0x00, 0x00};

// Storage for the multi-array message data
float pressure_values[NUM_SENSORS];

// Micro-ROS objects
rcl_publisher_t pressure_publisher;
std_msgs__msg__Float32MultiArray pressure_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define RCCHECK(fn)     { rcl_ret_t temp_rc = fn; if ((temp_rc != RCL_RET_OK)) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; (void)temp_rc; }

void error_loop() {
  while (1) {
    delay(100);
  }
}

// Select a channel on the TCA9548A
void tcaSelect(uint8_t channel) {
  if (channel > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

// Read pressure in kPa from the sensor on the currently selected MUX channel
float readPressure() {
  Wire.beginTransmission(PRESSURE_SENSOR_ADDR);
  Wire.write(pressure_cmd, 3);
  Wire.endTransmission();

  Wire.requestFrom(PRESSURE_SENSOR_ADDR, (uint8_t)7);
  for (int i = 0; i < 7; i++) {
    pressure_data[i] = Wire.read();
  }

  double press_counts = (double)((int32_t)pressure_data[3] +
                                 (int32_t)pressure_data[2] * 256 +
                                 (int32_t)pressure_data[1] * 65536);

  double pressure_psi = ((press_counts - outputmin) * (pmax - pmin)) /
                        (outputmax - outputmin) + pmin;
  return (float)(pressure_psi * 6.89476);
}

// Timer callback – read all 6 sensors and publish at 100 Hz
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    for (uint8_t i = 0; i < NUM_SENSORS; i++) {
      tcaSelect(MUX_CHANNELS[i]);
      pressure_values[i] = readPressure();
    }
    RCSOFTCHECK(rcl_publish(&pressure_publisher, &pressure_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  // Initialise I2C
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);
  delay(10);

  // Initialise micro-ROS transport
  set_microros_serial_transports(Serial);
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "pressure_test_node", "", &support));

  // Publisher on pressure_ser
  RCCHECK(rclc_publisher_init_default(
    &pressure_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
    "pressure_ser"));

  // Wire the message data pointer to the static array (no heap allocation)
  pressure_msg.data.data     = pressure_values;
  pressure_msg.data.size     = NUM_SENSORS;
  pressure_msg.data.capacity = NUM_SENSORS;
  pressure_msg.layout.dim.data     = NULL;
  pressure_msg.layout.dim.size     = 0;
  pressure_msg.layout.dim.capacity = 0;
  pressure_msg.layout.data_offset  = 0;

  // 100 Hz timer (10 ms period)
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(10),
    timer_callback));

  // Executor with 1 handle (the timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
