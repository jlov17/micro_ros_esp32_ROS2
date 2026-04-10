#include <Arduino.h>
#include <HX711.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// HX711 Force Sensor pins
#define HX711_DOUT_PIN  2
#define HX711_SCK_PIN   10
#define HX711_CALIBRATION_FACTOR  52700.0

HX711 forceSensor;
bool force_tared = false;

// Micro-ROS variables
rcl_publisher_t force_publisher;
std_msgs__msg__Float32 force_msg;
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

float readForceN() {
  if (!force_tared) {
    return 0.0;
  }
  float reading = forceSensor.get_units(1);
  return reading;
}

// Timer callback - publish force at 100Hz
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    force_msg.data = readForceN();
    RCSOFTCHECK(rcl_publish(&force_publisher, &force_msg, NULL));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=== Force Sensor Only Node ===");
  
  // Initialize HX711 force sensor
  forceSensor.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  forceSensor.set_scale(HX711_CALIBRATION_FACTOR);
  
  Serial.println("Taring force sensor in 3 seconds...");
  delay(3000);
  forceSensor.tare();
  force_tared = true;
  
  Serial.println("✓ Force sensor initialized and tared");
  
  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "force_sensor_node", "", &support));
  
  // Create force publisher
  RCCHECK(rclc_publisher_init_default(
    &force_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/force"));
  
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
  
  Serial.println("✓ Force sensor node ready - publishing to /force at 100Hz");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
