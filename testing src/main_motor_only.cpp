

#include <Arduino.h>
#include <SPI.h>
#include <TMCStepper.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pin definitions
constexpr uint8_t CS_PIN = 7;
constexpr uint8_t CS_PIN2 = 18;
constexpr uint8_t EN_PIN = 3;
constexpr float RSENSE = 0.075f;
const uint8_t NUM_MOTORS = 3;

// Create TMC5272 drivers
TMC5272Stepper driver1(CS_PIN, RSENSE);
TMC5272Stepper driver2(CS_PIN2, RSENSE);

// Micro-ROS variables
rcl_publisher_t motor_pos_publisher;
rcl_subscription_t motor_cmd_subscriber;
std_msgs__msg__Int32MultiArray motor_pos_msg;
std_msgs__msg__Int32MultiArray motor_cmd_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Storage for array data
int32_t motor_pos_data[NUM_MOTORS];
int32_t motor_cmd_data[NUM_MOTORS];

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){while(1){delay(100);}}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    // Read motor positions only
    motor_pos_data[0] = driver1.M0_XACTUAL();
    motor_pos_data[1] = driver1.M1_XACTUAL();
    motor_pos_data[2] = driver2.M0_XACTUAL();
    
    // Publish
    RCSOFTCHECK(rcl_publish(&motor_pos_publisher, &motor_pos_msg, NULL));
  }
}

void motor_cmd_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  driver1.M0_XTARGET(msg->data.data[0]);
  driver1.M1_XTARGET(msg->data.data[1]);
  driver2.M0_XTARGET(msg->data.data[2]);
}

void setup() {
  Serial.begin(921600);
  delay(100);
  
  // Initialize SPI
  SPI.begin();
  driver1.setSPISpeed(8000000);
  
  // Setup pins
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);
  pinMode(CS_PIN2, OUTPUT);
  digitalWrite(CS_PIN2, HIGH);
  pinMode(EN_PIN, OUTPUT);
  digitalWrite(EN_PIN, HIGH);
  delay(100);
  
  // Initialize drivers
  driver1.begin();
  driver1.GCONF(0x00020002);
  driver1.defaults();
  driver1.GLOBALSCALER(0x00000000);
  driver1.RAMPMODE(0);
  
  driver2.begin();
  driver2.GCONF(0x00020002);
  driver2.defaults();
  driver2.GLOBALSCALER(0x00000000);
  driver2.RAMPMODE(0);
  
  // Configure M0
  driver1.M0_IHOLD_IRUN(0x000A1008);
  driver1.M0_CHOPCONF(0x00000003);
  driver1.M0_VSTART(500);
  driver1.M0_VSTOP(700);
  driver1.M0_A1(3000);
  driver1.M0_AMAX(50000);
  driver1.M0_D1(3000);
  driver1.M0_DMAX(50000);
  driver1.M0_V1(10000);
  driver1.M0_VMAX(50000);
  driver1.M0_XTARGET(driver1.M0_XACTUAL());
  
  // Configure M1
  driver1.M1_IHOLD_IRUN(0x000A1008);
  driver1.M1_CHOPCONF(0x00000003);
  driver1.M1_VSTART(500);
  driver1.M1_VSTOP(700);
  driver1.M1_A1(3000);
  driver1.M1_AMAX(50000);
  driver1.M1_D1(3000);
  driver1.M1_DMAX(50000);
  driver1.M1_V1(10000);
  driver1.M1_VMAX(50000);
  driver1.M1_XTARGET(driver1.M1_XACTUAL());
  
  // Configure M2
  driver2.M0_IHOLD_IRUN(0x000A1008);
  driver2.M0_CHOPCONF(0x00000003);
  driver2.M0_VSTART(500);
  driver2.M0_VSTOP(700);
  driver2.M0_A1(3000);
  driver2.M0_AMAX(50000);
  driver2.M0_D1(3000);
  driver2.M0_DMAX(50000);
  driver2.M0_V1(10000);
  driver2.M0_VMAX(50000);
  driver2.M0_XTARGET(driver2.M0_XACTUAL());

  // Initialize micro-ROS
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "motor_node", "", &support));
  
  RCCHECK(rclc_publisher_init_default(
    &motor_pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_pos"));
  
  RCCHECK(rclc_subscription_init_default(
    &motor_cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_cmd"));
  
  // Timer for 100Hz (10ms period)
  const unsigned int timer_timeout = 5;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));
  
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_cmd_subscriber, &motor_cmd_msg, &motor_cmd_callback, ON_NEW_DATA));
  
  motor_pos_msg.data.data = motor_pos_data;
  motor_pos_msg.data.size = NUM_MOTORS;
  motor_pos_msg.data.capacity = NUM_MOTORS;
  
  motor_cmd_msg.data.data = motor_cmd_data;
  motor_cmd_msg.data.size = NUM_MOTORS;
  motor_cmd_msg.data.capacity = NUM_MOTORS;
  
  for (uint8_t i = 0; i < NUM_MOTORS; i++) {
    motor_pos_data[i] = 0;
    motor_cmd_data[i] = 0;
  }
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
