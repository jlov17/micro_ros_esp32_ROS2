#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <HX711.h>
#include <TMCStepper.h>
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/float32.h>
#include <std_msgs/msg/int32_multi_array.h>

#if !defined(MICRO_ROS_TRANSPORT_ARDUINO_SERIAL)
#error This example is only available for Arduino framework with serial transport.
#endif

// Pin definitions
constexpr uint8_t CS_PIN = 7;
constexpr uint8_t CS_PIN2 = 18;
constexpr uint8_t EN_PIN = 3;
constexpr float RSENSE = 0.075f;

// I2C pins for pressure sensor
constexpr uint8_t I2C_SDA = 8;
constexpr uint8_t I2C_SCL = 9;

// Pressure sensor definitions
const uint8_t PRESSURE_SENSOR_ADDR = 0x28;
const double outputmax = 15099494.0;
const double outputmin = 1677722.0;
const double pmax = 60.0;
const double pmin = 0.0;

// HX711 Force Sensor
#define HX711_DOUT_PIN  2
#define HX711_SCK_PIN   10
#define HX711_CALIBRATION_FACTOR  52700.0

HX711 forceSensor;
bool force_tared = false;

// Number of motors
const uint8_t NUM_MOTORS = 3;

// Create TMC5272 drivers
TMC5272Stepper driver1(CS_PIN, RSENSE);
TMC5272Stepper driver2(CS_PIN2, RSENSE);

// Pressure sensor variables
uint8_t pressure_data[7];
uint8_t pressure_cmd[3] = {0xAA, 0x00, 0x00};
double press_counts = 0.0;
double temp_counts = 0.0;
double pressure_value = 0.0;
double temperature_value = 0.0;
double percentage = 0.0;

// Micro-ROS variables
rcl_publisher_t motor_pos_publisher;
rcl_publisher_t force_publisher;
rcl_publisher_t pressure_publisher;
rcl_subscription_t motor_cmd_subscriber;
std_msgs__msg__Int32MultiArray motor_pos_msg;
std_msgs__msg__Int32MultiArray motor_cmd_msg;
std_msgs__msg__Float32 force_msg;
std_msgs__msg__Float32 pressure_msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t motor_pos_timer;
rcl_timer_t force_timer;
rcl_timer_t pressure_timer;

// Storage for array data
int32_t motor_pos_data[NUM_MOTORS];
int32_t motor_cmd_data[NUM_MOTORS];

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

float readPressure() {
  Wire.beginTransmission(PRESSURE_SENSOR_ADDR);
  int stat = Wire.write(pressure_cmd, 3);
  stat |= Wire.endTransmission();
  
  Wire.requestFrom(PRESSURE_SENSOR_ADDR, 7);
  int i = 0;
  for (i = 0; i < 7; i++) {
    pressure_data[i] = Wire.read();
  }
  
  press_counts = (double)((int32_t)pressure_data[3] + 
                          (int32_t)pressure_data[2] * (int32_t)256 + 
                          (int32_t)pressure_data[1] * (int32_t)65536);
  
  temp_counts = (double)((int32_t)pressure_data[6] + 
                         (int32_t)pressure_data[5] * (int32_t)256 + 
                         (int32_t)pressure_data[4] * (int32_t)65536);
  
  temperature_value = (temp_counts * 200.0 / 16777215.0) - 50.0;
  percentage = (press_counts / 16777215.0) * 100.0;
  
  double pressure_psi = ((press_counts - outputmin) * (pmax - pmin)) / 
                        (outputmax - outputmin) + pmin;
  
  pressure_value = pressure_psi * 6.89476;
  
  return (float)pressure_value;
}

// Motor position timer callback
void motor_pos_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    motor_pos_data[0] = driver1.M0_XACTUAL();
    motor_pos_data[1] = driver1.M1_XACTUAL();
    motor_pos_data[2] = driver2.M0_XACTUAL();
    
    RCSOFTCHECK(rcl_publish(&motor_pos_publisher, &motor_pos_msg, NULL));
  }
}

// Force sensor timer callback
void force_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    force_msg.data = readForceN();
    RCSOFTCHECK(rcl_publish(&force_publisher, &force_msg, NULL));
  }
}

// Pressure sensor timer callback
void pressure_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    pressure_msg.data = readPressure();
    RCSOFTCHECK(rcl_publish(&pressure_publisher, &pressure_msg, NULL));
  }
}

// Callback for motor command subscriber
void motor_cmd_callback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * msg = (const std_msgs__msg__Int32MultiArray *)msgin;
  
  driver1.M0_XTARGET(msg->data.data[0]);
  driver1.M1_XTARGET(msg->data.data[1]);
  driver2.M0_XTARGET(msg->data.data[2]);
}

void setup() {
  Serial.begin(115200);
  delay(100);
  
  Serial.println("\n=== Simple Test Node: Motors + Sensors ===");
  
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

  Serial.println("✓ All 3 motors configured");
  
  // Initialize I2C for pressure sensor
  Wire.setPins(I2C_SDA, I2C_SCL);
  Wire.begin();
  Wire.setClock(400000);
  Serial.println("✓ I2C initialized");
  
  // Initialize HX711 force sensor
  forceSensor.begin(HX711_DOUT_PIN, HX711_SCK_PIN);
  forceSensor.set_scale(HX711_CALIBRATION_FACTOR);
  Serial.println("Taring force sensor in 3 seconds...");
  delay(3000);
  forceSensor.tare();
  force_tared = true;
  Serial.println("✓ Force sensor tared");

  // Initialize micro-ROS
  Serial.println("Initializing micro-ROS...");
  set_microros_serial_transports(Serial);
  delay(2000);
  
  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, "simple_test_node", "", &support));
  
  // Create publishers
  RCCHECK(rclc_publisher_init_default(
    &motor_pos_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_pos"));
  
  RCCHECK(rclc_publisher_init_default(
    &force_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/force"));
  
  RCCHECK(rclc_publisher_init_default(
    &pressure_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
    "/pressure"));
  
  // Create motor command subscriber
  RCCHECK(rclc_subscription_init_default(
    &motor_cmd_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/motor_cmd"));
  
  // Create timers - separate for each publisher
  const unsigned int motor_timer_timeout = 10;  // 100Hz for motors
  const unsigned int force_timer_timeout = 13;  // ~80Hz for force
  const unsigned int pressure_timer_timeout = 10;  // 100Hz for pressure
  
  RCCHECK(rclc_timer_init_default(
    &motor_pos_timer,
    &support,
    RCL_MS_TO_NS(motor_timer_timeout),
    motor_pos_timer_callback));
  
  RCCHECK(rclc_timer_init_default(
    &force_timer,
    &support,
    RCL_MS_TO_NS(force_timer_timeout),
    force_timer_callback));
  
  RCCHECK(rclc_timer_init_default(
    &pressure_timer,
    &support,
    RCL_MS_TO_NS(pressure_timer_timeout),
    pressure_timer_callback));

  // Create executor (3 timers + 1 subscriber)
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &motor_pos_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &force_timer));
  RCCHECK(rclc_executor_add_timer(&executor, &pressure_timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &motor_cmd_subscriber, &motor_cmd_msg, &motor_cmd_callback, ON_NEW_DATA));
  
  // Initialize array messages
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
  
  Serial.println("✓ Simple test node ready!");
  Serial.println("Publishing: /motor_pos (100Hz), /force (~80Hz), /pressure (100Hz)");
  Serial.println("Subscribing: /motor_cmd");
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)));
}
