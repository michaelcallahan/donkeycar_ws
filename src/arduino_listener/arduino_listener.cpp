#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/int32_multi_array.h>

#include <Wire.h>

// Initialize microros elements
rcl_publisher_t publisher;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

// Buffers to store the received values
uint8_t onL = 0;
uint8_t onH = 0;
uint8_t offL = 0;
uint8_t offH = 0;

// ROS2 publisher and message
std_msgs__msg__Int32MultiArray msg;

int32_t data[2];

// Buffers to store the received values
volatile int steer_angle = 0;
volatile int motor_speed = 0;
volatile bool new_data = false;

// Define I2C address for PCA9685
#define I2C_ADDRESS 0x40

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void receiveEvent(int howMany) {
  if (howMany == 4) {
    uint8_t steer_high = Wire.read();
    uint8_t steer_low = Wire.read();
    uint8_t speed_high = Wire.read();
    uint8_t speed_low = Wire.read();

    steer_angle = (steer_high << 8) | steer_low;
    motor_speed = (speed_high << 8) | speed_low;

    new_data = true; // Set flag to indicate new data has been received
  }
}

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  }
}

void setup() {
  set_microros_transports();
  
  // Initialize I2C communication as a slave device
  Wire.begin(I2C_ADDRESS);

  // Register the receive event handler
  Wire.onReceive(receiveEvent);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), "steering_motor_data"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  //RCCHECK(rclc_executor_add_timer(&executor, &timer));
  
  // Initialize the message
  msg.data.data = data;
  msg.data.size = 2;
  msg.data.capacity = 2;
}

void loop() {
  // Check if new I2C data has been received
  if (new_data) {
    // Reset the flag
    new_data = false;

    // Print received values to the serial monitor for debugging
    Serial.print("Received Steer Angle: ");
    Serial.print(steer_angle);
    Serial.print(" | Received Motor Speed: ");
    Serial.println(motor_speed);

    // Prepare and publish the ROS2 message
    data[0] = steer_angle;
    data[1] = motor_speed;
    msg.data.data = data;
    msg.data.size = 2;
    msg.data.capacity = 2;
    rcl_publish(&publisher, &msg, NULL);
  }

  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
