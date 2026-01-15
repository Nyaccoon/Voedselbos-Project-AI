#include <micro_ros_arduino.h>
#include <ESP32Servo.h>
#include <stdio.h>
#include <string.h> // Needed for string comparison

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <rcutils/logging_macros.h>

#include <std_msgs/msg/string.h> 

// --- SERVO CONFIGURATION ---
Servo s;
Servo s2;
const int SERVO_PIN = 27;
const int SERVO_PIN2 = 15;

// --- MICRO-ROS CONFIGURATION ---
rcl_subscription_t subscriber;
std_msgs__msg__String msg; // Using String message
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 13

// --- TIMING VARIABLES ---
bool is_open = false;
unsigned long open_start_time = 0;
const unsigned long OPEN_DURATION = 15000; // 15 seconds in milliseconds

// --- MACROS ---
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

struct Message {
  String id;
  String receiver;
  String type_of_message;
  String message_content;
};

void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// --- SERVO FUNCTIONS ---
void open_servos() {
    s.write(25);
    s2.write(110);
}

void close_servos() {
    s.write(110);
    s2.write(25);
}

// --- CALLBACK ---
void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__String * received_msg = (const std_msgs__msg__String *)msgin;
  String temp = parse_message(received_msg->data.data);
  if (received_msg->data.data != NULL && temp == "open") {
      if (!is_open) {
          // Trigger the open sequence
          open_servos();
          is_open = true;
          open_start_time = millis(); 
      }
  }
}

void setup() {
  set_microros_transports();
  
  // Servo Setup
  s.setPeriodHertz(100);
  s.attach(SERVO_PIN, 500, 2400);
  s2.setPeriodHertz(100);
  s2.attach(SERVO_PIN2, 500, 2400);
  
  // Start in closed position
  close_servos();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Connection Check
  rcl_ret_t ping_result = rmw_uros_ping_agent(100, 1);
  while (ping_result != RCL_RET_OK) {
     
      digitalWrite(LED_PIN, !digitalRead(LED_PIN)); 
      delay(250);
      ping_result = rmw_uros_ping_agent(100, 1);
  }
  digitalWrite(LED_PIN, HIGH); // Solid ON means connected

  allocator = rcl_get_default_allocator();

  // Create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Create node "bakSysteem"
  RCCHECK(rclc_node_init_default(&node, "bakSysteem", "", &support));

  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
    "S/topic")); 

  msg.data.capacity = 255;
  msg.data.data = (char*) malloc(msg.data.capacity * sizeof(char));
  msg.data.size = 0;

  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  if(rmw_uros_ping_agent(100, 1) == RCL_RET_OK) {
     RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
  }

  if (is_open) {
      if (millis() - open_start_time >= OPEN_DURATION) {
          close_servos();
          is_open = false;
      }
  }
}

Message parseMessage(const String& raw) {
  Message m;
  int p1 = raw.indexOf(';');
  if (p1 < 0) {
    RCUTILS_LOG_INFO("bakSysteem", "Invalid message (missing ID): %s", raw.c_str());
    Serial.println();
    return m;
  }

  int p2 = raw.indexOf(';', p1 + 1);
  if (p2 < 0) {
    RCUTILS_LOG_INFO("bakSysteem", "Invalid message (missing Receiver): %s", raw.c_str());
    return m;
  }

  int p3 = raw.indexOf(';', p2 + 1);
  if (p3 < 0) {
    RCUTILS_LOG_INFO("bakSysteem", "Invalid message (missing MessageType): %s", raw.c_str());
    return m;
  }

  m.id              = raw.substring(0, p1);
  m.receiver        = raw.substring(p1 + 1, p2);
  m.type_of_message = raw.substring(p2 + 1, p3);
  m.message_content = raw.substring(p3 + 1);

  return m;
}

String parse_message(String message){
  Message m = parseMessage(message);
  if(m.type_of_message == "Command"){
    if(m.message_content == "open"){
      return m.message_content;
      RCUTILS_LOG_INFO("bakSysteem", "Message Parsed returning", m.message_content.c_str());
      }
  }
  return "no";
}