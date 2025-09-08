#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/string.h>
#include <std_msgs/msg/int32_multi_array.h>

// ===== Motor pins =====
#define INA 26
#define INB 27
#define ENA 14

#define INC 32
#define IND 33
#define ENB 25

// ===== PWM channels =====
const int pwmChannelA = 6;
const int pwmChannelB = 7;

// ===== Servo =====
Servo servo1, servo2, servo3, servo4;

// ===== ROS2 objects =====
rcl_subscription_t cmdvel_sub;
geometry_msgs__msg__Twist cmdvel_msg;

rcl_subscription_t servo_sub;
std_msgs__msg__Int32MultiArray servo_msg;

rcl_publisher_t publisher;
std_msgs__msg__String status_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ===== State =====
float linear_x = 0;
float angular_z = 0;

// ===== Motor control =====
void setMotor(int pwmA, int dirA1, int dirA2, int pwmB, int dirB1, int dirB2) {
  ledcWrite(pwmChannelA, pwmA);
  ledcWrite(pwmChannelB, pwmB);
  digitalWrite(INA, dirA1);
  digitalWrite(INB, dirA2);
  digitalWrite(INC, dirB1);
  digitalWrite(IND, dirB2);
}

// ===== Callback for /cmd_vel =====
void cmdVelCallback(const void * msgin) {
  const geometry_msgs__msg__Twist * cmd = (const geometry_msgs__msg__Twist *)msgin;

  linear_x = cmd->linear.x;
  angular_z = cmd->angular.z;

  int speed = constrain(map(abs(linear_x * 100), 0, 100, 0, 255), 0, 255);

  if (linear_x > 0.05) {            // forward
    setMotor(speed, HIGH, LOW, speed, HIGH, LOW);
    Serial.println("FORWARD");
  } else if (linear_x < -0.05) {    // backward
    setMotor(speed, LOW, HIGH, speed, LOW, HIGH);
    Serial.println("BACKWARD");
  } else if (angular_z > 0.05) {    // turn left
    setMotor(speed, LOW, HIGH, speed, HIGH, LOW);
    Serial.println("TURN LEFT");
  } else if (angular_z < -0.05) {   // turn right
    setMotor(speed, HIGH, LOW, speed, LOW, HIGH);
    Serial.println("TURN RIGHT");
  } else {                          // stop safely
    setMotor(0, LOW, LOW, 0, LOW, LOW);
    Serial.println("STOP");
  }
}

// ===== Callback for /servo_cmd =====
void servoCallback(const void * msgin) {
  const std_msgs__msg__Int32MultiArray * servo_cmd = 
      (const std_msgs__msg__Int32MultiArray *)msgin;

  if (servo_cmd->data.size >= 4) {
    int s1 = constrain(servo_cmd->data.data[0], 0, 180);
    int s2 = constrain(servo_cmd->data.data[1], 0, 180);
    int s3 = constrain(servo_cmd->data.data[2], 0, 180);
    int s4 = constrain(servo_cmd->data.data[3], 0, 180);

    servo1.write(s1);
    servo2.write(s2);
    servo3.write(s3);
    servo4.write(s4);

    Serial.printf("Servo angles: %d %d %d %d\n", s1, s2, s3, s4);
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);

  // WiFi transport for Micro-ROS
  char ssid[] = "UET-Wifi-Office-Free 2.4Ghz";
  char pass[] = "";
  char agent_ip[] = "10.11.99.71";   // Adjust to your ROS2 agent IP
  set_microros_wifi_transports(ssid, pass, agent_ip, 8888);

  // Motor pins
  pinMode(INA, OUTPUT);
  pinMode(INB, OUTPUT);
  pinMode(INC, OUTPUT);
  pinMode(IND, OUTPUT);

  ledcSetup(pwmChannelA, 1000, 8);
  ledcSetup(pwmChannelB, 1000, 8);
  ledcAttachPin(ENA, pwmChannelA);
  ledcAttachPin(ENB, pwmChannelB);

  // Servos
  servo1.attach(15); servo1.write(90);
  servo2.attach(16); servo2.write(90);
  servo3.attach(17); servo3.write(90);
  servo4.attach(4);  servo4.write(90);

  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // ROS2 init
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    Serial.println("Micro-ROS init failed!");
    while (1);
  }

  // Initialize node
  if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) {
    Serial.println("Node init failed!");
    while (1);
  }

  // Initialize /cmd_vel subscriber
  if (rclc_subscription_init_default(
        &cmdvel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) {
    Serial.println("Subscriber init failed!");
    while (1);
  }

  // Initialize /servo_cmd subscriber
  if (rclc_subscription_init_default(
        &servo_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "/servo_cmd") != RCL_RET_OK) {
    Serial.println("Servo subscriber init failed!");
    while (1);
  }

  // Initialize publisher for status
  if (rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
        "/esp32_status") != RCL_RET_OK) {
    Serial.println("Publisher init failed!");
    while (1);
  }

  // Initialize executor (2 subs)
  if (rclc_executor_init(&executor, &support.context, 2, &allocator) != RCL_RET_OK) {
    Serial.println("Executor init failed!");
    while (1);
  }

  // Add subscriptions
  if (rclc_executor_add_subscription(&executor, &cmdvel_sub, &cmdvel_msg, &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Add /cmd_vel failed!");
    while (1);
  }

  if (rclc_executor_add_subscription(&executor, &servo_sub, &servo_msg, &servoCallback, ON_NEW_DATA) != RCL_RET_OK) {
    Serial.println("Add /servo_cmd failed!");
    while (1);
  }

  // Publish connection status
  static char status_text[] = "ESP32 connected";
  status_msg.data.data = status_text;
  status_msg.data.size = strlen(status_text);
  status_msg.data.capacity = strlen(status_text) + 1;
  if (rcl_publish(&publisher, &status_msg, NULL) != RCL_RET_OK) {
    Serial.println("Failed to publish status!");
  } else {
    Serial.println("Published: ESP32 connected");
  }
}

// ===== Loop =====
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
