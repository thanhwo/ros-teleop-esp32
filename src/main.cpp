#include <Arduino.h>
#include <micro_ros_arduino.h>
#include <WiFi.h>
#include <ESP32Servo.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>

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
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;

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

  int speed = map(abs(linear_x * 100), 0, 100, 0, 255);

  if (linear_x > 0.05) {            // forward
    setMotor(speed, HIGH, LOW, speed, HIGH, LOW);
  } else if (linear_x < -0.05) {    // backward
    setMotor(speed, LOW, HIGH, speed, LOW, HIGH);
  } else if (angular_z > 0.05) {    // turn left
    setMotor(speed, LOW, HIGH, speed, HIGH, LOW);
  } else if (angular_z < -0.05) {   // turn right
    setMotor(speed, HIGH, LOW, speed, LOW, HIGH);
  } else {                          // stop
    setMotor(0, HIGH, HIGH, 0, HIGH, HIGH);
  }
}

// ===== Setup =====
void setup() {
  Serial.begin(115200);

  // WiFi transport cho micro-ROS
  char ssid[] = "InfinixNOTE30";
  char pass[] = "12345678";
  char agent_ip[] = "192.168.1.100";   // đổi theo PC chạy ROS2 agent
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

  delay(2000);

  // ROS2 init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);
  rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "/cmd_vel");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &msg, &cmdVelCallback, ON_NEW_DATA);
}

// ===== Loop =====
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
