#include <micro_ros_arduino.h>
#include <Arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32_multi_array.h>

// ===================== MOTOR PINS =====================
#define IN1 23
#define IN2 14
#define ENA 16

// ===================== ENCODER PINS =====================
#define ENC_R_A 33
#define ENC_R_B 32

// ===================== micro-ROS =====================
rcl_allocator_t allocator;
rcl_subscription_t subscriber;
rcl_publisher_t publisher;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rclc_support_t support;

std_msgs__msg__Int32MultiArray encoder_msg;
std_msgs__msg__Int32MultiArray motor_msg;

// ===================== ENCODER =====================
volatile long encoder_count = 0;

void IRAM_ATTR encoderISR() {
  if (digitalRead(ENC_R_B) == HIGH)
    encoder_count++;
  else
    encoder_count--;
}

// ===================== MOTOR CONTROL =====================
void set_motor(int pwm_value) {
  if (pwm_value >= 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwm_value);
}

// ===================== CALLBACKS =====================
void publish_encoder_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer != NULL) {
    encoder_msg.data.data[0] = encoder_count;
    rcl_publish(&publisher, &encoder_msg, NULL);
  }
}

void pwm_command_callback(const void *msgin) {
  const std_msgs__msg__Int32MultiArray *msg =
    (const std_msgs__msg__Int32MultiArray *)msgin;

  int pwm = msg->data.data[0];
  set_motor(pwm);
}

// ===================== SETUP =====================
void setup() {
  Serial.begin(115200);
  set_microros_transports();

  // Motor pins
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  // Encoder pins
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), encoderISR, RISING);

  // micro-ROS init
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_motor_node", "", &support);

  // Publisher — sends encoder ticks
  encoder_msg.data.size = 1;
  encoder_msg.data.capacity = 1;
  encoder_msg.data.data = (int32_t*)malloc(1 * sizeof(int32_t));
  encoder_msg.data.data[0] = 0;

  rclc_publisher_init_default(
    &publisher, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/get_ticks"
  );

  // Subscriber — receives PWM command
  motor_msg.data.size = 1;
  motor_msg.data.capacity = 1;
  motor_msg.data.data = (int32_t*)malloc(1 * sizeof(int32_t));

  rclc_subscription_init_default(
    &subscriber, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "/get_pwm_values"
  );

  // Timer — publish encoder every 50ms
  rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10), publish_encoder_callback);

  // Executor
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_timer(&executor, &timer);
  rclc_executor_add_subscription(&executor, &subscriber, &motor_msg, pwm_command_callback, ON_NEW_DATA);
}

// ===================== LOOP =====================
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
  delay(1);
}