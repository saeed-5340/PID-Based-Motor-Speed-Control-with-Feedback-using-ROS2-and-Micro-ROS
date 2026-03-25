#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
import math


class MotorSpeedControlNode(Node):
    def __init__(self):
        super().__init__('motor_speed_control_node')

        # ===================== PARAMETERS =====================
        self.declare_parameter('encoder_ticks_per_revolution', 48)
        self.declare_parameter('desired_speed_rad_s', 25.0)

        # PID gains — start with P only, set ki=0, kd=0
        self.declare_parameter('pid_kp', 8.0)
        self.declare_parameter('pid_ki', 2.0)   # enable after P is tuned
        self.declare_parameter('pid_kd', 0.3)   # enable after I is tuned

        self.encoder_ticks_per_revolution = self.get_parameter('encoder_ticks_per_revolution').value
        self.desired_speed = self.get_parameter('desired_speed_rad_s').value
        self.kp = self.get_parameter('pid_kp').value
        self.ki = self.get_parameter('pid_ki').value
        self.kd = self.get_parameter('pid_kd').value

        # ===================== PUBLISHER & SUBSCRIBER =====================
        self.pwm_pub = self.create_publisher(Int32MultiArray, '/get_pwm_values', 10)

        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/get_ticks',
            self.encoder_callback,
            10
        )

        # Control loop runs every 100ms (matches well with 50ms encoder publish)
        self.control_timer = self.create_timer(0.1, self.control_loop)

        # ===================== STATE VARIABLES =====================
        self.last_encoder_value = 0
        self.actual_speed = 0.0         # rad/s
        self.min_pwm = 70
        self.last_time = self.get_clock().now()
        self.filter_alpha = 0.3
        self.filter_speed = 0.0

        # PID state
        self.error_sum = 0.0
        self.last_error = 0.0

        # Anti-windup limit for integrator
        self.integral_limit = 200.0

        self.get_logger().info('Motor Speed Control Node started')
        self.get_logger().info(f'Desired speed: {self.desired_speed} rad/s')
        # self.get_logger().info(f'PID gains -> Kp: {self.kp}, Ki: {self.ki}, Kd: {self.kd}')


    # ===================== ENCODER CALLBACK =====================
    def encoder_callback(self, msg):
        current_ticks = msg.data[0]
        time_now = self.get_clock().now()
        dt = (time_now - self.last_time).nanoseconds / 1e9
        
        if dt < 0.04:
            return

        if dt > 0:
            delta_ticks = current_ticks - self.last_encoder_value
            revolutions = delta_ticks / self.encoder_ticks_per_revolution
            raw_speed = (2.0 * math.pi * revolutions) / dt  # rad/s
            
            self.filter_speed = (self.filter_alpha*raw_speed) + ((1-self.filter_alpha)*self.filter_speed)
            self.actual_speed = self.filter_speed

            self.last_encoder_value = current_ticks
            self.last_time = time_now

        self.get_logger().info(
            f'Ticks: {current_ticks} | Speed: {self.actual_speed:.3f} rad/s'
        )


    # ===================== PID CONTROL LOOP =====================
    def control_loop(self):
        dt = 0.1  # matches timer interval

        error = self.desired_speed - self.actual_speed

        # --- P term ---
        P = self.kp * error

        # --- I term (with anti-windup) ---
        self.error_sum += error * dt
        # self.error_sum = max(-self.integral_limit,
        #                       min(self.integral_limit, self.error_sum))
        I = self.ki * self.error_sum

        # --- D term ---
        derivative = (error - self.last_error) / dt
        D = self.kd * derivative

        self.last_error = error

        # --- Total output ---
        output = P + I + D

        # Clamp output to valid PWM range
        output = int(round(max(self.min_pwm, min(255.0, output))))

        # Publish PWM command
        # output = int(round(output))
        cmd_msg = Int32MultiArray()
        cmd_msg.data = [output]
        self.pwm_pub.publish(cmd_msg)

        self.get_logger().info(
            # f'Error: {error:.3f} | P: {P:.1f} | I: {I:.1f} | D: {D:.1f} | PWM: {output}'
            f'Error: {error:.3f} | PWM: {output} | Speed: {self.actual_speed:.3f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()