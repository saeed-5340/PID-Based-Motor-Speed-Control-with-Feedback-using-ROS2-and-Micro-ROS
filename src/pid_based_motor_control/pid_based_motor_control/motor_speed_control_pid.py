#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32MultiArray
from geometry_msgs.msg import Twist
import time
import math


class MotorSpeedControlNode(Node):
    def __init__(self):
        super().__init__('motor_speed_control_node')
        
        # Define parameters:
        self.declare_parameter('wheel_separation', 0.185)
        self.declare_parameter('wheel_radius', 0.021)
        self.declare_parameter('encoder_ticks_per_revolution', 48)
        self.declare_parameter('pid_kp', 50)
        self.declare_parameter('pid_ki', 0)
        self.declare_parameter('pid_kd', 0)
        # self.declare_parameter('pid_max_output', 255)
        
        
        # Retrieve parameters:
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.encoder_ticks_per_revolution = self.get_parameter('encoder_ticks_per_revolution').value
        self.pid_kp = self.get_parameter('pid_kp').value
        self.pid_ki = self.get_parameter('pid_ki').value
        self.pid_kd = self.get_parameter('pid_kd').value
        # self.pid_max_output = self.get_parameter('pid_max_output').value
        
        
        # publisher and subscriber:
        
        self.motor_command_pub = self.create_publisher(Int32MultiArray,'/get_pwm_values',10)
        
        self.encoder_sub = self.create_subscription(
            Int32MultiArray,
            '/get_ticks',
            self.encoder_data_callback,
            10
        )
        
        # self.cmd_vel_sub = self.create_subscription(Twist,'/cmd_vel',self.cmd_vel_callback,10)
        
        self.control_timer = self.create_timer(0.5, self.control_loop)
        
        
        # static variables:
        self.last_left_encoder_value = 0
        self.last_right_encoder_value = 0
        self.left_wheel_actual_speed = 0     
        self.right_wheel_actual_speed = 0
        self.left_wheel_desired_speed = 5
        self.right_wheel_desired_speed = 5
        self.last_time = self.get_clock().now()

        # PID control variables:
        self.left_wheel_error_sum = 0
        self.right_wheel_error_sum = 0
        self.last_left_wheel_error = 0
        self.last_right_wheel_error = 0
        
        
    def encoder_data_callback(self, msg):
        left = msg.data[0]   
        right = msg.data[1]
        time_now = self.get_clock().now()
        measure_time = (time_now - self.last_time).nanoseconds / 1e9
        
        if measure_time > 0:
            N_left = ((left - self.last_left_encoder_value)/self.encoder_ticks_per_revolution) 
            self.left_wheel_actual_speed = ((2 * math.pi * N_left) / measure_time )         # rad/s
            # self.left_wheel_actual_speed = self.left_wheel_actual_speed * self.wheel_radius    # m/s
            
            
            N_right = ((right - self.last_right_encoder_value)/self.encoder_ticks_per_revolution) 
            self.right_wheel_actual_speed = ((2 * math.pi * N_right) / measure_time )           # rad/s
            # self.right_wheel_actual_speed = self.right_wheel_actual_speed* self.wheel_radius    #m/s
            
            self.last_left_encoder_value = left
            self.last_right_encoder_value = right
            self.last_time = time_now
            
        self.get_logger().info(f"Received encoder data: {left}, {right}")
            
    def control_loop(self):
        # current_time = self.get_clock().now()
        # dt = (current_time - self.last_control_time).nanoseconds / 1e9
        # self.last_control_time = current_time
        # if dt <= 0:
        #     return
        left_error = self.left_wheel_desired_speed - self.left_wheel_actual_speed
        dt = 0.5
        P_term_L = left_error * self.pid_kp
        self.left_wheel_error_sum += left_error * dt
        I_term_L = self.pid_ki * self.left_wheel_error_sum
        left_derivative = (left_error - self.last_left_wheel_error)/dt
        D_term_L = self.pid_kd * left_derivative
        output_left = P_term_L + I_term_L + D_term_L
        
        
        right_error = self.right_wheel_desired_speed - self.right_wheel_actual_speed
        P_term_R = right_error * self.pid_kp
        self.right_wheel_error_sum += right_error*dt
        I_term_R = self.pid_ki * self.right_wheel_error_sum
        right_derivative = (right_error - self.last_right_wheel_error) / dt
        D_term_R = self.pid_kd * right_derivative
        output_right = P_term_R + I_term_R + D_term_R
        
                
        self.last_left_wheel_error = left_error
        self.last_right_wheel_error = right_error
        
        
        cmd_msg = Int32MultiArray()
        cmd_msg.data = [int(round(output_left)),int(round(output_right))]
        self.motor_command_pub.publish(cmd_msg)
        self.get_logger().info(f'Left out: {cmd_msg.data[0]}, Right out: {cmd_msg.data[1]}')
    
    
def main(args=None):
    rclpy.init(args=args)
    node = MotorSpeedControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()