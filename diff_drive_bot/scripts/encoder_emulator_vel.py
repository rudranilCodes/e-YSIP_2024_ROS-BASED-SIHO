#!/usr/bin/env python3

"""

Author: Rudranil Bose
Project: ROS-Based SLAM Implementation and Hardware Optimization for Small Differential Drive Bot
eYSIP 2024

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import math
import random

class Encoder_emulator(Node):
    def __init__(self):
        super().__init__('encoder_emulator')

        self.encoder_pub_left_fw_a = self.create_publisher(Float32, '/front_left_wheel/channel_a', 10)
        self.encoder_pub_left_fw_b = self.create_publisher(Float32, '/front_left_wheel/channel_b', 10)
        self.encoder_pub_left_bw_a = self.create_publisher(Float32, '/back_left_wheel/channel_a', 10)
        self.encoder_pub_left_bw_b = self.create_publisher(Float32, '/back_left_wheel/channel_b', 10)
        self.encoder_pub_right_fw_a = self.create_publisher(Float32, '/front_right_wheel/channel_a', 10)
        self.encoder_pub_right_fw_b = self.create_publisher(Float32, '/front_right_wheel/channel_b', 10)
        self.encoder_pub_right_bw_a = self.create_publisher(Float32, '/back_right_wheel/echannel_a', 10)
        self.encoder_pub_right_bw_b = self.create_publisher(Float32, '/back_right_wheel/channel_b', 10)
     
        self.vel_sub = self.create_subscription( Twist, '/cmd_vel',self.vel_callback, 50000)
        

        self.encoder_left_bw_ticks = 0.0
        self.encoder_left_fw_ticks = 0.0
        self.encoder_right_fw_ticks = 0.0
        self.encoder_right_bw_ticks = 0.0
        

        self.wheel_base = 0.0816  # Distance between left and right wheels
        self.wheel_track = 0.097  # Distance between front and back wheels
        self.wheel_radius = 0.080  # Radius of the wheel
        self.ticks_per_wheel_revolution = 50  # Number of ticks per wheel revolution

        # Standard deviation for Gaussian noise
        self.noise_stddev = 1.0  
        self.rate = self.create_rate(10)
        self.create_timer(0.1, self.run)

    def vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z
        print("lin", linear_velocity)
        print("ang", angular_velocity)
        # Calculating left and right wheel velocities
        left_wheel_velocity = abs(linear_velocity - (self.wheel_base / 2.0) * angular_velocity)
        right_wheel_velocity = abs(linear_velocity + (self.wheel_base / 2.0) * angular_velocity)
        print("lwv:", left_wheel_velocity)
        print("rwv", right_wheel_velocity)
        
        # Calculate wheel rotations per second
        left_wheel_rotations_per_sec = left_wheel_velocity / (2 * math.pi * self.wheel_radius)
        right_wheel_rotations_per_sec = right_wheel_velocity / (2 * math.pi * self.wheel_radius)
        
        delta_t = 0.1  # Time step duration in seconds 

        # Update encoder ticks for left and right sides
        delta_ticks_left = left_wheel_rotations_per_sec * self.ticks_per_wheel_revolution * delta_t
        delta_ticks_right = right_wheel_rotations_per_sec * self.ticks_per_wheel_revolution * delta_t

        # Add Gaussian noise
        delta_ticks_left += random.gauss(0, self.noise_stddev)
        delta_ticks_right += random.gauss(0, self.noise_stddev)
        # Update cumulative encoder ticks for each wheel
        self.encoder_left_fw_ticks += delta_ticks_left
        self.encoder_right_bw_ticks += delta_ticks_right
        self.encoder_left_bw_ticks += delta_ticks_left
        self.encoder_right_bw_ticks += delta_ticks_right

        
    def get_quadrature_signal(self, ticks):
        # Determine the state of A and B channels based on tick count
        state_a = (ticks // 2) % 2
        state_b = ((ticks + 1) // 2) % 2
        return state_a, state_b
    


    def run(self):
        left_fw_a, left_fw_b = self.get_quadrature_signal(self.encoder_left_fw_ticks)
        left_bw_a, left_bw_b = self.get_quadrature_signal(self.encoder_left_bw_ticks)
        right_fw_a, right_fw_b = self.get_quadrature_signal(self.encoder_right_fw_ticks)
        right_bw_a, right_bw_b = self.get_quadrature_signal(self.encoder_right_bw_ticks)

        msg_left_fw_a = Float32()
        msg_left_fw_a.data = left_fw_a
        msg_left_fw_b = Float32()
        msg_left_fw_b.data = left_fw_b

        msg_left_bw_a = Float32()
        msg_left_bw_a.data = left_bw_a
        msg_left_bw_b = Float32()
        msg_left_bw_b.data = left_bw_b

        msg_right_fw_a = Float32()
        msg_right_fw_a.data = right_fw_a
        msg_right_fw_b = Float32()
        msg_right_fw_b.data = right_fw_b

        msg_right_bw_a = Float32()
        msg_right_bw_a.data = right_bw_a
        msg_right_bw_b = Float32()
        msg_right_bw_b.data = right_bw_b

        self.encoder_pub_left_fw_a.publish(msg_left_fw_a)
        self.encoder_pub_left_fw_b.publish(msg_left_fw_b)
        self.encoder_pub_left_bw_a.publish(msg_left_bw_a)
        self.encoder_pub_left_bw_b.publish(msg_left_bw_b)
        self.encoder_pub_right_fw_a.publish(msg_right_fw_a)
        self.encoder_pub_right_fw_b.publish(msg_right_fw_b)
        self.encoder_pub_right_bw_a.publish(msg_right_bw_a)
        self.encoder_pub_right_bw_b.publish(msg_right_bw_b)

def main(args=None):
    rclpy.init(args=args)
    enc = Encoder_emulator() 
    rclpy.spin(enc)
    enc.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()