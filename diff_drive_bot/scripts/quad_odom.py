#!/usr/bin/env python3

"""
Author: Rudranil Bose
Project: ROS-Based SLAM Implementation and Hardware Optimization for Small Differential Drive Bot
eYSIP 2024

"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
from nav_msgs.msg import Odometry

class Odom_from_encoder(Node):
    def __init__(self):
        super().__init__('Odometry_class')
        # Define variables for encoder ticks and wheelbase
        self.left_fw_ticks = 0.0
        self.right_fw_ticks = 0.0
        self.left_bw_ticks = 0.0
        self.right_bw_ticks = 0.0
        self.wheelbase = 0.4  # Example value, adjust based on your robot

        # State of the channels for direction detection
        self.left_fw_last_state = 0.0
        self.right_fw_last_state = 0.0
        self.left_bw_last_state = 0.0
        self.right_bw_last_state = 0.0


        self.left_fw_c1_ticks_sub = self.create_subscription(Float32, '/front_left_wheel/channel_a', self.left_fw_ticks_c1_callback, 10 )
        self.left_fw_c2_ticks_sub = self.create_subscription(Float32, '/front_left_wheel/channel_b', self.left_fw_ticks_c2_callback, 10 )

        self.right_fw_c1_ticks_sub = self.create_subscription(Float32, '/front_right_wheel/channel_a', self.right_fw_ticks_c1_callback, 10 )
        self.right_fw_c2_ticks_sub = self.create_subscription(Float32, '/front_right_wheel/channel_b', self.right_fw_ticks_c2_callback, 10 )

        self.left_bw_c1_ticks_sub = self.create_subscription(Float32, '/back_left_wheel/channel_a', self.left_bw_ticks_c1_callback, 10 )
        self.left_bw_c2_ticks_sub = self.create_subscription(Float32, '/back_left_wheel/channel_b', self.left_bw_ticks_c2_callback, 10 )

        self.right_bw_c1_ticks_sub = self.create_subscription(Float32, '/back_right_wheel/channel_a', self.right_bw_ticks_c1_callback, 10 )
        self.right_bw_c2_ticks_sub = self.create_subscription(Float32, '/back_right_wheel/channel_b', self.right_bw_ticks_c2_callback, 10 )

        # Create publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/enc_odom', 10)

        # Create a timer to periodically publish odometry
        self.timer = self.create_timer(0.1, self.publish_encoder_odometry)

        # Define variables for current pose and orientation
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.odom = Odometry()

    def left_fw_ticks_c1_callback(self, msg):
        self.update_ticks(msg.data, 'left_fw')

    def left_fw_ticks_c2_callback(self, msg):
        self.left_fw_last_state = msg.data

    def right_fw_ticks_c1_callback(self, msg):
        self.update_ticks(msg.data, 'right_fw')

    def right_fw_ticks_c2_callback(self, msg):
        self.right_fw_last_state = msg.data

    def left_bw_ticks_c1_callback(self, msg):
        self.update_ticks(msg.data, 'left_bw')

    def left_bw_ticks_c2_callback(self, msg):
        self.left_bw_last_state = msg.data

    def right_bw_ticks_c1_callback(self, msg):
        self.update_ticks(msg.data, 'right_bw')

    def right_bw_ticks_c2_callback(self, msg):
        self.right_bw_last_state = msg.data




    def update_ticks(self, channel_a_value, wheel):
        if wheel == 'left_fw':
            if channel_a_value == 1 and self.left_fw_last_state == 0:
                self.left_fw_ticks += 1
            elif channel_a_value == 0 and self.left_fw_last_state == 1:
                self.left_fw_ticks -= 1

        elif wheel == 'right_fw':
            if channel_a_value == 1 and self.right_fw_last_state == 0:
                self.right_fw_ticks += 1
            elif channel_a_value == 0 and self.right_fw_last_state == 1:
                self.right_fw_ticks -= 1

        elif wheel == 'left_bw':
            if channel_a_value == 1 and self.left_bw_last_state == 0:
                self.left_bw_ticks += 1
            elif channel_a_value == 0 and self.left_bw_last_state == 1:
                self.left_bw_ticks -= 1

        elif wheel == 'right_bw':
            if channel_a_value == 1 and self.right_bw_last_state == 0:
                self.right_bw_ticks += 1
            elif channel_a_value == 0 and self.right_bw_last_state == 1:
                self.right_bw_ticks -= 1

    def calculate_distance(self, ticks):
        # Calculate the distance traveled based on the number of ticks
        wheel_circumference = 2 * math.pi * 0.040  
        distance_per_tick = wheel_circumference / 894  # Ticks per revolution
        return ticks * distance_per_tick

    def publish_encoder_odometry(self):

        left_fw_distance = self.calculate_distance(self.left_fw_ticks)
        left_bw_distance = self.calculate_distance(self.left_bw_ticks)
        right_fw_distance = self.calculate_distance(self.right_fw_ticks)
        right_bw_distance = self.calculate_distance(self.right_bw_ticks)

        left_distance = (left_fw_distance + left_bw_distance) / 2.0
        right_distance = (right_fw_distance + right_bw_distance) / 2.0

        # Calculate change in position and orientation
        delta_theta = (right_distance - left_distance) / self.wheelbase
        delta_x = (left_distance + right_distance) / 2.0 * math.cos(self.current_theta + delta_theta / 2.0)
        delta_y = (left_distance + right_distance) / 2.0 * math.sin(self.current_theta + delta_theta / 2.0)

        # Update current pose and orientation
        self.current_x += delta_x
        self.current_y += delta_y
        self.current_theta += delta_theta

        # Create Odometry message
        self.odom.header.frame_id = 'enc_odom'
        self.odom.child_frame_id = 'base_link'
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.pose.pose.position.x = self.current_x
        self.odom.pose.pose.position.y = self.current_y
        self.odom.pose.pose.orientation.z = math.sin(self.current_theta / 2.0)
        self.odom.pose.pose.orientation.w = math.cos(self.current_theta / 2.0)

        # Publish the Odometry message
        self.odom_pub.publish(self.odom)

        
def main(args=None):
    rclpy.init(args=args)
    node = Odom_from_encoder()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()



