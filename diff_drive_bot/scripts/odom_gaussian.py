#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion
import numpy as np
import random

class Gaussian_Sensor_data(Node):
    def __init__(self):
        super().__init__('Sensors_Node')
        
        self.enc_odom_sub = self.create_subscription(Odometry, "/odom", self.odom_callback, 10)
        self.imu_data = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.odom_gaussian = self.create_publisher(Odometry, "/odom/Gaussian", 10)
        self.imu_gasussian = self.create_publisher(Imu, "/imu/data/Gaussian", 10)
        self.noise_stddev = 1.0
        self.a_B = 0.0
        self.omega_B = 0.0
        self.euler_angles = 0.0
        self.odom_noisy = Odometry()
        self.imu_noisy = Imu()

    def odom_callback(self,msg):
        
        self.linear_vel_x = msg.twist.twist.linear.x + random.gauss(0, self.noise_stddev)
        self.linear_vel_y = msg.twist.twist.linear.y + random.gauss(0, self.noise_stddev)
        self.odom_noisy.header.frame_id = 'odom_noisy'
        self.odom_noisy.child_frame_id = 'base_link'
        self.odom_noisy.header.stamp = self.get_clock().now().to_msg()
        self.odom_noisy.twist.twist.linear.x = self.linear_vel_x
        self.odom_noisy.twist.twist.linear.y = self.linear_vel_y
        self.odom_gaussian.publish(self.odom_noisy)

    def imu_callback(self, msg):

        self.a_B = np.array([msg.linear_acceleration.x + random.gauss(0, self.noise_stddev), msg.linear_acceleration.y + random.gauss(0, self.noise_stddev), msg.linear_acceleration.z + random.gauss(0, self.noise_stddev)])
        self.omega_B = np.array([msg.angular_velocity.x + random.gauss(0, self.noise_stddev), msg.angular_velocity.y + random.gauss(0, self.noise_stddev), msg.angular_velocity.z + random.gauss(0, self.noise_stddev)])
        self.euler_angles = euler_from_quaternion([msg.orientation.x + random.gauss(0, self.noise_stddev), msg.orientation.y + random.gauss(0, self.noise_stddev), msg.orientation.z, msg.orientation.w + random.gauss(0, self.noise_stddev)])
        self.imu_noisy.linear_acceleration.x = self.a_B[0]
        self.imu_noisy.linear_acceleration.y = self.a_B[1]
        self.imu_noisy.linear_acceleration.y = self.a_B[2]
        self.imu_noisy.angular_velocity.x = self.omega_B[0]
        self.imu_noisy.angular_velocity.y = self.omega_B[1]
        self.imu_noisy.angular_velocity.z = self.omega_B[2]
        self.imu_noisy.orientation.x = self.euler_angles[0]
        self.imu_noisy.orientation.y = self.euler_angles[1]
        self.imu_noisy.orientation.z = self.euler_angles[2]
        self.imu_gasussian.publish(self.imu_noisy)

def main(args = None):
    rclpy.init(args=args)
    Sensor_Node = Gaussian_Sensor_data()
    rclpy.spin(Sensor_Node)
    Sensor_Node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




