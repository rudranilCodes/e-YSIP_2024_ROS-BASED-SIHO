#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import math
import numpy as np
from Align2D import Align2D
import matplotlib.pyplot as plt


axes_points = np.array([[0,1,0],
					    [0,0,1],
					    [1,1,1]])

class Scan2Odom(Node):
    def __init__(self):
        super().__init__('tf_listener')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',  # Replace with your scan topic name
            self.scan_callback,
            10)

        self.previous_ranges = None
        self.new_ranges = None

        self.previous_pcl = None
        self.new_pcl = None

        self.pose = np.identity(3)


    def scan_callback(self, msg):
        if self.previous_ranges is None:
            self.previous_ranges = self.new_ranges
        self.new_ranges = msg.ranges
        
    
    def Range2Pcl(self, ranges):
        beam_angle_increment = math.pi / 180.0
        beam_angle = 0.0
        points = []
        for beam_length in ranges:

			# only convert points with nonzero range
            if str(beam_length) != 'inf' and beam_length > 0.05:

				# convert polar to cartesian coordinates
                point_x = beam_length * math.cos(beam_angle)
                point_y = beam_length * math.sin(beam_angle)

				# store x and y values in a numpy array and append it to the point list
                point = np.array([point_x,point_y,1.0])
                
                points.append(point)

			# increment the beam angle for the next point
            beam_angle += beam_angle_increment

        return np.array(points)
    

def main(args=None):
    rclpy.init(args=args)
    
    # Create an instance of the HBController class
    node = Scan2Odom()
    rclpy.spin_once(node)
    plt.figure()
    # plt.ion()  # Turn on interactive mode

    rclpy.spin_once(node)
    # Main loop
    while rclpy.ok():
        # if node.previous_ranges is not None and node.new_ranges is not None: 
        if node.previous_ranges is not None and node.new_ranges is not None:
            node.new_pcl = node.Range2Pcl(node.new_ranges)
            node.previous_pcl = node.Range2Pcl(node.previous_ranges)

            aligner = Align2D(node.new_pcl,node.previous_pcl, np.identity(3))
            est_T = aligner.transform
            
            target_points = np.dot(node.new_pcl, node.pose.T)

            node.pose = np.dot(node.pose, est_T)

            source_points = np.dot(node.previous_pcl, node.pose.T)

            tf_axes = np.dot(node.pose, axes_points)
            plt.clf()
            plt.xlim(xmax = 3, xmin = -3)
            plt.ylim(ymax =  3, ymin = -3)

            # plot robot pose
            plt.plot(tf_axes[0,:2],tf_axes[1,:2],c='r')
            plt.plot(tf_axes[0,:3:2],tf_axes[1,:3:2],c='g')

            # plot source and target point clouds
            plt.scatter(target_points[:,0],target_points[:,1],marker='.',s=1.0,c='c')
            plt.scatter(source_points[:,0],source_points[:,1],marker=',',s=1.0,c='m')
            plt.pause(0.01)
            node.previous_ranges = node.new_ranges

        # Spin once to process callbacks
        rclpy.spin_once(node)
    
    plt.show()
    # Destroy the node and shut down ROS
    node.destroy_node()
    rclpy.shutdown()

# Entry point of the script
if __name__ == '__main__':
    main()