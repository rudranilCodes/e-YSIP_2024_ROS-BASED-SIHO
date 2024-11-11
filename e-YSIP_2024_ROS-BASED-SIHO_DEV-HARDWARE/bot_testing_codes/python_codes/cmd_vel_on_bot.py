'''
    Author: Pradeep J
    Project: ROS_based_SIHO
    Date:10 Jun 2024
'''

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup,MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import Imu
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import socket
import time
import struct
from tf_transformations import quaternion_from_euler


UDP_IP = "192.168.80.67"  # Listen on all available interfaces
ESP32_IP = '192.168.80.57'
UDP_PORT_1 = 8888



class udp_pub(Node):
    def __init__(self):
        super().__init__('udp_node')

        self.subscriber_1=self.create_subscription(Twist,"/cmd_vel", self.cmd_esp,10)

    def cmd_esp(self,data):
        lin_vel=float(data.linear.x) 
        ang_vel=float(data.angular.z)
        data=[lin_vel,ang_vel]
        print(data)
        
        byte_data = struct.pack('f' * len(data), *data)

        # Create a UDP socket
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(byte_data, (ESP32_IP, UDP_PORT_1))
        sock.close()
        print(f"Message '{data}' sent to {ESP32_IP}:{UDP_PORT_1}")
        
        

def main(args=None):
    rclpy.init(args=args)
    imu_pub = udp_pub()
    executor = MultiThreadedExecutor(3)
    executor.add_node(imu_pub)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        imu_pub.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
    


