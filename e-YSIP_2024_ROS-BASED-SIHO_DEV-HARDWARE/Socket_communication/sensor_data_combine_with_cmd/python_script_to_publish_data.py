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


UDP_IP = "192.168.113.67"  # Listen on all available interfaces
ESP32_IP = '192.168.113.118'
UDP_PORT_1 = 4210
UDP_PORT_2 = 8888
UDP_PORT_3 = 6666


class udp_pub(Node):
    def __init__(self):
        super().__init__('udp_node')
        self.publisher_1=self.create_publisher(Imu,'/imu',10)
        self.publisher_2=self.create_publisher(LaserScan,'/scan',10)
        self.subscriber_1=self.create_subscription(Twist,"/cmd_vel", self.cmd_esp,10)

        self.callback_group_1=MutuallyExclusiveCallbackGroup()
        self.callback_group_2=MutuallyExclusiveCallbackGroup()
        
        self.timer_1=self.create_timer(0.001,self.receive_data,callback_group=self.callback_group_1)
        self.timer_2=self.create_timer(0.001,self.lidar_data,callback_group=self.callback_group_2)
        
        
        self.start_time = time.time()
        self.packet_count = 0
        self.last_packet_time = self.start_time
        
        self.start_time_1 = time.time()
        self.packet_count_1 = 0
        self.last_packet_time_1 = self.start_time_1
        
    def receive_data(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_1))

        try:
            data, addr = sock.recvfrom(9 * 4)
            num_ints = len(data) // struct.calcsize('i')
            fmt = str(num_ints)+"f"
            values = struct.unpack(fmt, data[:struct.calcsize(fmt)])

            # print("Received values:", round(values[0],2),round(values[1],2),round(values[2],2),round(values[3],2),round(values[4],2),round(values[5],2),round(values[6],2),round(values[7],2),round(values[0],2),8)
            current_time = time.time()
            self.packet_count += 1
            time_diff = current_time - self.last_packet_time
            self.last_packet_time = current_time
            # print(f"Packet received #{self.packet_count}, Time since last packet: {time_diff:.6f}s")

            new_data=Imu()
            new_data.header.stamp= self.get_clock().now().to_msg()
            new_data.header.frame_id="imu"
            ori=quaternion_from_euler(values[6],values[7],values[8])
            new_data.orientation.x=ori[0]
            new_data.orientation.y=ori[1]
            new_data.orientation.z=ori[2]
            new_data.orientation.w=ori[3]
            
            new_data.angular_velocity.x=values[3]
            new_data.angular_velocity.y=values[4]
            new_data.angular_velocity.z=values[5]
            
            new_data.linear_acceleration.x=values[0]
            new_data.linear_acceleration.y=values[1]
            new_data.linear_acceleration.z=values[2]
            
            self.publisher_1.publish(new_data)

        
        except KeyboardInterrupt:
            end_time = time.time()
            duration = end_time - self.start_time
            if duration != 0:
                frequency = self.packet_count / duration
                print(f"\nFrequency of UDP connection: {frequency:.2f} packets/s")
    
    def lidar_data(self):

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT_2))

        try:
            data, addr = sock.recvfrom(360 * 4)
      
            num_ints = len(data) // struct.calcsize('f')
            fmt = str(num_ints)+"f"
            
            values = struct.unpack(fmt, data[:struct.calcsize(fmt)])

            list1=list(values)
            for i in range(360):
                list1[i]=list1[i]/1000
            # print("Received values:", values)
            current_time_1 = time.time()
            self.packet_count_1 += 1
            time_diff = current_time_1 - self.last_packet_time_1
            self.last_packet_time_1 = current_time_1
            # print(f"Packet received #{self.packet_count_1}, Time since last packet: {time_diff:.6f}s")
            
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser_frame'  

            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 3.141592653589793*2
            scan_msg.angle_increment = 3.141592653589793 / 180
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0
            scan_msg.range_min = 0.2  
            scan_msg.range_max = 12.0  

            scan_msg.ranges = list1
            scan_msg.intensities = []  
            
            self.publisher_2.publish(scan_msg)
            
        except KeyboardInterrupt:
            end_time = time.time()
            duration = end_time - self.start_time_1
            if duration != 0:
                frequency = self.packet_count_1 / duration
                print(f"\nFrequency of UDP connection: {frequency:.2f} packets/s")
    
    def cmd_esp(self,data):
        lin_vel=str(data.linear.x) +str(" ")+str(data.linear.y)
        ang_vel=str(data.angular.z)
        message = str(lin_vel)+str(" ")+str(ang_vel)
        print(message)
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.sendto(message.encode(), (ESP32_IP, UDP_PORT_3))
        sock.close()
        print(f"Message '{message}' sent to {ESP32_IP}:{UDP_PORT_3}")
        
        

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
    

