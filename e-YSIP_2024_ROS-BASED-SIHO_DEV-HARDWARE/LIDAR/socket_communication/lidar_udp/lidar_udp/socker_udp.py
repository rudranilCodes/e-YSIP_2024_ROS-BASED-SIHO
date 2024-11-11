import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import time
import struct

UDP_IP = "192.168.201.67"
UDP_PORT = 8888

class udp_pub(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.timer_=self.create_timer(0.001,self.receive_data)
        self.publisher_2=self.create_publisher(LaserScan,'/scan',10)
        self.start_time = time.time()
        self.packet_count = 0
        self.last_packet_time = self.start_time
        
    def receive_data(self,):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))

        try:
            data, addr = sock.recvfrom(360 * 4)
            num_ints = len(data) // struct.calcsize('i')
            fmt = str(num_ints)+"f"
            
            values = struct.unpack(fmt, data[:struct.calcsize(fmt)])
            value_arr=list(values)
            for i in range(360):
                value_arr[i]=value_arr[i]/1000
            print("Received values:", value_arr[10])
            current_time = time.time()
            self.packet_count += 1
            time_diff = current_time - self.last_packet_time
            self.last_packet_time = current_time
            
            scan_msg = LaserScan()
            scan_msg.header.stamp = self.get_clock().now().to_msg()
            scan_msg.header.frame_id = 'laser_frame'  

            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 3.141592653589793*2
            scan_msg.angle_increment = 3.141592653589793 / 180
            scan_msg.time_increment = 0.0
            scan_msg.scan_time = 1.0
            scan_msg.range_min = 0.5  
            scan_msg.range_max = 12.0  

            scan_msg.ranges = list(value_arr)  
            scan_msg.intensities = []  
            
            self.publisher_2.publish(scan_msg)
        
        except KeyboardInterrupt:
            end_time = time.time()
            duration = end_time - self.start_time
            if duration != 0:
                frequency = self.packet_count / duration
                print(f"\nFrequency of UDP connection: {frequency:.2f} packets/s")


def main(args=None):
    rclpy.init(args=args)
    imu_pub = udp_pub()
    rclpy.spin(imu_pub)
    imu_pub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

