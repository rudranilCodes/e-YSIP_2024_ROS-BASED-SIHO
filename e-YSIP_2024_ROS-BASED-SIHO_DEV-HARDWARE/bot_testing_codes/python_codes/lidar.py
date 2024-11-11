import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct

class UDPLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('udp_laser_scan_publisher')
        self.publisher_ = self.create_publisher(LaserScan, 'scan', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.udp_ip = "192.168.80.67"
        self.udp_port = 6666

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        self.get_logger().info(f"Listening on {self.udp_ip}:{self.udp_port}")

    def timer_callback(self):
        data, addr = self.sock.recvfrom(360 * 4)  # Buffer size to handle 360 floats (4 bytes each)
        arr = struct.unpack('360f', data)
        self.get_logger().info(f"Received array from {addr}")

        # Create and populate the LaserScan message
        scan_msg = LaserScan()
        scan_msg.header.stamp = self.get_clock().now().to_msg()
        scan_msg.header.frame_id = 'laser_frame'  

        scan_msg.angle_min = 0.0
        scan_msg.angle_max = 3.141592653589793*2
        scan_msg.angle_increment = 3.141592653589793 / 180
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = 1.0
        scan_msg.range_min = 2.0  
        scan_msg.range_max = 12.0  

        scan_msg.ranges = list(arr)  
        scan_msg.intensities = []  

        # Publish the message
        self.publisher_.publish(scan_msg)
        self.get_logger().info("LaserScan message published")

def main(args=None):
    rclpy.init(args=args)
    udp_laser_scan_publisher = UDPLaserScanPublisher()
    rclpy.spin(udp_laser_scan_publisher)
    udp_laser_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
