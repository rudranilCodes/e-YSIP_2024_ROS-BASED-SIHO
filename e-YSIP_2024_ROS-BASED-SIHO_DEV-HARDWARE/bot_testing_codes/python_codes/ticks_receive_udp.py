import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import socket
import struct

class UDPLaserScanPublisher(Node):
    def __init__(self):
        super().__init__('udp_laser_scan_publisher')
        self.timer = self.create_timer(0.001, self.timer_callback)

        self.udp_ip = "192.168.80.67"
        self.udp_port = 4210

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))

        # self.get_logger().info(f"Listening on {self.udp_ip}:{self.udp_port}")

    def timer_callback(self):
        data, addr = self.sock.recvfrom(4 * 4)  # Buffer size to handle 360 floats (4 bytes each)
        arr = struct.unpack('4f', data)
        # print(addr)
        # self.get_logger().info(f"Received array from {addr}")

        print(arr)


def main(args=None):
    rclpy.init(args=args)
    udp_laser_scan_publisher = UDPLaserScanPublisher()
    rclpy.spin(udp_laser_scan_publisher)
    udp_laser_scan_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()