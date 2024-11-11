import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import socket
import time
import struct
from tf_transformations import quaternion_from_euler

UDP_IP = "192.168.80.67"  # Listen on all available interfaces
UDP_PORT = 7777

class udp_pub(Node):
    def __init__(self):
        super().__init__('imu_publisher')
        self.publisher_=self.create_publisher(Imu,'/imu_array',10)
        self.timer_=self.create_timer(0.001,self.receive_data)
        self.start_time = time.time()
        self.packet_count = 0
        self.last_packet_time = self.start_time
        
    def receive_data(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))


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
            print(f"Packet received #{self.packet_count}, Time since last packet: {time_diff:.6f}s")
            # print(addr)
            new_data=Imu()
            new_data.header.stamp= self.get_clock().now().to_msg()
            new_data.header.frame_id="imu"
            ori=quaternion_from_euler(values[0],values[1],values[2])
            new_data.orientation.x=ori[0]
            new_data.orientation.y=ori[1]
            new_data.orientation.z=ori[2]
            new_data.orientation.w=ori[3]
            
            new_data.angular_velocity.x=values[6]
            new_data.angular_velocity.y=values[7]
            new_data.angular_velocity.z=values[8]
            
            new_data.linear_acceleration.x=values[3]
            new_data.linear_acceleration.y=values[4]
            new_data.linear_acceleration.z=values[5]
            
            self.publisher_.publish(new_data)

        
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
    
