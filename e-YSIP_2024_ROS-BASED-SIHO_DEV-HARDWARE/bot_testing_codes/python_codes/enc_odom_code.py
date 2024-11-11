import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster
from std_msgs.msg import Int32
import math

class SkidSteeringOdomNode(Node):

    def __init__(self):
        super().__init__('skid_steering_odom_node')
        
        self.wheel_base = 0.5  # distance between the wheels
        self.wheel_radius = 0.075  # radius of the wheels
        self.ticks_per_revolution = 3000  # encoder ticks per wheel revolution
        self.slip_factor = 1.0  # slip factor (adjust based on your robot)

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        self.current_front_left_ticks = 0
        self.current_front_right_ticks = 0
        self.current_rear_left_ticks = 0
        self.current_rear_right_ticks = 0
        
        self.prev_front_left_ticks = 0
        self.prev_front_right_ticks = 0
        self.prev_rear_left_ticks = 0
        self.prev_rear_right_ticks = 0

        # Subscriptions
        self.front_left_tick_sub = self.create_subscription(Int32,'front_left_ticks',self.front_left_tick_callback,10)
        self.front_right_tick_sub = self.create_subscription(Int32,'front_right_ticks',self.front_right_tick_callback,10)
        self.rear_left_tick_sub = self.create_subscription(Int32,'back_left_ticks',self.rear_left_tick_callback,10)
        self.rear_right_tick_sub = self.create_subscription(Int32,'back_right_ticks',self.rear_right_tick_callback,10)

        #timer function to update
        self.timer_=self.create_timer(0.01,self.update_odometry)
        
        # Publisher
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.prev_time = self.get_clock().now()

    def front_left_tick_callback(self, msg):
        self.current_front_left_ticks = msg.data

    def front_right_tick_callback(self, msg):
        self.current_front_right_ticks = msg.data

    def rear_left_tick_callback(self, msg):
        self.current_rear_left_ticks = msg.data

    def rear_right_tick_callback(self, msg):
        self.current_rear_right_ticks = msg.data

    def update_odometry(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        self.prev_time = current_time

        
        fl_ticks=self.current_front_left_ticks  - self.prev_front_left_ticks 
        fr_ticks=self.current_front_right_ticks - self.prev_front_right_ticks
        bl_ticks=self.current_rear_left_ticks   - self.prev_rear_left_ticks  
        br_ticks=self.current_rear_right_ticks  - self.prev_rear_right_ticks 

        self.prev_front_left_ticks  = self.current_front_left_ticks   
        self.prev_front_right_ticks = self.current_front_right_ticks  
        self.prev_rear_left_ticks   = self.current_rear_left_ticks    
        self.prev_rear_right_ticks  = self.current_rear_right_ticks   

        # if(fl_ticks >2,147,483,000):
        #     fl_ticks=0-(2,147,483,647-fl_ticks)
        # elif(fl_ticks <-2,147,483,000):
        #     fl_ticks=(2,147,483,647-fl_ticks)
        
        # if(fr_ticks >2,147,483,000):
        #     fr_ticks=0-(2,147,483,647-fr_ticks)
        # elif(fr_ticks <-2,147,483,000):
        #     fr_ticks=(2,147,483,647-fr_ticks)
        
        # if(bl_ticks >2,147,483,000):
        #     bl_ticks=0-(2,147,483,647-bl_ticks)
        # elif(bl_ticks <-2,147,483,000):
        #     bl_ticks=(2,147,483,647-bl_ticks)
            
        # if(br_ticks >2,147,483,000):
        #     br_ticks=0-(2,147,483,647-br_ticks)
        # elif(br_ticks <-2,147,483,000):
        #     br_ticks=(2,147,483,647-br_ticks)
        
        
        
        # Calculate wheel displacements
        front_left_distance  = 2 * math.pi * self.wheel_radius * (fl_ticks) / self.ticks_per_revolution
        front_right_distance = 2 * math.pi * self.wheel_radius * (fr_ticks)/ self.ticks_per_revolution
        rear_left_distance   = 2 * math.pi * self.wheel_radius * (bl_ticks)/ self.ticks_per_revolution
        rear_right_distance  = 2 * math.pi * self.wheel_radius * (br_ticks)/ self.ticks_per_revolution
        


        # Consider the slip factor
        # front_left_distance *= self.slip_factor
        # front_right_distance *= self.slip_factor
        # rear_left_distance *= self.slip_factor
        # rear_right_distance *= self.slip_factor

        # Calculate average distances
        left_distance = (front_left_distance + rear_left_distance) / 2.0
        right_distance = (front_right_distance + rear_right_distance) / 2.0

        # Calculate change in position and orientation
        d = (left_distance + right_distance) / 2.0
        theta_delta = (right_distance - left_distance) / self.wheel_base

        self.x += d * math.cos(self.theta)
        self.y += d * math.sin(self.theta)
        self.theta += theta_delta

        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # Publish odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        # Position
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0

        # Orientation
        q = self.quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]

        # Publish the odometry message
        self.odom_pub.publish(odom_msg)

        # Broadcast the transform over TF
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

    def quaternion_from_euler(self, roll, pitch, yaw):
        """
        Convert an Euler angle to a quaternion.
        """
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = SkidSteeringOdomNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

