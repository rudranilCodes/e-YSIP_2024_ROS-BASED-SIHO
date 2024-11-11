#!/usr/bin/env python3
"""
Author: Rudranil Bose
Project: ROS-Based SLAM Implementation and Hardware Optimization for Small Differential Drive Bot
eYSIP 2024
This code creates a ROS 2 node to run an Extended Kalman Filter to fuse IMU and encoder odometry,
using a slip-compensated technique derived from kinematics modeling of a skid-steer drive robot.


**The velocity of the robot mass center is denoted by vG
    vLx := v1x = v2x = vGx - W/2 . ψ'˙ 
    vRx := v3x = v4x = vGx + W/2 . ψ'

** For the IMU located at (xM , yM , 0) in B, we obtain the IMU
   velocity vB = [vBx vBy vBz ]T in B as:
    vBx = vGx - yM ψ'
    vBy = vGy + xM ψ'

** We define the longitudinal wheel slip λi at each wheel as the
   ratio of the wheel velocity and its center velocity, namely
    λi := (rωi - vix)/rωi = -∆vix/rωi, i = 1,..., 4
   
   where r is the effective wheel radius, and ∆vix := vix - rωi.
   For left- and right-side's wheels, the wheel slips satisfy
    λL := λ1 = λ2 , λR := λ3 = λ4

** Let ICRl, ICRr , and ICRG denote the instantaneous center
   of rotation (ICR) of the left-side wheel contact points, rightside wheel contact points, and the robot body, respectively. It is
   known that ICRl, ICRr , and ICRG lie on a line parallel to the
   y-axis. We denote the coordinates for ICRl, ICRr ,
   and ICRG in B as (xlc , ylc , 0), (xrc , yrc , 0), and (xGc , yGc , 0),
   respectively.    

** We define the longitudinal ICR location, which
   is denoted as S, as the x-coordinate of the collinear ICRl, ICRr ,
   and ICRG in frame B.   

** The longitudinal ICR location S (see Fig. 2) satisfies the following constraints:

    S = xlc = xrc = xGc = -vGy.ψ'˙ .

** We write the longitudinal slip velocities of the wheel/ground
   contact points as
    ∆v1x = ∆v2x = ylc - W/2.ψ'
    ∆v3x = ∆v4x = yrc + W/2.ψ'

---------------------------------------------------------------------------------------------------------------------------------------------------
---------------------------------------------------- IMU Kinematic Motion Equations----------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------
** Let PI (t)=[XI(t) YI(t) ZI(t)]T ∈ R3 and 
   VI(t) =[Vx(t) Vy(t) Vz(t)]T ∈ R3 
   denote the position and velocity
   vectors of the IMU in I, respectively.

** Calculate angular velocities

    phi_dot = omega_Bx + np.sin(phi) * np.tan(theta) * omega_By + np.cos(phi) * np.tan(theta) * omega_Bz
    theta_dot = np.cos(phi) * omega_By - np.sin(phi) * omega_Bz
    psi_dot = (np.sin(phi) / np.cos(theta)) * omega_By + (np.cos(phi) / np.cos(theta)) * omega_Bz

---------------------------------------------------------------------------------------------------------------------------------------------------
------------------------------------------------------------ EKF Design ---------------------------------------------------------------------------
---------------------------------------------------------------------------------------------------------------------------------------------------

X'(k|k - 1) = X'(k - 1|k - 1) + ∆Tf(X'(k - 1|k - 1), u(k - 1)) 
P(k|k - 1) = F(k)P(k|k - 1).F^T(k) + Q(k)
X'(k|k) = X'(k|k - 1) + W(k)[y(k) - H(k) X's(k|k - 1)] 
W(k) = P(k|k - 1).H^T(k).S-1(k)
S(k) = H(k)P(k|k - 1)HT (k) + R 
P(k|k) = (I9 - W(k).H(k)).P(k|k - 1) * (I9 - W(k).H(k))^T + W(k).R.W^T(k)

"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math
import numpy as np
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion


class ekf(Node):
    def __init__(self):
        super().__init__('EKF_NODE')

        self.lw_vel = 0.0
        self.rw_vel = 0.0
        
        self.lw_omega = 0.0
        self.rw_omega = 0.0

        self.omega_B = np.zeros(3) #initialising angular velocity from IMU gyro
        self.a_B = np.zeros(3)     #initialising acceleration from IMU accmtr
        self.V_B = np.zeros(3)     #initialising IMU linear velocity in body frame
        self.v_Bz = 0.0            #velocity of IMU in z
        self.x_M = 0.0             #x coordindate of IMU sensor in body frame
        self.y_M = 0.0             #y coordindate of IMU sensor in body frame

        self.radius = 0.040 
        self.wheel_base = 0.136

        #state space
        self.X = np.zeros(9)
        #gravity vector
        self.G = np.array([0.0, 0.0, -9.81])

        self.dt = 1.0

        # Calculate angular velocities
        self.phi_dot = 0.0
        self.theta_dot = 0.0
        self.psi_dot = 0.1

        self.state_dim = 9         # Dimension of state vector [P_I, V_I, Theta]
                                   # Dimension of measurement vector [vB_x, vB_y, vB_z]

        self.u = np.zeros(6)

        self.P = np.zeros((self.state_dim, self.state_dim)) # State covariance matrix
        self.P[0:3, 0:3] = np.diag([1, 1, 1]) 
        self.P[3:6, 3:6] = np.diag([1, 1, 1])        
        self.P[6:9, 6:9] = np.diag([1, 1, 1])            
        self.P = self.P.astype(np.float64)
        # Process noise covariance Q
        self.Q = np.zeros((self.state_dim, self.state_dim))
        self.Q[0:3, 0:3] = np.diag([1e-2, 1e-2, 1e-2])        # Least confidence in position
        self.Q[3:6, 3:6] = np.diag([1e-2, 1e-2, 1e-2])        # Very good confidence in velocity
        self.Q[6:9, 6:9] = np.diag([1e-2, 1e-2, 1e-2])        # Below average confidence in orientation
        self.Q = self.Q.astype(np.float64)

        self.R = np.diag([1e-1, 1e-1, 1e-1])                  # Measurement noise covariance
        self.enc_odom_sub = self.create_subscription(Odometry, "/odom", self.odom_enc_callback, 10)
        self.imu_data = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.odom_filtered = self.create_publisher(Odometry, "/odom_filtered", 10)

        self.previous_time_step = 0.0
        self.current_time_step = 0.0
        self.euler_angles = np.zeros(3)

        self.odometry = Odometry()

    def update_time(self):
        current_time = self.get_clock().now().nanoseconds / 1e9                   # Current time in seconds
        if self.previous_time_step != 0.0:                                        # Ensure this is not the first call
            self.dt = current_time - self.previous_time_step                      # Calculate elapsed time
        self.previous_time_step = current_time                                    # Update previous time
        print(current_time)

    def predict(self, u, euler_angles, dt):
            
            # X(k|k-1) = X(k-1|k-1) + ΔT * f(X(k-1|k-1), u(k-1))
            self.X = self.state_transition(self.X,  u, euler_angles, dt)
 
            # P(k|k-1) = F(k) * P(k-1|k-1) * F^T(k) + Q(k)
            F = self.jacobian_F(self.X, u)
            F = F.astype(np.float64)
            self.P = F @ self.P @ F.T + self.Q

    def update(self, z):

            H = self.jacobian_H(self.X)
            
            # S(k) = H(k) * P(k|k-1) * H^T(k) + R
            S = H @ self.P @ H.T + self.R
            # W(k) = P(k|k-1) * H^T(k) * S^(-1)(k)
            K = self.P @ H.T @ np.linalg.inv(S)
            
            # X(k|k) = X(k|k-1) + W(k) * [y(k) - H(k) * X(k|k-1)]
            y = z - H @ self.X
            self.X = self.X + K @ y
            # P(k|k) = (I - W(k) * H(k)) * P(k|k-1) * (I - W(k) * H(k))^T + W(k) * R * W^T(k)
            I = np.eye(self.state_dim)
            self.P = ((I - K @ H) @ self.P @ (I - K @ H).T) + K @ self.R @ K.T
            self.publish_odometry()

    
    def transformation_matrix(self, Theta):

        roll, pitch, yaw = Theta
        c, s = np.cos, np.sin

        R = np.array([
            [c(yaw) * c(pitch), c(yaw) * s(pitch) * s(roll) - s(yaw) * c(roll), c(yaw) * s(pitch) * c(roll) + s(yaw) * s(roll)],
            [s(yaw) * c(pitch), s(yaw) * s(pitch) * s(roll) + c(yaw) * c(roll), s(yaw) * s(pitch) * c(roll) - c(yaw) * s(roll)],
            [-s(pitch), c(pitch) * s(roll), c(pitch) * c(roll)]])
                
        return R
    


    def state_transition(self, X, u, euler, dt):

        self.dt = dt
        P_I, V_I, Theta = X[:3], X[3:6], X[6:]
        A_B, omega_B = u[:3], u[3:]
        C_BI = self.transformation_matrix(euler)
        f_P = V_I
        f_V = C_BI @ A_B + self.G
        f_Theta = np.array([
            omega_B[0] + np.tan(euler[1]) * (np.sin(euler[0]) * omega_B[1] + np.cos(euler[0]) * omega_B[2]),
            np.cos(euler[0]) * omega_B[1] - np.sin(euler[0]) * omega_B[2],
            (np.sin(euler[0]) / np.cos(euler[1])) * omega_B[1] + (np.cos(euler[0]) / np.cos(euler[1])) * omega_B[2]
        ])
        return np.concatenate((P_I + self.dt * f_P, V_I + self.dt * f_V, Theta + self.dt * f_Theta))



    def jacobian_F(self, X, u):

        Theta = X[6:9]
        A_B, omega_B = u[:3], u[3:]
        C_BI = self.transformation_matrix(Theta)
        f_V = C_BI @ A_B + self.G

        F_V = self.jacobian_F_V(Theta, A_B, f_V)
        F_Theta = self.jacobian_F_Theta(Theta)

        F = np.block([
            [np.zeros((3,3)), np.eye(3), np.zeros((3,3))],
            [np.zeros((3,3)), np.zeros((3,3)), F_V],
            [np.zeros((3,3)), np.zeros((3,3)), F_Theta]
        ])
        #print(F)
        return F



    def jacobian_F_V(self, Theta, A_B, f_V):
            
            roll, pitch, yaw = Theta
            C_IB = self.transformation_matrix(Theta)
            aB_x, aB_y, aB_z = A_B

            #extract V_I_dot components here
            V_x_dot, V_y_dot, V_z_dot = f_V

            g = 9.81

            s, c, t = np.sin, np.cos, np.tan

            F_V = np.array([
                [(C_IB[0, 2] * aB_y - C_IB[0, 1] * aB_z), c(yaw) * (V_z_dot + g), - V_y_dot],
                [(C_IB[1, 2] * aB_y - C_IB[1, 1] * aB_z), s(yaw) * (V_z_dot + g), V_x_dot],
                [(C_IB[2, 2] * aB_y - C_IB[2, 1] * aB_z), ((- c(pitch)) * aB_x ) - s((pitch) * s(roll) * aB_y) - (c(roll) * s(pitch) * aB_z), 0]
            ])

            return F_V


    def jacobian_F_Theta(self, Theta):
        phi, theta, psi = Theta
        
        # Construct the Jacobian matrix F_Theta
        F_Theta = np.array([
            [self.theta_dot * np.tan(theta), self.psi_dot / np.cos(theta), 0],
            [-self.psi_dot * np.cos(theta), 0, 0],
            [self.theta_dot / np.cos(theta), self.psi_dot * np.tan(theta), 0]
        ])
        
        return F_Theta
    

    def f_Theta(self, Theta, omega_B):

        roll, pitch, yaw = Theta
        s, c, t = np.sin, np.cos, np.tan

        f_Theta = np.array([
            omega_B[0] + t(pitch) * (s(roll) * omega_B[1] + c(roll) * omega_B[2]),
            c(roll) * omega_B[1] - s(roll) * omega_B[2],
            s(roll) / c(pitch) * omega_B[1] + c(roll) / c(pitch) * omega_B[2]
        ])

        return f_Theta
    
    
    def jacobian_H(self, X):
        Theta = X[6:9]
        V_I = X[3:6]
        V_x, V_y, V_z = V_I

        # Compute the rotation matrix C_I_B and its transpose
        C_I_B = self.transformation_matrix(Theta)
        C_I_B_T = C_I_B.T

        # Compute H_V(k)
        H_V_k = C_I_B_T

        # Compute H_Theta(k)
        H_Theta_k = np.array([
            [0, (-np.sin(Theta[1]) * np.cos(Theta[2]) * V_x ) - (np.sin(Theta[1]) * np.sin(Theta[2]) * V_y) - (np.cos(Theta[1]) * V_z), C_I_B_T[0,0] * V_y - C_I_B[1,0] * V_x],
            [V_z * np.sin(Theta[0]) * V_x, C_I_B[0,1] * V_y - C_I_B[1,1] * V_x, -V_y * np.cos(Theta[0]) * V_x],
            [C_I_B[0,2] * V_y - C_I_B[1,2] * V_x, 0, 0]
        ])

        # Construct H(k)
        H_k = np.block([
            [np.zeros((3, 3)), H_V_k, H_Theta_k]
        ])

        return H_k



    def odom_enc_callback(self, msg):

        self.lw_vel =  (msg.twist.twist.linear.x - (self.wheel_base / 2)) * self.psi_dot
        self.rw_vel =  (msg.twist.twist.linear.x + (self.wheel_base / 2)) * self.psi_dot

        self.lw_omega = self.lw_vel / self.radius
        self.rw_omega = self.rw_vel / self.radius

        self.λl = (self.lw_omega - self.lw_vel) / (self.lw_omega)
        self.λr = (self.rw_omega - self.rw_vel) / (self.rw_omega)

        if self.lw_omega >= self.rw_omega:
            lambda_prime_L = max(self.λl, 0)
            lambda_prime_R = max(self.λr, 0)
        else:
            lambda_prime_L = min(self.λl, 0)
            lambda_prime_R = min(self.λr, 0)     
        
        S = - (msg.twist.twist.linear.y) / self.psi_dot

        v_Bx = ((self.radius / 2) * ((self.lw_omega + self.rw_omega) - (lambda_prime_L * self.lw_omega + lambda_prime_R * self.rw_omega))) - self.y_M * self.psi_dot
        v_By = (self.x_M - S) * self.psi_dot

        self.V_B = np.array([v_Bx, v_By, self.v_Bz])
       
        #update step
        self.update(self.V_B) 


    def imu_callback(self, msg):
            
        self.update_time()  #Update dt and prev_time

        self.a_B = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
        self.omega_B = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])

        self.euler_angles = euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        
        self.phi_dot = self.omega_B[0] + np.sin(self.euler_angles[0]) * np.tan(self.euler_angles[1]) * self.omega_B[1] + np.cos(self.euler_angles[0]) * np.tan(self.euler_angles[1]) * self.omega_B[2]
        self.theta_dot = np.cos(self.euler_angles[0]) * self.omega_B[1] - np.sin(self.euler_angles[0]) * self.omega_B[2]
        self.psi_dot = (np.sin(self.euler_angles[0]) / np.cos(self.euler_angles[1])) * self.omega_B[1] + (np.cos(self.euler_angles[0]) / np.cos(self.euler_angles[1])) * self.omega_B[2] 

        self.u = np.array([self.a_B[0], self.a_B[1], self.a_B[2] , self.omega_B[0], self.omega_B[1], self.omega_B[2]])
       
        #prediction step
        self.predict(self.u, self.euler_angles, self.dt)  

    def publish_odometry(self):

        cos_phi_2 = np.cos(self.X[6] / 2)
        sin_phi_2 = np.sin(self.X[6] / 2)
        cos_theta_2 = np.cos(self.X[7] / 2)
        sin_theta_2 = np.sin(self.X[7] / 2)
        cos_psi_2 = np.cos(self.X[8] / 2)
        sin_psi_2 = np.sin(self.X[8] / 2)

        q_x = cos_phi_2 * cos_theta_2 * cos_psi_2 + sin_phi_2 * sin_theta_2 * sin_psi_2
        q_y = sin_phi_2 * cos_theta_2 * cos_psi_2 - cos_phi_2 * sin_theta_2 * sin_psi_2
        q_z = cos_phi_2 * sin_theta_2 * cos_psi_2 + sin_phi_2 * cos_theta_2 * sin_psi_2
        q_w = cos_phi_2 * cos_theta_2 * sin_psi_2 - sin_phi_2 * sin_theta_2 * cos_psi_2

        self.odometry.header.frame_id = 'odom_filtered'
        self.odometry.child_frame_id = 'base_link'
        self.odometry.header.stamp = self.get_clock().now().to_msg()
        self.odometry.pose.pose.position.x = self.X[0]
        self.odometry.pose.pose.position.y = self.X[1]
        self.odometry.pose.pose.position.z = self.X[2]
        self.odometry.twist.twist.linear.x = self.X[3]

        self.odometry.pose.pose.orientation.x = q_x
        self.odometry.pose.pose.orientation.y = q_y
        self.odometry.pose.pose.orientation.z = q_z
        self.odometry.pose.pose.orientation.w = q_w

        # Publish the Odometry message
        self.odom_filtered.publish(self.odometry)        


def main(args=None):
    rclpy.init(args=args)
    ekf_node = ekf()
    rclpy.spin(ekf_node)
    ekf_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
         


