#!/usr/bin/env python3

import sys
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


# Topics used
SCAN_TOPIC = '/scan'
DRIVE_TOPIC = '/drive'

"""
Ce code présente un algorithme de suivit de ligne médianne avec un noeud Wall_Follow
"""
class WallFollow(Node):
    def __init__(self):
        super().__init__('wall_follow')
        
        # Paramètres
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', 0.5), # Proportional gain of the PID controller on the distance error to the centerline
                ('ki', 0.05), # Integral gain of the PID controller on the distance error to the centerline
                ('kd', 0.0), # Derivative gain of the PID controller on the distance error to the centerline
                ('timestep', 0.025), # s, Timestep of the PID controller
                ('max_vel', 1.5), # m/s, Max car speed
                ('max_angle', 24.0), # degres, Max steering angle
                ('speed_reduction_factor', 0.6), # Speed reduction factor when the car is far from the centerline
                ('desired_distance', 0.0),
            ])

        # Constants
        self.LIDAR_FOV = 4.71238898038
        self.LIDAR_POINTS = 1081
        self.LIDAR_RESOLUTION = self.LIDAR_FOV/self.LIDAR_POINTS # radians
        self.THETA_CHECK = np.deg2rad([70, 65, 60, 55, 50, 45, 40, 35, 30, 20, 10, 5])
        self.KL = 1.2 ## Proportionality factor for the additional length
        self.CAR_LENGTH = 0.50 # Traxxas Rally is 20 inches or 0.5 meters

        # Wall_Follow Params
        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value
        self.timestep =  self.get_parameter('timestep').value
        self.max_vel = self.get_parameter('max_vel').value
        self.max_angle = np.radians(self.get_parameter('max_angle').value)  
        self.speed_reduction_factor = self.get_parameter('speed_reduction_factor').value
        self.desired_distance = self.get_parameter('desired_distance').value

        # Integral and derivative errors are initialized to zero
        self.error_integral = 0.0
        self.error_prev = 0.0

        # Publishers and subscribers
        self.lidar_sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)

    def get_error(self, data):
        """prend la donnée lidar et calcule l'erreur par rapport à la distance que l'on souhaite par rapport au milieu 

        Args:
            data : donnée lidar

        Returns:
            error : erreur de distance par rapport au fait de rester au milieu
        """
        distances_left = list()
        distances_right = list()

        look_perpendicular = np.deg2rad(45)
        b = data.ranges[int((self.LIDAR_FOV - look_perpendicular) / self.LIDAR_RESOLUTION)]
        b_right = data.ranges[int(look_perpendicular / self.LIDAR_RESOLUTION)]

        for theta in self.THETA_CHECK:
            a = data.ranges[int((self.LIDAR_FOV - look_perpendicular - theta) / self.LIDAR_RESOLUTION)]
            alpha = math.atan((a * math.cos(theta) - b) / (a * math.sin(theta)))
            Dt = b * math.cos(alpha)
            distances_left.append(Dt)

            a_right = data.ranges[int((look_perpendicular + theta) / self.LIDAR_RESOLUTION)]
            alpha_right = math.atan((a_right * math.cos(theta) - b_right) / (a_right * math.sin(theta)))
            Dt_right = b_right * math.cos(alpha_right)
            distances_right.append(Dt_right)

        average_distance = np.mean(np.array(distances_left) - np.array(distances_right))
        L = self.KL * self.CAR_LENGTH
        return average_distance + L * math.sin(alpha) - self.desired_distance

    def pid_control(self, error):
        """contrôleur PID sur l'erreur de distance. Publie une vitesse et un angle.

        Args:
            error : l'erreur entre la distance au mur réelle et celle que l'on veut
        """
        angle = 0.0
        self.error_integral = self.error_integral + self.timestep * (error + self.error_prev) / 2
        angle = self.kp * error + self.ki * self.error_integral * self.timestep + self.kd * (
                error - self.error_prev) / self.timestep
        angle = np.clip(angle, -self.max_angle, self.max_angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        # interpolation entre les vitesses par rapport aux angle pour obtenir la vitesse coresspondant à notre angle
        xp = np.array([0, np.deg2rad(5), np.deg2rad(20), np.deg2rad(25)])
        yp = np.array([self.max_vel, self.max_vel, self.max_vel * self.speed_reduction_factor, self.max_vel * self.speed_reduction_factor])
        velocity = np.interp(angle, xp, yp)
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def lidar_callback(self, data):
        """callback du lidar à chaque réception de donnée 

        Args:
            data : lidar message
        """
        error = self.get_error(data)
        self.pid_control(error)


def main(args=None):
    rclpy.init(args=args)
    wf = WallFollow()
    rclpy.spin(wf)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

