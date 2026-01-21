#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped


# Topics used
SCAN_TOPIC = '/scan'
DRIVE_TOPIC = '/drive'

# Constants
ANGLE_RANGE = 270  # Hokuyo 10LX has 270 degrees scan
CAR_LENGTH = 0.50  # Traxxas Rally is 20 inches or 0.5 meters

"""
Ce code présente un noeud de suivi de mur gauche
"""
class WallFollow(Node):
    """ Implement Wall Following on the car
    """

    def __init__(self):
        super().__init__('wall_follow_penn')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('kp', -20.0),  # Proportional gain of the PID controller on the distance error
                ('kd', 8.0),  # Derivative gain of the PID controller on the distance error
                ('ki', 0.015),  # Integral gain of the PID controller on the distance error
                ('desired_distance_left', 0.55),  # meters, Desired distance from the left wall
                ('max_speed', 1.5),  # m/s, Max car speed
            ])

        self.kp = self.get_parameter('kp').value
        self.kd = self.get_parameter('kd').value
        self.ki = self.get_parameter('ki').value
        self.desired_distance_left = self.get_parameter('desired_distance_left').value
        self.max_speed = self.get_parameter('max_speed').value

        # Variables
        self.prev_error = 0.0
        self.integral = 0.0

        # Publishers and Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)

    def getRange(self, data, angle):
        # data: single message from topic /scan
        # angle: between -45 to 225 degrees, where 0 degrees is directly to the right
        # Outputs length in meters to object with angle in lidar scan field of view
        # make sure to take care of nans etc.
        ranges = np.array(data.ranges)
        angle_incr = data.angle_increment
        desired_idx = int(
            (np.radians(angle) - data.angle_min) / angle_incr)
        if np.isfinite(ranges[desired_idx]):
            return ranges[desired_idx]
        else:
            return None

    def pid_control(self, error):
        """PID contrôleur par rapport à l'erreur

        Args:
            error : erreur entre la distance voulue et celle réelle par rapport au mur gauche
        """
        deriv_error = error - self.prev_error
        self.prev_error = error
        self.integral += error

        angle = np.radians(self.kp * error + self.kd * deriv_error + self.ki * self.integral)
        velocity = self.calc_speed(angle)
        self.get_logger().info("Error {}, Angle {}".format(error, np.degrees(angle)))

        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = velocity
        self.drive_pub.publish(drive_msg)

    def calc_speed(self, angle):
        """calcul la vitesse en fonction de l'angle

        Args:
            angle : angle voulu jusqu'au point que l'on vise

        Returns:
            speed : vitesse de commande 
        """
        angle = np.abs(np.degrees(angle))
        if 0 <= angle < 10:
            speed = self.max_speed
        elif 10 <= angle < 20:
            speed = self.max_speed * 0.66
        else:
            speed = self.max_speed * 0.33
        return speed

    def followLeft(self, data):
        """prend les données lidar et retourne l'erreur entre la distance réelle et celle que l'on veut (par rapport au mur gauche)

        Args:
            data : données lidar
        Returns:
            error: l'erreur par rapport à la distance voulue
        """
        zero_angle = 90
        b = self.getRange(data, zero_angle)

        theta = 40
        a = self.getRange(data, zero_angle - theta)
        theta = np.radians(theta)
        if b is not None and a is not None:
            alpha = np.arctan2(a * np.cos(theta) - b, a * np.sin(theta))
            Dleft = b * np.cos(alpha)

            D_left_lookahead = Dleft + CAR_LENGTH * np.sin(alpha)

            error = self.desired_distance_left - D_left_lookahead

            return error
        else:
            return None

    def lidar_callback(self, msg):
        """fonction de callback quand on reçoit des messages lidar

        Args:
            msg : un message du lidar
        """
        error = self.followLeft(msg)
        if error is not None:
            self.pid_control(error)


def main(args=None):
    rclpy.init(args=args)
    wf = WallFollow()
    rclpy.spin(wf)
    wf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

