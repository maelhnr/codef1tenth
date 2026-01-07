#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import atexit
import tf2_ros
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry

class WaypointsLogger(Node):
    def __init__(self):
        super().__init__('waypoints_logger')

        # Initialize tf2 listener
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Create a CSV file to log waypoints
        self.filename = 'src/f1_tenth_2023_2024/Simple_algo/Simple_algo/waypoints.csv'  # Replace with your desired path
        self.file = open(self.filename, 'w')

        # Write the login timestamp to the file
        self.file.write("Login at time:" + str(self.get_clock().now()) + "\n")

        # Subscribe to Odometry messages
        self.create_subscription(Odometry, '/odom', self.save_waypoint, 10)

    def save_waypoint(self, msg):
        try:
            # Transform the pose to the odom frame
            transform = self.tfBuffer.lookup_transform(
                'odom', msg.header.frame_id, msg.header.stamp)
            pose_transformed = tf2_ros.transformations.do_transform_pose(
                Pose(), transform)

            # Extract relevant information
            position = pose_transformed.position
            orientation = pose_transformed.orientation
            speed = msg.twist.twist.linear.x

            # Write the waypoint information to the CSV file
            self.file.write(
                f'{position.x}, {position.y}, {position.z}, {orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}, {speed}\n')

        except tf2_ros.LookupException as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')
        except tf2_ros.ExtrapolationException as e:
            self.get_logger().warn(f'Transform extrapolation failed: {e}')

    def shutdown(self):
        self.file.close()
        self.get_logger().info('Goodbye')


def main(args=None):
    rclpy.init(args=args)
    waypoints_logger = WaypointsLogger()
    atexit.register(waypoints_logger.shutdown)
    rclpy.spin(waypoints_logger)
    waypoints_logger.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

