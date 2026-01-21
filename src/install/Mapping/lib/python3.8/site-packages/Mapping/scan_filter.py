import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import LaserScan


class ScanFilter(Node):
    """
    A class that implements a LaserScan filter that removes all of the points
    that are not in front of the robot.
    """
    def __init__(self):
        super().__init__('scan_filter__node')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('side_filter', 0.13),
                ('front_filter', 0.20),
                ('back_filter', 0.10),
            ]
        )

        # Variables
        self.side_filter = self.get_parameter('side_filter').get_parameter_value().double_value
        self.front_filter = self.get_parameter('front_filter').get_parameter_value().double_value
        self.back_filter = self.get_parameter('back_filter').get_parameter_value().double_value

        # Subscribers and publishers
        self.sub = self.create_subscription(LaserScan, 'scan_raw', self.callback, 10)
        self.pub = self.create_publisher(LaserScan, 'scan', 10)

    def callback(self, msg: LaserScan):
        """
        Callback function to deal with incoming LaserScan messages.
        :param self: The self reference.
        :param msg: The subscribed LaserScan message.

        :publishes msg: updated LaserScan message.
        """

        # Based on the ranges, compute the (x, y) position of each impact point
        ranges = np.array(msg.ranges)
        ranges[:30] = np.inf
        ranges[-30:] = np.inf
        angles = np.arange(len(ranges)) * msg.angle_increment + msg.angle_min
        x_pos = ranges * np.sin(angles)
        y_pos = ranges * np.cos(angles)

        # Filter the points within the car's bounding box
        side_mask = np.abs(x_pos) < self.side_filter
        front_back_mask = (-self.back_filter < y_pos) & (y_pos < self.front_filter)
        invalid_points_mask = side_mask & front_back_mask

        # Update the ranges
        ranges[invalid_points_mask] = np.inf
        msg.ranges = ranges.tolist()

        # Publish the updated message
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ScanFilter()
    rclpy.spin(node)
    node.occupancy_grid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
