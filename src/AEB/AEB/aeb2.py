import rclpy
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool


###### VERSION AMELIOREE DE L'AEB - NECESSITE L'UTILISATION DU MULTIPLEXEUR ######
ODOM_TOPIC: str = '/odom'
EMERGENCY_TOPIC: str = '/emergency_breaking'
SCAN_TOPIC: str = '/scan'


class AEB2(Node):
    def __init__(self):
        super().__init__('aeb2')
        self.get_logger().info('AEB2 has been started.')

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('threshold_side', 0.15),
                ('threshold_front', 0.40),
                ('angle_threshold', np.radians(10)),
            ]
        )
        
        # Variables
        self.speed = 0.0
        self.emergency_breaking = False
        
        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry,
            ODOM_TOPIC,
            self.odom_callback,
            10
            )
        
        self.lidar_sub = self.create_subscription(
            LaserScan, 
            SCAN_TOPIC, 
            self.lidar_callback, 
            10
            )
        
        # Publisher
        self.bool_publisher = self.create_publisher(
            Bool, 
            EMERGENCY_TOPIC, 
            10
            )
        
    def odom_callback(self, odom_msg: Odometry):
        self.speed = odom_msg.twist.twist.linear.x

    def lidar_callback(self, scan_msg: LaserScan):
        for idx, r in enumerate(scan_msg.ranges):
            # Check if the scan is in range or not NaN
            if (np.isnan(r) or r > scan_msg.range_max or r < scan_msg.range_min): continue

            # Compute the angle between the lidar point and the car
            scan_angle = scan_msg.angle_min + idx * scan_msg.angle_increment

            # Apply a different threshold depending on the angle
            if abs(scan_angle) < self.get_parameter('angle_threshold').value: 
                threshold = self.get_parameter('threshold_front').value
            else:
                threshold = self.get_parameter('threshold_side').value

            # Compute the time to collision
            ttc = r / max(self.speed*np.cos(scan_angle), 0.0000001)
            if ttc < threshold:
                self.get_logger().error(f'Emergency breaking with ttc: {ttc:.2f} < {threshold} (scan_angle={np.degrees(scan_angle):.2f}°)')
                self.emergency_breaking = True
                break

        emergency_msg = Bool()
        emergency_msg.data = self.emergency_breaking
        self.bool_publisher.publish(emergency_msg)
        

def main(args=None):
    rclpy.init(args=args)
    node = AEB2()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
