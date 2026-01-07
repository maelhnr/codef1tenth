import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool


EMERGENCY_TOPIC: str = '/emergency_breaking'
WALL_FOLLOW_SPEED_TOPIC: str = '/wall_follow_speed'
DISPARITY_SPEED_TOPIC: str = '/disparity_speed'
DRIVE_TOPIC: str = '/drive'


class Multiplexer(Node):
    def __init__(self):
        super().__init__('multiplexer')
        self.get_logger().info('Multiplexer started ...')
        
        self.aeb_condition = False
        self.aeb_permanent_condition = False
        
        self.multiplex_speed = 0.0
        
        # Subscribers:
        self.aeb_bool_sub = self.create_subscription(
            Bool,
            EMERGENCY_TOPIC,
            self.aeb_bool_sub_callback,
            255
            )
        
        self.wall_follow_speed_sub = self.create_subscription(
            AckermannDriveStamped,
            WALL_FOLLOW_SPEED_TOPIC,
            self.wall_follow_speed_sub_callback,
            1
            )
        
        self.disparity_speed_sub = self.create_subscription(
            AckermannDriveStamped,
            DISPARITY_SPEED_TOPIC,
            self.disparity_speed_sub_callback,
            1
            )
        
        # Publisher to the '/drive' topic:
        self.multiplex_drive_publisher = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 1000)
        
    def aeb_bool_sub_callback(self, aeb_bool_msg):
        self.aeb_condition = aeb_bool_msg.data
        
    def wall_follow_speed_sub_callback(self, wall_follow_speed_msg):
        if self.aeb_condition == True:
            self.aeb_permanent_condition = True
        
        if self.aeb_permanent_condition == False:
            self.multiplex_speed = wall_follow_speed_msg.drive.speed
        else:
            self.multiplex_speed = 0.0 
        
            
        drive_msg_to_publish = AckermannDriveStamped()
        drive_msg_to_publish.drive.speed = self.multiplex_speed
        self.multiplex_drive_publisher.publish(drive_msg_to_publish)
        
    def disparity_speed_sub_callback(self, disparity_speed_msg):
        drive_msg_to_publish = AckermannDriveStamped()
        if self.aeb_condition == True:
            self.aeb_permanent_condition = True
            
        if self.aeb_permanent_condition == False:
            drive_msg_to_publish = disparity_speed_msg
        else:
            self.multiplex_speed = 0.0
            drive_msg_to_publish.drive.speed = self.multiplex_speed
            
        self.multiplex_drive_publisher.publish(drive_msg_to_publish)

def main(args=None):
    rclpy.init(args=args)
    node = Multiplexer()
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
    
