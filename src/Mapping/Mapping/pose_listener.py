import rclpy
from tf2_msgs import msg
from geometry_msgs.msg import Vector3
"""
    @author Vladimir Célaudoux
    
    This code implements a node that listen to tf2_msgs from the /tf topic, translate them into position in the /map frame and publish them on a /pose_tf topic
    
"""
class RobotPoseListener:
    """
    A class to represent the tf listener node
    --------
    Attributes :
    
    node : node
        The node 
    tf_subscriber : subsciber
        The subscriber of this node
    publisher : publisher 
        The publisher of this node
    ---------  
    Methods :
    
    tf_callback(msg)
        callback when a message is received from the subscriber 
    run 
        function to execute the node after initialization     
    
    """
    def __init__(self):        
        rclpy.init()
        self.node = rclpy.create_node('pose_listener')

        self.tf_subscriber = self.node.create_subscription(
            msg.TFMessage,
            '/tf',
            self.tf_callback,
            10)
        self.publisher=self.node.create_publisher(Vector3,"pose_tf",10)

    def tf_callback(self, msg):
        """ When receiving a message, get the transform in the good frame, transform it in the good position and publish it

        Args:
            msg (tf2_msgs.msg.TFMessage): the message from the subscriber
        """
        for transform_stamped in msg.transforms:
            if transform_stamped.header.frame_id=="map" :
                # Extract position information from TransformStamped
                position = transform_stamped.transform.translation 
                
                self.publisher.publish(position)
                print(f"Frame: {transform_stamped.header.frame_id}")
                print(f"Position: x={position.x}, y={position.y}, z={position.z}")

    def run(self):
        """
        Run the node 
        """
        rclpy.spin(self.node)
        rclpy.shutdown()


listener = RobotPoseListener()
listener.run()
