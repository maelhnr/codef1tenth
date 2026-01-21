import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import numpy as np

class VirtualImuNode(Node):
    def __init__(self):
        super().__init__('virtual_imu_node')

        # Déclaration des paramètres (Modifications propres)
        self.declare_parameter('odom_topic', 'odom')
        self.declare_parameter('imu_topic', 'imu')
        self.declare_parameter('frame_id', 'base_link')
        self.declare_parameter('publish_frequency', 200.0)
    

        # Récupération des paramètres
        odom_topic = self.get_parameter('odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        freq = self.get_parameter('publish_frequency').value

        # Variables pour la dérivation
        self.prev_v_x = 0.0
        self.prev_v_y = 0.0
        self.prev_v_z = 0.0
        self.prev_time = self.get_clock().now()

        self.latest_odom = None
        
        # Timer de publication indépendant
        timer_period = 1.0 / freq
        self.timer = self.create_timer(timer_period, self.odom_callback)
        
        # Subscriber (ne fait plus de calcul, il stocke juste)
        self.odom_sub = self.create_subscription(Odometry, odom_topic, self.odom_store_callback, 10)

        # Publication de l'IMU
        self.imu_pub = self.create_publisher(Imu, imu_topic, 10)

        self.get_logger().info(f"Virtual IMU démarré : {odom_topic} -> {imu_topic}")

    def odom_store_callback(self, msg):
        # On stocke simplement la donnée dès qu'elle arrive
        self.latest_odom = msg

    def odom_callback(self):
        if self.latest_odom is None:
            return

        current_time = self.get_clock().now()

        # Calcul du dt
        dt = (current_time - self.prev_time).nanoseconds / 1e9
        if dt <= 0:
            return

        # Extraction des vitesses actuelles
        curr_v_x = self.latest_odom.twist.twist.linear.x
        curr_v_y = self.latest_odom.twist.twist.linear.y
        curr_v_z = self.latest_odom.twist.twist.linear.z
        curr_w_z = self.latest_odom.twist.twist.angular.z

        # Dérivation pour l'accélération
        acc_x = (curr_v_x - self.prev_v_x) / dt
        centripetal_accel = curr_v_x * curr_w_z
        acc_y = (curr_v_y - self.prev_v_y) / dt + centripetal_accel
        acc_z = (curr_v_z - self.prev_v_z) / dt + 9.81

        # Création du message IMU
        imu_msg = Imu()
        imu_msg.header.stamp = current_time.to_msg()
        imu_msg.header.frame_id = self.frame_id

        # 1. Orientation (on recopie celle de l'odom)
        imu_msg.orientation = self.latest_odom.pose.pose.orientation

        # 2. Vitesse angulaire (déjà présente dans l'odom)
        imu_msg.angular_velocity.x = 0.0
        imu_msg.angular_velocity.y = 0.0
        imu_msg.angular_velocity.z = curr_w_z

        # 3. Accélération linéaire
        imu_msg.linear_acceleration.x = acc_x
        imu_msg.linear_acceleration.y = acc_y
        imu_msg.linear_acceleration.z = acc_z

        # 4. Covariances (Valeurs réalistes pour un EKF)
        imu_msg.orientation_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        imu_msg.angular_velocity_covariance = [0.001, 0.0, 0.0, 0.0, 0.001, 0.0, 0.0, 0.0, 0.001]
        imu_msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

        # Publication
        self.imu_pub.publish(imu_msg)

        # Mise à jour des mémoires
        self.prev_v_x = curr_v_x
        self.prev_v_y = curr_v_y
        self.prev_v_z = curr_v_z
        self.prev_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = VirtualImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()