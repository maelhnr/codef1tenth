import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped, PointStamped
import numpy as np
import tf2_geometry_msgs
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
import numpy as np
from scipy.spatial.transform import Rotation as R
from scipy.optimize import least_squares
from geometry_msgs.msg import Quaternion
import math


qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,  # Ensures reliable message delivery
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,  # Store messages for late subscribers (like RViz)
    depth=1  # Keep only the latest message
)

class custom_slam(Node):
    def __init__(self):
        super().__init__('custom_slam')
        # Map settings
        self.map_resolution = 0.05  # 10 cm per cell
        self.map_width = 1000  # in cells
        self.map_height = 1000  # in cells
        self.map_origin = [self.map_width*self.map_resolution/2, self.map_height*self.map_resolution/2, 0]  # Map origin (x, y) in meters
        self.map_data = np.zeros((self.map_width , self.map_height), dtype=np.int8)  # Initialize all cells as free (0)
        self.scan0 = None # Now scan0 is an instance of LaserScan        # Subscribe to the LaserScan topic

        self.scan_subscriber = self.create_subscription(
            LaserScan, 
            '/scan', 
            self.scan_callback, 
            10)

        # Publisher for transformed LaserScan
        self.scan_publisher = self.create_publisher(LaserScan, '/transformed_scan', 10)

        # Publisher for the Map
        self.map_publisher = self.create_publisher(OccupancyGrid, '/map_slam', qos_profile)

        # TF2 buffer and listener to query transforms
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info("Node started, waiting for scan and tf data.")



    def map_callback(self, scan_msg):
        try: 
            self.get_logger().info(f"dime algo por el amor de dios")
            self.get_logger().info(f"Grid out of map size:  {scan_msg}")
        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")

    def scan_callback(self, scan_msg):
        try:
            # Get the transform from map -> ego_racecar/laser
            transform: TransformStamped = self.tf_buffer.lookup_transform(
                'odom', 
                'laser',  # "ego_racecar/laser"
                rclpy.time.Time()  # Look up the most recent transform
            )

            if abs(scan_msg.header.stamp.sec - transform.header.stamp.sec) ==0 and abs(scan_msg.header.stamp.nanosec - transform.header.stamp.nanosec) < 50000000: 

                cartesian_points = self.laser_scan_to_cartesian(scan_msg)

                if self.scan0 is not None:
                    oldscan = self.scan0
                    translation_x, translation_y, rotation_angle = self.find_best_transform( oldscan, cartesian_points)

                    x1, y1 = zip(*oldscan)  # Reference scan
                    x2, y2 = zip(*cartesian_points)  # Current scan (assuming self-matching for now)
                    transformed_points = self.rotate_and_translate_points( cartesian_points, translation_x, translation_y, rotation_angle)
                    self.scan0 = transformed_points;
                else:
                    self.scan0 = cartesian_points;


            # Check if the timestamps are close enough
                # Convert laser scan to Cartesian points
                cartesian_points = self.laser_scan_to_cartesian(scan_msg)

                # Transform the points from laser to map frame
                transformed_points2 = self.transform_points(cartesian_points, transform)

                # Create a new LaserScan message to publish the transformed points
                transformed_scan_msg = self.create_transformed_scan(scan_msg, transformed_points)

                # Publish the transformed LaserScan message
                self.scan_publisher.publish(transformed_scan_msg)

                # Create a map from the transformed points and publish it
                occupancy_grid_msg = self.create_map_from_points(transformed_points2)
                self.map_publisher.publish(occupancy_grid_msg)
            else:
                self.get_logger().info("Timestamps are too far apart, skipping this scan.")

        except Exception as e:
            self.get_logger().warn(f"Failed to process scan or transform: {e}")

        except Exception as e:
            self.get_logger().warn(f"Transform not available: {e}")


    def find_best_transform(self, scan1, scan2):


        # Initial guess for translation and rotation: (tx, ty, angle)
        initial_guess = np.array([0.0, 0.0, 0.0])

        # Use least squares optimization to minimize the cost function
        result = least_squares(self.cost_function, initial_guess, args=(scan1, scan2))

        # Extract the optimized translation and rotation
        translation_x, translation_y, rotation_angle = result.x

        return translation_x, translation_y, rotation_angle


    def cost_function(self, params, points1, points2):
      
        tx, ty, angle = params

        # Rotate and translate points2
        rotated_translated_points2 = self.rotate_and_translate_points(points2, tx, ty, angle)

        # Compute the residuals (difference between points1 and transformed points2)
        residuals = []
        for (x1, y1), (x2, y2) in zip(points1, rotated_translated_points2):
            residuals.append(x1 - x2)
            residuals.append(y1 - y2)

        return residuals

    def laser_scan_to_cartesian(self, scan_msg):
        """Convert LaserScan to Cartesian coordinates."""
        ranges = np.array(scan_msg.ranges)
        angles = np.linspace(scan_msg.angle_min, scan_msg.angle_max, len(ranges))

        sorted_indices = np.argsort(ranges)  # This gives the indices that would sort the ranges
        half_length = len(ranges) // 2  # Half the length of the array
    
    # Get the smallest half of the ranges and their corresponding angles
        ranges = ranges[sorted_indices[:half_length]]
        angles = angles[sorted_indices[:half_length]]

        # Convert polar coordinates to Cartesian
        x_coords = ranges * np.cos(angles)
        y_coords = ranges * np.sin(angles)

        # Remove invalid readings
        valid = np.isfinite(x_coords) & np.isfinite(y_coords)
        return list(zip(x_coords[valid], y_coords[valid]))

    def transform_points(self, points, transform):
        """Transform points from laser frame to map frame."""
        # Transform the Cartesian points to the map frame using tf2
        transformed_points = []
        
        for x, y in points:
            # Create a PointStamped for each laser point in the laser frame
            laser_point = PointStamped()
            laser_point.header.frame_id = 'laser'
            laser_point.header.stamp = self.get_clock().now().to_msg()

            laser_point.point.x = x
            laser_point.point.y = y
            laser_point.point.z = 0.0  # Assuming 2D lidar (no height)

            # Transform the laser point to the map frame
            try:
                transformed_point = tf2_geometry_msgs.do_transform_point(laser_point, transform)
                transformed_points.append((transformed_point.point.x, transformed_point.point.y))
            except Exception as e:
                self.get_logger().warn(f"Failed to transform point: {e}")
                continue

        return transformed_points


    def rotate_and_translate_points(self, points, tx, ty, angle):
            """Rotate and translate points using the given transformation parameters."""
            # Rotation matrix for the given angle
            rotation_matrix = np.array([[np.cos(angle), -np.sin(angle)],
                                        [np.sin(angle), np.cos(angle)]])
            
            # Apply the rotation and translation to each point
            transformed_points = []
            for x, y in points:
                rotated_point = np.dot(rotation_matrix, np.array([x, y]))  # Rotate the point
                translated_point = rotated_point + np.array([tx, ty])  # Translate the point
                transformed_points.append(tuple(translated_point))
            
            return transformed_points

    def create_transformed_scan(self, original_scan_msg, transformed_points):
        """Create a new LaserScan message with the transformed points."""
        transformed_scan_msg = LaserScan()

        # Copy metadata from the original scan
        transformed_scan_msg.header = original_scan_msg.header
        transformed_scan_msg.angle_min = original_scan_msg.angle_min
        transformed_scan_msg.angle_max = original_scan_msg.angle_max
        transformed_scan_msg.angle_increment = original_scan_msg.angle_increment
        transformed_scan_msg.time_increment = original_scan_msg.time_increment
        transformed_scan_msg.scan_time = original_scan_msg.scan_time
        transformed_scan_msg.range_min = original_scan_msg.range_min
        transformed_scan_msg.range_max = original_scan_msg.range_max

        # Convert the transformed Cartesian points back to polar coordinates (ranges and angles)
        transformed_ranges = np.zeros(len(transformed_points))
        for i, (x, y) in enumerate(transformed_points):
            transformed_ranges[i] = np.sqrt(x**2 + y**2)

        # Populate the LaserScan's range data with the transformed ranges
        transformed_scan_msg.ranges = transformed_ranges.tolist()

        return transformed_scan_msg

    def create_map_from_points(self, transformed_points):
        """Create an OccupancyGrid message from the transformed points."""

        # Map origin (in grid cells)
        origin_x = int(self.map_origin[0] / self.map_resolution)
        origin_y = int(self.map_origin[1] / self.map_resolution)

        for x, y in transformed_points:
            # Convert the map coordinates to grid cell indices
            grid_x = int((-x + self.map_origin[0]) / self.map_resolution)
            grid_y = int((-y + self.map_origin[1]) / self.map_resolution)

            # Ensure the grid cells are within the map size
            if 0 <= grid_x < self.map_width and 0 <= grid_y < self.map_height:
                idx = grid_y
                idy = grid_x
                self.map_data[idx,idy] = 100  # Mark as occupied (100)

        # Create the OccupancyGrid message
        map_msg = OccupancyGrid()
        map_msg.header = Header()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'odom'
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_width
        map_msg.info.height = self.map_height
        map_msg.info.origin.position.x = float(self.map_origin[0])
        map_msg.info.origin.position.y = float(self.map_origin[1])
        map_msg.info.origin.orientation.w = float(self.map_origin[2])
        map_msg.data = [item for sublist in self.map_data.tolist() for item in sublist]

        
        return map_msg  

def main(args=None):
    rclpy.init(args=args)
    node = custom_slam()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
