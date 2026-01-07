import numpy as np
from skimage.morphology import skeletonize, disk, binary_dilation, binary_erosion
from skimage.segmentation import watershed
from skimage.filters import threshold_yen
from skimage import measure
from scipy.spatial import distance

import matplotlib.pyplot as plt
from skimage.morphology import *
from skimage.segmentation import watershed

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Bool
from custom_msgs.msg import TrackDescription

MAP_TOPIC = '/map'
ODOM_TOPIC = '/odom'
TRIGGER_TOPIC = '/centerline_trigger'
TRACK_TOPIC = '/track_description'

class CenterlineExtraction(Node):
    def __init__(self):
        super().__init__('centerline_extraction_node')

        # Variables
        self.cached_odom_msg = None
        self.cached_map_msg = None

        # Parameterss
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_binary_threshold', 0.6),
                ('max_road_width', 5.0),
                ('visualize_matplotlib_on_success', False),
                ('visualize_matplotlib_on_failure', False),
            ]
        )

        self.map_binary_threshold = self.get_parameter('map_binary_threshold').get_parameter_value().double_value
        self.max_road_width = self.get_parameter('max_road_width').get_parameter_value().double_value
        self.visualize_matplotlib_on_success = self.get_parameter('visualize_matplotlib_on_success').get_parameter_value().bool_value
        self.visualize_matplotlib_on_failure = self.get_parameter('visualize_matplotlib_on_failure').get_parameter_value().bool_value

        # Topics
        self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 1)
        self.create_subscription(OccupancyGrid, MAP_TOPIC, self.map_callback, 1)
        self.create_subscription(Bool, TRIGGER_TOPIC, self.trigger_callback, 1)
        self.track_publisher = self.create_publisher(TrackDescription, TRACK_TOPIC, 10)

    def odom_callback(self, msg: Odometry):
        self.cached_odom_msg = msg
        
    def map_callback(self, msg: OccupancyGrid):
        self.cached_map_msg = msg

    def trigger_callback(self, msg: Bool):
        if not (self.cached_odom_msg is not None and self.cached_map_msg is not None and msg.data):
            missing = [x[1] for x in [(self.cached_odom_msg, 'odom'), (self.cached_map_msg, 'map')] if x[0] is None]
            missing = ' and '.join(missing)
            self.get_logger().error(f'Received a trigger message, but the {missing} message(s) is/are missing')
        else:
            centerline_extraction_success, centerline_extraction_result = centerline_extraction_entrypoint(self.cached_odom_msg, self.cached_map_msg, self.map_binary_threshold, self.max_road_width)
            if centerline_extraction_success:
                # Extract the data from the result
                (centerline_traj_world, inside_wall_world, outside_wall_world), (boolean_map, inside_wall, outside_wall, centerline_traj, robot_position_grid, robot_heading) = centerline_extraction_result

                # Create the path messages
                centerline_msg = self.numpy_array_to_path_msg(centerline_traj_world)
                inside_wall_msg = self.numpy_array_to_path_msg(inside_wall_world)
                outside_wall_msg = self.numpy_array_to_path_msg(outside_wall_world)

                # Create the overall message
                track_description_msg = TrackDescription()
                track_description_msg.success = True
                track_description_msg.header.stamp = self.get_clock().now().to_msg()
                track_description_msg.centerline = centerline_msg
                track_description_msg.inside_wall = inside_wall_msg
                track_description_msg.outside_wall = outside_wall_msg
                self.track_publisher.publish(track_description_msg)

                # Visualize
                if self.visualize_matplotlib_on_success:
                    plt.imshow(boolean_map.T)
                    plt.plot(inside_wall[:, 0], inside_wall[:, 1], linewidth=2, color='red', label="Inside Wall")
                    plt.plot(outside_wall[:, 0], outside_wall[:, 1], linewidth=2, color='blue', label="Outside Wall")
                    plt.scatter(centerline_traj[:, 0], centerline_traj[:, 1], linewidth=2, c=np.arange(len(centerline_traj)), label="Centerline")
                    plt.scatter(robot_position_grid[0], robot_position_grid[1], c='black', label='Origin')
                    plt.quiver(robot_position_grid[0], robot_position_grid[1], np.cos(robot_heading), np.sin(robot_heading), color='black', label='Heading')

                    # plt.legend()
                    plt.title("Centerline Extraction Success")
                    plt.axis('image')
                    plt.show()
            else:
                self.get_logger().error('The racetrack must have exactly two contours')

                # Create an empty message, with success = False
                track_description_msg = TrackDescription()
                track_description_msg.success = False
                track_description_msg.header.stamp = self.get_clock().now().to_msg()
                self.track_publisher.publish(track_description_msg)

                if self.visualize_matplotlib_on_failure:
                    (boolean_map, distance_field, centerline_binarymap, raw_centerline, racetrack_binarymap, robot_position_grid, robot_heading) = centerline_extraction_result
                    plt.imshow(distance_field.T)
                    plt.imshow(raw_centerline.T, alpha=0.5, cmap='gray')
                    plt.scatter(robot_position_grid[0], robot_position_grid[1], c='black', label='Origin')
                    plt.quiver(robot_position_grid[0], robot_position_grid[1], np.cos(robot_heading), np.sin(robot_heading), color='black', label='Heading')

                    plt.title("Centerline Extraction Failure")
                    plt.axis('image')
                    plt.show()

    def numpy_array_to_path_msg(self, array: np.ndarray):
        # Returns a list of Pose messages
        return [Pose(position=Point(x=point[0], y=point[1])) for point in array]




def centerline_extraction_entrypoint(odom_msg: Odometry, map_msg: OccupancyGrid, map_binary_threshold: float=0.6, max_road_width: float=5.0):
    # Extract the data from the odom topic
    robot_position_ros = np.array([odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y])
    robot_heading = heading_from_quat(odom_msg.pose.pose.orientation) - np.pi / 2

    # Extract the data from the map message
    map_width = map_msg.info.width
    map_height = map_msg.info.height
    map_origin = np.array([map_msg.info.origin.position.x, map_msg.info.origin.position.y])
    map_resolution = map_msg.info.resolution
    map_data = np.array(map_msg.data)

    # Convert the robot position to a grid position
    robot_position_grid = world_position_to_grid(robot_position_ros, map_origin, map_resolution)

    # Get the trajectory
    raw_map, boolean_map = prepare_map(map_data, map_width, map_height, map_binary_threshold)
    distance_field = generate_distance_field(boolean_map, map_resolution, max_road_width)
    centerline_binarymap, racetrack_binarymap, raw_centerline = extract_centerline_binarymap(boolean_map, distance_field)
    inside_wall, outside_wall = extract_racetrack_walls(racetrack_binarymap)
    if inside_wall is None or outside_wall is None:
        return False, (boolean_map, distance_field, centerline_binarymap, raw_centerline, racetrack_binarymap, robot_position_grid, robot_heading)
    centerline_traj = extract_centerline_trajectory(centerline_binarymap, robot_position_grid, robot_heading)

    # Generate the trajectory in world coordinates
    centerline_traj_world = grid_trajectory_to_world(centerline_traj, map_origin, map_resolution)
    inside_wall_world = grid_trajectory_to_world(inside_wall, map_origin, map_resolution)
    outside_wall_world = grid_trajectory_to_world(outside_wall, map_origin, map_resolution)

    return True, ((centerline_traj_world, inside_wall_world, outside_wall_world), (raw_map, inside_wall, outside_wall, centerline_traj, robot_position_grid, robot_heading))


def prepare_map(data: np.ndarray, width: int, height: int, occupancy_threshold: float):
    # Reshape the data from the topic to the shape of the map
    data = data.reshape((height, width))

    # replace unknow values (== -1) with empty cells (?)
    data[data <= 0] = 0

    #(f'Recommended threshold by Yen\'s method:', threshold_yen(data))

    # Binarize the map
    boolean_map = data > int(np.max(data) * occupancy_threshold)

    # Apply a morphological open filter to remove small noise
    boolean_map = binary_dilation(boolean_map, disk(2)) # inflate walls, and thus remove missing values/gaps
    boolean_map = binary_erosion(boolean_map, disk(1)) # erode the inflated walls to remove the noise
    # basically we use a morphological closing filter, but with different sturcturing elements
    
    return data, boolean_map

def generate_distance_field(boolean_data, resolution, max_road_width):
    # might be replaced by ndi.distance_transform_edt(boolean_data).astype(int)
    # if we use some other filters applied on top of it
    
    # Compute the distance field
    max_tries = int(max_road_width / resolution) + 1
    padding = max_tries + 1

    distance_field = boolean_data.copy().astype(np.uint8)
    distance_field = np.pad(distance_field, padding, mode='constant', constant_values=0)

    for prev_dist in range(1,max_tries+1):
        # Get all the points that have the current distance to the walls
        prev_points = np.argwhere(distance_field == prev_dist)
        # Get their neighbors
        neighbors = np.array([[0,1],[0,-1],[1,0],[-1,0]])

        new_points = prev_points[:, None] + neighbors[None, :]
        new_points = np.unique(new_points.reshape(-1, 2), axis=0)
        new_points = new_points[distance_field[new_points[:, 0], new_points[:, 1]] == 0]

        # Update come from
        distance_field[new_points[:, 0], new_points[:, 1]] = prev_dist + 1

    # Consider that if distance == max_tries, we are outside of the circuit, and thus rmeove those points, again using a percolation method
    prev_points = np.argwhere(distance_field == max_tries)
    while prev_points.size > 0:
        # Get all the points that have the current distance to the walls
        neighbors = np.array([[0,1],[0,-1],[1,0],[-1,0]])

        new_points = prev_points[:, None] + neighbors[None, :]
        new_points = np.unique(new_points.reshape(-1, 2), axis=0)
        new_points = new_points[distance_field[new_points[:, 0], new_points[:, 1]] > 1]
        prev_points = new_points

        # Update come from
        distance_field[new_points[:, 0], new_points[:, 1]] = 0

    return distance_field[padding:-padding, padding:-padding]

def extract_centerline_binarymap(boolean_map, distance_field):
    raw_centerline_binarymap = (skeletonize(distance_field, method='lee') > 0).astype(np.uint8)
    racetrack_binarymap = watershed(-distance_field, raw_centerline_binarymap, mask=~boolean_map).astype(bool)
    centerline_binarymap = raw_centerline_binarymap * racetrack_binarymap
    return centerline_binarymap, racetrack_binarymap, raw_centerline_binarymap

def extract_racetrack_walls(racetrack_binarymap):
    racetrack_contours = measure.find_contours(racetrack_binarymap)
    if len(racetrack_contours) != 2:
        return None, None
        # raise ValueError('The racetrack must have exactly two contours') # To change later

    # Figure out which wall is the inside wall and which is the outside wall
    # We assume that the inside wall is the one with the smallest area
    areas = [polygon_area(contour) for contour in racetrack_contours]
    inside_wall, outside_wall = np.argsort(areas)
    return racetrack_contours[inside_wall], racetrack_contours[outside_wall]

def extract_centerline_trajectory(centerline_binarymap, robot_position, robot_heading):
    centerline_contours = measure.find_contours(centerline_binarymap)

    # Compute centroids of contours
    centroids = np.array([np.mean(contour, axis=0) for contour in centerline_contours])

    # Compute pairwise distances between centroids once
    dist_matrix = distance.cdist(centroids, centroids)

    # Create a visited list and order the contours
    order = [0]  # Start with the centroid
    visited = [False] * len(centerline_contours)
    visited[0] = True

    # Greedy approach to find closest neighbor
    for _ in range(1, len(centerline_contours)):
        last_idx = order[-1]
        # Find the nearest unvisited contour
        nearest_idx = np.argmin(dist_matrix[last_idx] + np.array(visited) * 1e6)  # Large value for visited
        visited[nearest_idx] = True
        order.append(nearest_idx)

    # Reorder contours based on calculated order
    ordered_centroids = centroids[order]

    # Set the origin of the trajectory as close as possible to the robot_position
    origin_index = np.argmin(np.linalg.norm(ordered_centroids - robot_position, axis=1))
    ordered_centroids = np.roll(ordered_centroids, -origin_index, axis=0)

    # Use the trajectory heading to determine the direction of the trajectory
    # Compute the angle between the heading and the first segment of the trajectory
    heading_vector = np.array([np.cos(robot_heading), np.sin(robot_heading)])
    first_segment = ordered_centroids[1] - ordered_centroids[0]
    angle = np.arccos(np.dot(heading_vector, first_segment) / (np.linalg.norm(heading_vector) * np.linalg.norm(first_segment)))

    # If the angle is greater than 90 degrees, reverse the trajectory
    if angle > np.pi / 2:
        ordered_centroids = ordered_centroids[::-1]
    
    return ordered_centroids

def grid_trajectory_to_world(trajectory, origin, resolution):
    # swap x and y
    return np.flip(trajectory * resolution, axis=1) + origin

def world_position_to_grid(world_position, origin, resolution):
    # swap x and y
    grid_pos_float = (world_position - origin) / resolution
    return grid_pos_float.astype(int)

def polygon_area(vertices):
    # Using the shoelace formula
    x = vertices[:, 0]
    y = vertices[:, 1]
    return 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

def heading_from_quat(orientation):
    w, x, y, z = orientation.w, orientation.x, orientation.y, orientation.z
    return np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))


def main():
    rclpy.init()
    node = CenterlineExtraction()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

