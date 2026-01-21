#!/usr/bin/env python3

from math import atan2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry

class OccupancyGridNode(Node):
    def __init__(self, lidarscan_topic='/scan', odom_topic=None, map_topic=None):
        super().__init__('occupancy_grid_node')

        bounds = [-10, 15, -30, 10] # Tuples which defines the boundaries (xmin, xmax, ymin, ymax)
        self.resolution = 0.3 # Side length of each cell
        self.occ_threshold = 0.4 # Probability above which the cell is considered occupied
        self.occ_probability = 0.65 # Probability that the cell is occupied when the LIDAR says it is occupied
        self.free_probability = 0.35 # Probability that the cell is empty when the LIDAR says it is empty
        self.prior_probability = 0.5 # Probability that the cell is occupied without any info (no scan), we put 0.5
        self.log_prob_clip = 8 # Maximum value for log-prob. This is used because we do not need to store log-probability to infinty
        self.lidar_fov = 4.71238898038 # Radians
        self.lidar_points = 1080
        self.lidar_resolution = self.lidar_fov / self.lidar_points
        self.scan_distance_to_base_link = 0.275 # Distance between lidar_base and the base of the car. Used to change reference frame 
        self.throttle_scans = 1 # The map-update will use 1 out of throttle_scans dataset from the LIDAR. To be used if algorithm is too slow
        self.count = 0

        assert(bounds[1] > bounds[0])
        assert(bounds[3] > bounds[2])
        assert(self.resolution > 0)

        self.xmin, self.xmax, self.ymin, self.ymax = bounds

        self.num_cell_x = int(np.ceil((self.xmax - self.xmin) / self.resolution))
        self.num_cell_y = int(np.ceil((self.ymax - self.ymin) / self.resolution))

        self.l_prior = self.logodds(self.prior_probability)
        self.l_occ = self.logodds(self.occ_probability)
        self.l_free = self.logodds(self.free_probability)

        self.prob_map = np.ones((self.num_cell_x, self.num_cell_y)) * self.prior_probability
        self.log_prob_map = self.logodds(self.prob_map.copy())
        self.occupancy_grid = np.zeros((self.num_cell_x, self.num_cell_y), dtype=np.int8)

        self.alpha = self.resolution
        self.beta = self.lidar_resolution
        self.z_max = 6

        self.z = np.zeros((self.lidar_points, 2))
        self.LIDAR_ANGLES = np.linspace(-self.lidar_fov / 2, self.lidar_fov / 2, self.lidar_points)
        self.z[:, 0] = np.linspace(-self.lidar_fov / 2, self.lidar_fov / 2, self.lidar_points)
        self.pose = [0, 0, 0]

        X_mesh, Y_mesh = np.meshgrid(np.arange(self.xmin, self.xmax, self.resolution),
                                     np.arange(self.ymin, self.ymax, self.resolution), sparse=True)
        self.X_mesh = X_mesh.T
        self.Y_mesh = Y_mesh.T

        self.theta_to_grid = np.empty(np.shape(X_mesh))
        self.dist_to_grid = np.empty(np.shape(X_mesh))

        self.create_subscription(Odometry, odom_topic, self.callback_odom, 10)
        self.create_subscription(LaserScan, lidarscan_topic, self.lidar_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, map_topic, 10)

    def update_map(self):
        dx = self.X_mesh - self.pose[0]
        dy = self.Y_mesh - self.pose[1]

        self.theta_to_grid = np.arctan2(dy, dx) - self.pose[2]
        self.theta_to_grid[self.theta_to_grid > np.pi] -= 2 * np.pi
        self.theta_to_grid[self.theta_to_grid < -np.pi] += 2 * np.pi

        self.dist_to_grid = np.linalg.norm([dx, dy])

        mask = (self.dist_to_grid < self.z_max) & (np.abs(self.theta_to_grid) <= self.lidar_fov / 2)
        theta_to_grid_masked = self.theta_to_grid[mask]
        dist_to_grid_masked = self.dist_to_grid[mask]
        log_prob_map_masked = self.log_prob_map[mask]

        temparray_angle = np.empty(np.shape(mask), dtype=bool)
        temparray_dist_free = np.empty(np.shape(mask), dtype=bool)
        temparray_dist_occ = np.empty(np.shape(mask), dtype=bool)

        free_mask = np.empty(np.shape(mask), dtype=bool)
        occ_mask = np.empty(np.shape(mask), dtype=bool)

        for z_i in self.z:
            b = z_i[0]
            r = z_i[1]

            temparray_angle = np.abs(theta_to_grid_masked - b) <= self.beta / 2.0
            temparray_dist_free = dist_to_grid_masked < (r - self.alpha / 2.0)
            temparray_dist_occ = np.abs(dist_to_grid_masked - r) <= self.alpha / 2.0

            free_mask = (temparray_angle) & (temparray_dist_free)
            occ_mask = (temparray_angle) & (temparray_dist_occ)

            log_prob_map_masked[occ_mask] += self.l_occ
            log_prob_map_masked[free_mask] += self.l_free

        self.log_prob_map[mask] = log_prob_map_masked
        return

    def lidar_callback(self, data):
        self.count += 1
        if self.count == self.throttle_scans:
            self.count = 0

            self.z[:, 1] = data.ranges[:]

            my_msg = OccupancyGrid()
            my_msg.info.width = self.num_cell_x
            my_msg.info.height = self.num_cell_y
            my_msg.info.resolution = self.resolution
            my_msg.info.origin.position.x = float(self.pose[0])
            my_msg.info.origin.position.y = float(self.pose[1])
            my_msg.info.origin.orientation.w = float(self.pose[2])

            start = rclpy.clock.Clock().now()
            self.update_map()
            self.update_prob_map()
            self.update_occupancy_grid()
            #print(np.array(self.occupancy_grid))
            #my_msg.data = self.occupancy_grid.flatten()
            my_msg.data = [item for sublist in self.occupancy_grid.tolist() for item in sublist]
            #print(self.occupancy_grid)
            
            # On ecrit la map dans un fichier texte
            #f = open('map.txt', 'w')
            #for line in self.occupancy_grid:
            	#f.write(" "+ str(line) + "\n")
            #f.close()
            
            self.map_pub.publish(my_msg)

            print(rclpy.clock.Clock().now() - start)
            print("Map published")
        print(self.count)
        return

    def update_prob_map(self):
        self.log_prob_map = np.clip(self.log_prob_map, -self.log_prob_clip, self.log_prob_clip)
        self.prob_map = (1.0 - 1.0 / (1.0 + np.exp(self.log_prob_map)))

    def update_occupancy_grid(self):
        where_occ = self.prob_map > self.occ_threshold
        where_free = self.prob_map <= self.occ_threshold
        self.occupancy_grid[where_free] = 0
        self.occupancy_grid[where_occ] = 1

    def callback_odom(self, msg):
        pose = msg.pose.pose.position
        quat = msg.pose.pose.orientation
        siny_cosp = 2 * (quat.w * quat.z + quat.x * quat.y)
        cosy_cosp = 1 - 2 * (quat.y * quat.y + quat.z * quat.z)
        self.pose = [pose.x, pose.y, atan2(siny_cosp, cosy_cosp)]

    def logodds(self, prob):
        return np.log(prob / (1.0 - prob))


def main(args=None):
    rclpy.init(args=args)
    ros_topics = {
        'lidarscan_topic': '/scan',
        'odom_topic': None,  # Provide the correct topic name
        'map_topic': None   # Provide the correct topic name
    }
    occupancy_grid_node = OccupancyGridNode('/scan', '/odom', '/map')
    rclpy.spin(occupancy_grid_node)
    occupancy_grid_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

