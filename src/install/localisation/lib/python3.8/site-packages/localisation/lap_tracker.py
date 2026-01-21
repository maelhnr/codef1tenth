import rclpy
from rclpy.node import Node
import numpy as np
from nav_msgs.msg import Odometry
from custom_msgs.msg import LapProgress

ODOM_TOPIC = 'odom'
TOLERANCE = 0.20
        

def projection(point, segment_a, segment_b):
    """ Projects a point on a segment, returns the projected point and the distance to the original point """
    x1, y1 = segment_a
    x2, y2 = segment_b
    x0, y0 = point
    if x1 == x2:
        return x1, y0
    a = (y2-y1)/(x2-x1)
    b = y1-a*x1
    x = (a*y0+x0-a*b)/(1+a**2)
    y = a*x+b
    projection = np.array([x, y])
    return projection, np.linalg.norm(projection - point)


def project_on_path(path, point):
    # Find the closest point on the path
    closest_index = np.argmin(np.linalg.norm(path - point, axis=1))
    closest_pt = path[closest_index]

    # We need to find the projection of the lookahead point to the trajctory, just is on the segment [closest_pt-1, closest_pt] or [closest_pt, closest_pt+1]
    proj_a, dist_a = projection(point, path[(closest_index-1) % len(path)], closest_pt)
    proj_b, dist_b = projection(point, closest_pt, path[(closest_index+1) % len(path)])
    if dist_a < dist_b:
        projected_pt = proj_a
        # other_curvature = self.curvature[(closest_index-1) % len(waypoints)]
        forward_pt = path[closest_index]
        backward_pt = path[(closest_index-1) % len(path)]
    else:
        projected_pt = proj_b
        # other_curvature = self.curvature[(closest_index+1) % len(waypoints)]
        forward_pt = path[(closest_index+1) % len(path)]
        backward_pt = path[closest_index]

    # Compute the heading on the path
    heading = np.arctan2(forward_pt[1] - backward_pt[1], forward_pt[0] - backward_pt[0])
    
    # Find the signed distance
    T = (forward_pt - backward_pt)
    N = np.array([T[1], -T[0]])
    signed_dist = np.dot(point - projected_pt, N)

    return projected_pt, signed_dist, heading, closest_index

class LapTracker(Node):
    """
    The role of this Node is to detect wheter or not the robot has completed a full lap, and if so, to provide an estimation of the percentage of the track that has been covered.
    This node requires an Odometry that includes the position of the robot
    """
    def __init__(self):
        super().__init__('lap_tracker__node')

        self.waypoints = []
        self.cummulative_distances = []
        self.normalized_distances = None
        self.track_completed_once = False
        
        self.lap_number = 0
        self.lap_progress = np.nan
        
        self.odom_sub = self.create_subscription(Odometry, ODOM_TOPIC, self.odom_callback, 10)
        self.lap_progress_publisher = self.create_publisher(LapProgress, 'lap_progress', 10)

    def odom_callback(self, msg: Odometry):
        position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        velocity = msg.twist.twist.linear.x

        if np.abs(velocity) > 1e-6: # Update only if the car is moving
            if not self.track_completed_once:
                self.callback_not_completed(position)
            else:
                self.callback_completed(position)

        lap_progress_msg = LapProgress()
        lap_progress_msg.header.stamp = self.get_clock().now().to_msg()
        lap_progress_msg.lap_number = self.lap_number
        lap_progress_msg.lap_progress = self.lap_progress
        self.lap_progress_publisher.publish(lap_progress_msg)

    def callback_not_completed(self, position):
        on_start_segment = True if len(self.cummulative_distances) == 0 else self.cummulative_distances[-1] < 2 * TOLERANCE
        projected_pt, signed_dist, _, closest_index = project_on_path(self.waypoints, position) if len(self.waypoints) != 0 else (np.array([0., 0.]), np.inf, 0., -1)

        track_completed_trigger = not on_start_segment and np.abs(signed_dist) < TOLERANCE and closest_index <= np.floor(len(self.waypoints) * 0.25)
        first_trigger = track_completed_trigger and not self.track_completed_once
        self.track_completed_once |= track_completed_trigger

        if first_trigger:
            self.waypoints = np.array(self.waypoints)
            self.cummulative_distances = np.array(self.cummulative_distances)
            self.normalized_distances = self.cummulative_distances / self.cummulative_distances[-1]
            self.lap_number += 1
        
        if not self.track_completed_once:
            prev_position = self.waypoints[-1] if len(self.waypoints) > 0 else position
            prev_distance = self.cummulative_distances[-1] if len(self.cummulative_distances) > 0 else 0.

            distance = np.linalg.norm(position - prev_position)

            self.cummulative_distances.append(prev_distance + distance)
            self.waypoints.append(position)

    def callback_completed(self, position):
        closest_index = -1 if len(self.waypoints) == 0 else np.argmin(np.linalg.norm(self.waypoints - position, axis=1))
        current_progress = self.normalized_distances[closest_index]
        if np.abs(current_progress - self.lap_progress) > 0.5 and current_progress < 0.05:
            self.lap_number += 1
            self.lap_progress = current_progress
        else:
            self.lap_progress = max(current_progress, self.lap_progress)



def main(args=None):
    rclpy.init(args=args)
    node = LapTracker()
    rclpy.spin(node)
    node.occupancy_grid_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
