#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
import numpy as np
import math as mp
from rclpy.node import Node

from custom_msgs.msg import TrackDescription
from std_msgs.msg import Bool

ODOMETRY_TOPIC = 'odom'
DRIVE_TOPIC = '/drive'
WAYPOINTS_TOPIC = '/waypoints'
TRACK_DESCRIPTION_TOPIC = '/track_description'
LOOKAHEAD = 2.
SPEED_FACTOR = 3.0

class PurePursuit(Node):
    """
    The class that handles pure pursuit.
    """
    def __init__(self):
        #sbscribe to /pf/pose/odom for particle filter pose
        
        super().__init__('pure_pursuit')
        
        #The waypoint list is read from a topic and stored in the "waypoints" variable
        self.waypoints = []

        self.lookahead = LOOKAHEAD # const
        self.print_info = False

        #Topics definition and initialisation
        self.get_waypoints = self.create_subscription(Path, WAYPOINTS_TOPIC, self.waypoints_callback, 1)
        self.odometry_subscriber = self.create_subscription(Odometry, ODOMETRY_TOPIC, self.odometry_callback, 1)  
        self.track_description_subscriber = self.create_subscription(TrackDescription, TRACK_DESCRIPTION_TOPIC, self.track_description_callback, 1) # DEBUG FOR NOW   
        self.drive_publisher = self.create_publisher(AckermannDriveStamped,DRIVE_TOPIC,1)
        self.target_point_publisher = self.create_publisher(PointStamped, '/pathpt', 1)
        self.lookahead_point_publisher = self.create_publisher(PointStamped, '/lookhead', 1)
        self.debug_waypoints_publisher = self.create_publisher(Path, '/wpt', 1)
        
    def waypoints_callback(self, msg):
        """Load waypoints sent by Astar Planifier"""
        self.waypoints = [] # Suppression of former waypoints
        for elt in msg.poses:
            x = elt.pose.position.x
            y = elt.pose.position.y
            self.waypoints.append([x, y])
        self.waypoints = np.array(self.waypoints)

    def track_description_callback(self, msg: TrackDescription):
        # Temporarily, follow the centerline from the track description msg
        self.waypoints = []
        for pose in msg.centerline:
            self.waypoints.append([pose.position.x, pose.position.y])
        self.waypoints = np.array(self.waypoints)
        

    def projection(self, point, segment_a, segment_b):
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
        
    def get_next_waypoint(self, location, heading):
        # Find lookhead point
        heading_vec = np.array([np.cos(heading), np.sin(heading)])
        lookahead_pt = location + self.lookahead * heading_vec

        # Find the closest point on the path
        closest_index = np.argmin(np.linalg.norm(self.waypoints - lookahead_pt, axis=1))
        closest_pt = self.waypoints[closest_index]

        # We need to find the projection of the lookahead point to the trajctory, just is on the segment [closest_pt-1, closest_pt] or [closest_pt, closest_pt+1]
        proj_a, dist_a = self.projection(lookahead_pt, self.waypoints[(closest_index-1) % len(self.waypoints)], closest_pt)
        proj_b, dist_b = self.projection(lookahead_pt, closest_pt, self.waypoints[(closest_index+1) % len(self.waypoints)])
        if dist_a < dist_b:
            projected_pt = proj_a
        else:
            projected_pt = proj_b

        # Publish debug topics 
        lookahead_pt_msg = PointStamped()
        lookahead_pt_msg.header.stamp = self.get_clock().now().to_msg()
        lookahead_pt_msg.point.x = lookahead_pt[0]
        lookahead_pt_msg.point.y = lookahead_pt[1]
        lookahead_pt_msg.point.z = 0.0
        lookahead_pt_msg.header.frame_id = "map"
        self.lookahead_point_publisher.publish(lookahead_pt_msg)

        projected_pt_msg = PointStamped()
        projected_pt_msg.header.stamp = self.get_clock().now().to_msg()
        projected_pt_msg.point.x = projected_pt[0]
        projected_pt_msg.point.y = projected_pt[1]
        projected_pt_msg.point.z = 0.0
        projected_pt_msg.header.frame_id = "map"
        self.target_point_publisher.publish(projected_pt_msg)

        return projected_pt

    def compute_speed(self, angle):
        abs_angle = np.abs(angle)
        
        #When the angle is important the speed is reduced
        if abs_angle >= 0.4:
            speed = 1.0
            self.lookahead = 0.5 * LOOKAHEAD
        elif abs_angle < 0.4 or abs_angle >= 0.2:
            speed = 1.5
            self.lookahead = 0.75 * LOOKAHEAD
        else:
            speed = 2.0
            self.lookahead = 1. * LOOKAHEAD
        return speed
    
    def odometry_callback(self, pose_msg):
        # Only do something if some waypoints have been received
        if len(self.waypoints) == 0:
            return

        #The location of the car is retrieved using the subscribed topic (here it is the odom topic)
        position = np.array([pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y])
        w, x, y, z = pose_msg.pose.pose.orientation.w, pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z
        heading = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y**2 + z**2))

        # based on the book "motion planning and feedback control techniques"
        # transform the target_waypoint from global frame to car frame
        target_waypoint = self.get_next_waypoint(position, heading)

        rot_matrix = np.array([[np.cos(heading), np.sin(heading)], [-np.sin(heading), np.cos(heading)]])
        goal_point_body = np.dot(rot_matrix, target_waypoint - position)

        theta = np.arctan2(goal_point_body[1], goal_point_body[0])
        curvature = 2 * np.sin(theta) / np.linalg.norm(goal_point_body)
        wheel_base = 0.3302
        steering = np.arctan(wheel_base * curvature)
        
        angle = np.clip(steering, -0.4189, 0.4189)
        speed = self.compute_speed(angle) * SPEED_FACTOR

        # Write the drive message
        drive_msg = AckermannDriveStamped()
        drive_msg.header.stamp = self.get_clock().now().to_msg()
        drive_msg.header.frame_id = "laser"
        drive_msg.drive.steering_angle = angle
        drive_msg.drive.speed = speed
        self.drive_publisher.publish(drive_msg)

        # Output the trajectory as a message (DEBUG)
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        for waypoint in self.waypoints:
            pose = PoseStamped()
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            path.poses.append(pose)
        self.debug_waypoints_publisher.publish(path)
            

def main():
    rclpy.init()
    pp = PurePursuit()
    rclpy.spin(pp)
if __name__ == '__main__':
    main()

