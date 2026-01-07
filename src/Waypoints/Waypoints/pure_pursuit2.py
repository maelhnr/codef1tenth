
#!/usr/bin/env python

import rclpy
from geometry_msgs.msg import PoseStamped, PointStamped, Pose
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive
from nav_msgs.msg import Odometry, Path
from tf2_ros import TransformBroadcaster
import numpy as np
import math as mp
from rclpy.node import Node



class PurePursuit2(Node):       
    """
    The class that handles the enhanced pure pursuit.
    """
    
    # Constructor:

    def __init__(self, waypoint_topic='/waypoints'):                
        #sbscribe to /pf/pose/odom for particle filter pose
        
        super().__init__('Pure_pursuit')                            
                                                                    
        
        

        # Attributes:
        
        #The waypoint list is read from a topic and stored in the "waypoints" variable
        self.waypoints = []
        
        
        # csv load
        #self.waypoints = np.genfromtxt('src/Waypoints/Waypoints/waypoints2.csv', delimiter=',')       
        #Only the first two columns are kept (x and y position) 
        #self.waypoints = self.waypoints[:,0:2]
        #The end of the list is removed
        #self.waypoints = self.waypoints[0:int(self.waypoints.shape[0]*0.90),:]
        

        self.lookahead = 1 # const
        self.print_info = True



        #Topics definition and initialisation
        pf_odom_topic = 'ego_racecar/odom'      # str 
        drive_topic = '/drive'
        #waypoint_topic = '/waypoint'           
        self.get_waypoints = self.create_subscription(Path, waypoint_topic, self.load_waypoints, 1)
        self.odom_sub = self.create_subscription(Odometry, pf_odom_topic, self.pose_callback1, 1)  
        self.drive_pub = self.create_publisher(AckermannDriveStamped,drive_topic,1)
        #self.waypoint_pub = self.create_publisher(PointStamped, waypoint_topic,1)
        
        self.last_idx = 0




                                                                
    def load_waypoints(self, msg):                             
        """Load waypoints sent by Astar Planifier"""           
        self.waypoints = []                                     
        for elt in msg.poses:                                   
            x = elt.pose.position.x
            y = elt.pose.position.y
            self.waypoints.append([x, y])                       

        self.waypoints = np.array(self.waypoints)               

        # if len(self.waypoints) > 1:
        #     waypoints_increased = []
        #     for i in range(len(self.waypoints) - 1):
        #         x1, y1 = self.waypoints[i]
        #         x2, y2 = self.waypoints[i + 1]

        #         # First WP
        #         waypoints_increased.append([x1, y1])

        #         # Computing the middle extra WP
        #         xm = (x1 + x2) / 2
        #         ym = (y1 + y2) / 2

        #         # Middle extra WP
        #         waypoints_increased.append([xm, ym])

        #     # Last WP
        #     waypoints_increased.append(self.waypoints[-1].tolist())

        #     self.waypoints = np.array(waypoints_increased)
        





    def get_target_waypoint(self, location):
        """Choisit le waypoint le plus proche dans la liste de waypoints à atteindre"""

        stop = False
        horizon = 3 
        min_dist=10000
        #self.last_idx = 0

        if len(self.waypoints) > 0 :
            
            #stop is true if the closest waypoint (from the car) is the last waypoint of the list
            # stop = False
            
               
            if len(self.waypoints) > horizon:
                self.next_WPs = self.waypoints[0:horizon] # next waypoints in the list
            else:
                self.next_WPs = self.waypoints
            
            #The distance between the car and all the waypoints is computed
            dist = [np.sqrt((self.next_WPs[i][0]-location[0])**2+(self.next_WPs[i][1]-location[1])**2) for i in range(len(self.next_WPs))] 
            
            #This loop will determine what is the index of the closest waypoint
            
            
            best_idx = self.last_idx
            for i in range(self.next_WPs.shape[0]-1, -1,-1):
                if dist[i] < min_dist:
                    best_idx = i
                    min_dist = dist[i]
            self.last_idx = best_idx
            #print(self.last_idx)


            #Index of the closest waypoint
            waypoint_pre = self.next_WPs[self.last_idx]  
            
            if self.last_idx+1 > self.waypoints.shape[0]-1:
                stop = True  


            #If this is the last waypoint of the list, the next waypoint is set to 0, and stop is set to True
            # if self.last_idx+1>self.waypoints.shape[0]-1:
            #     waypoint_post=0.0
            #     stop=True
                    
            # else:
            #     waypoint_post = self.waypoints[self.last_idx+1]
            
            #The target waypoint is an average of the closest and the one after 
            #waypoint = (waypoint_pre+waypoint_post)/2 #average

            targetWaypoint = waypoint_pre

            
            if min_dist < 0.05:
                if self.waypoints.shape[0] > 1:  
                    self.waypoints = np.delete(self.waypoints, self.last_idx, axis=0)
                else:
                    self.waypoints = np.array([])  

            
            return targetWaypoint, stop
        







    def set_speed(self, angle):
        abs_angle = np.abs(angle)
        
        # When the angle is important the speed is reduced
        if abs_angle >= 0.4:
            speed = 1.0
            self.lookahead = 0.5
        elif abs_angle < 0.4 or abs_angle >= 0.2:
            speed = 1.5
            self.lookahead = 0.75
        else:
            speed = 2.0
            self.lookahead =1
        return speed
    


    def quaternion_to_rotation_matrix(self, q):
        #Convert quaternion to 3x3 rotation matrix.
    
        x, y, z, w = q
        norm = (x**2 + y**2 + z**2 + w**2)**0.5
        s = 2.0 / norm

        rot_matrix = [
	    [1 - s*(y**2 + z**2), s*(x*y - z*w), s*(x*z + y*w)],
	    [s*(x*y + z*w), 1 - s*(x**2 + z**2), s*(y*z - x*w)],
	    [s*(x*z - y*w), s*(y*z + x*w), 1 - s*(x**2 + y**2)]]
        
        return rot_matrix
        


    
        


        
    def pose_callback1(self, pose_msg):        

        if len(self.waypoints) > 0 :
        
            #The location of the car is retrieved using the subscribed topic (here it is the odom topic)
            location = [pose_msg.pose.pose.position.x, pose_msg.pose.pose.position.y]
            
            #Transformation from cartesian to quaternion
            quaternion = [pose_msg.pose.pose.orientation.x, pose_msg.pose.pose.orientation.y, pose_msg.pose.pose.orientation.z, pose_msg.pose.pose.orientation.w]
            
            #Computation of the rotation matrix from the quaternion
            rot_matrix = self.quaternion_to_rotation_matrix(quaternion)
            
            # Waypoint of interest is retrieved using: the 'get_next_waypoint' method or 'get_curr_waypoint':
            #target_waypoint, stop = self.get_next_waypoint(location)
            target_waypoint, stop = self.get_target_waypoint(location)

            if self.print_info :
                print("position = ", location)
                print("target = ", target_waypoint, "\n")
            
            #Case where this is not the last waypoint
            if not stop:
                #Initialisation of the waypoint message
                waypoint_msg = PointStamped()
                waypoint_msg.header.stamp = self.get_clock().now().to_msg()
                waypoint_msg.header.frame_id = "map"
                waypoint_msg.point.x = target_waypoint[0]
                waypoint_msg.point.y = target_waypoint[1]
                waypoint_msg.point.z = 0.0
                
                #Publication of the waypoint message
                #self.waypoint_pub.publish(waypoint_msg)


                goal_point_world = target_waypoint-location
                #Vector from the car to the target waypoint in the world frame
                goal_point_world = np.append(goal_point_world, 0)
                
                #Vector from the car to the target waypoint in the car frame
                goal_point_body = np.matmul(np.linalg.inv(rot_matrix), goal_point_world)
                
                #print(goal_point_body)
            
            #tan(angle)=goal_point_body_y/(goal_point_body_x*factor²)
            #The tangente is linearly approximated, with a correction factor for big angles (lookahead)
                angle = mp.atan2(goal_point_body[1], goal_point_body[0])
                
                #print(angle)
                
                # rospy.loginfo("Dgoal: {}, Desired angle: {}".format(goal_point_body[0:2], angle))


                
                angle = np.clip(angle, -0.4189, 0.4189)
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = self.get_clock().now().to_msg()
                drive_msg.header.frame_id = "laser"
                drive_msg.drive.steering_angle = angle
                drive_msg.drive.speed = self.set_speed(angle)
                
                
                
            #The last way-point has been retrieved ==> the car is stopped
            else:
                drive_msg = AckermannDriveStamped()
                drive_msg.header.stamp = self.get_clock().now().to_msg()
                drive_msg.header.frame_id = "laser"
                drive_msg.drive.steering_angle = 0.0
                drive_msg.drive.speed = 0.0
            self.drive_pub.publish(drive_msg)
            




def main():
    rclpy.init()
    pp = PurePursuit2()
    rclpy.spin(pp)              

if __name__ == '__main__':
    main()