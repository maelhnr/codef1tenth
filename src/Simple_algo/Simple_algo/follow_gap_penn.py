#!/usr/bin/env python3

import sys
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node

# Topics used
SCAN_TOPIC = '/scan'
DRIVE_TOPIC = '/drive'

"""
	Le but de ce code est de faire tourner le noeud follow_gap qui envoie des commandes en fonction des données lidar en suivant le principe d'un 
	algorithme de suivis du plus grand écart.
"""
class ReactiveFollowGap(Node):
	"""
	Classe du noeud assiocié à l'algorithme de follow_gap
 
	Args:
		Node : classe générale de noeud héritée par la classe ReactiveFollowGap 
	"""
	def __init__(self):
		super().__init__('gap_follow')

        # Parameters
		self.declare_parameters(
			namespace='',
			parameters=[
				('bubble_radius', 1.5),  # Rayon de la bulle
				('max_speed', 3.0),  # Vitesse maximale de la voiture
				('steering_fox_degrees', 70.0),  # Angle de vue de la direction
			])
		
		self.bubble_radius = self.get_parameter('bubble_radius').value
		self.max_speed = self.get_parameter('max_speed').value
		self.steering_fox = self.get_parameter('steering_fox_degrees').value

		# Variables
		self.angles = None
		self.angles_initialized = False

		# Topics
		self.lidar_sub = self.create_subscription(LaserScan, SCAN_TOPIC, self.lidar_callback, 10)
		self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)

	def preprocess_lidar(self, data, window_size=7):
		""" Preprocess the LiDAR scan array. 

			Création de la liste des angles
  			Expert implementation includes:
				1.Setting each value to the mean over some window
				2.Rejecting high values (eg. > 3m)

		Args:
			data : message de type sensor_msgs/LaserScan.msg
			window_size : _description_. Defaults to 7.

		Returns:
			smoothed_ranges : des distances modifiées.
		"""
		
		#initialize data
		steering_viewport = self.steering_fox
		ranges = np.array(data.ranges)
		if not self.angles_initialized:
			min_angle = data.angle_min
			max_angle = data.angle_max
			self.angles = np.linspace(min_angle, max_angle, ranges.shape[0]) #bon nombre de points équidistants 
			self.angles_initialized = True
			self.good_angle_idx = np.where(np.logical_and(self.angles > np.radians(-steering_viewport), self.angles < np.radians(steering_viewport))) 
   			#prends seulement les angles faisables ?
			self.angles = self.angles[self.good_angle_idx]

		#set Nan's to 0
		ranges[np.isnan(ranges)] = 0
		goodIdx = np.isfinite(ranges)
		#set InF's to 5 (don't set to 0 so that we don't remove legitimate gaps)
		ranges[~goodIdx] = 5
		kernel = np.ones((1,window_size))/window_size
		kernel = kernel[0,:]
		smoothed_ranges = np.convolve(ranges,kernel,'same') # fait la convoltion avec une somme le log de toutes les ranges # effets de bords ?
		smoothed_ranges[~goodIdx] = 0 #remet les points à l'infini à 0 pq ???
		smoothed_ranges = np.clip(smoothed_ranges, 0, 3) #fait entrer toutes les valeurs entre 0 et 3
		smoothed_ranges = smoothed_ranges[self.good_angle_idx]

		return smoothed_ranges


	def find_max_gap(self, free_space_ranges):
		"""Return the start index & end index of the max gap in free_space_ranges

		Args:
			free_space_ranges : liste des distances associées au points lidar (avec preprocessing et bulles)

		Returns:
			_type_: l'indice du min et du max 
		"""
		temp_arr = np.copy(free_space_ranges)
		temp_arr[np.nonzero(temp_arr)] = 2 #pourquoi ???
		split = np.split(np.arange(free_space_ranges.shape[0]), np.where(np.abs(np.diff(temp_arr)) >= 1)[0]+1) #fait les séparations en segments sous forme
		# d'une liste d'indices
			
		sorted_split = sorted(split, key=len, reverse=True) #classe les segments par longeurs décroissantes
		for i in range(len(sorted_split)):
			if np.any(free_space_ranges[sorted_split[i]]): #
				return np.min(sorted_split[i]), np.max(sorted_split[i]+1)


	def find_best_point(self, start_i, end_i, ranges):
		"""Start_i & end_i are start and end indicies of max-gap range, respectively
		Return index of best point in ranges
		Naive: Choose the furthest point within ranges and go there

		Args:
			start_i : indice début du gap
			end_i : indice fin du gap
			ranges : liste des distances

		Returns:
			best_point: le meilleur angles dans le segment
		"""

		return self.angles[np.argmax(ranges[start_i:end_i])+start_i]


	def set_bubble(self, ranges, closest_point_idx, rb=1.5):
		"""
		Args:
			ranges : toutes les distances des points lidars
			closest_point_idx : indice du point le plus proche de la voiture.
			rb : rayon de la bulle (taille de la voiture ?)

		Returns:
			ranges : liste de toutes les distances des points radars
		"""
		angle = self.angles[closest_point_idx] #angle du point le plus proche
		dtheta = np.arctan2(rb, ranges[closest_point_idx]) #angles correpondant à la taille de la bulle

		bubble_idx = np.where(np.logical_and(self.angles > angle-dtheta, self.angles < angle+dtheta))#cherche les angles qui sont dans le bulle

		ranges[bubble_idx] = 0 #met à 0 car la voiture ne passe pas (dans la bulle)

		return ranges


	def lidar_callback(self, data):
		"""Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

		Args:
			data : données transmises par le topic lidar
		"""
		logger = self.get_logger()
		proc_ranges = self.preprocess_lidar(data)

		#Find closest point to LiDAR
		closest_point_idx = np.argmin(proc_ranges[np.nonzero(proc_ranges)]) #nonzero
		logger.info("closest_point_idx{}".format(closest_point_idx))
		
		#Eliminate all points inside 'bubble' (set them to zero) 

		bubbled_ranges = self.set_bubble(proc_ranges, closest_point_idx, self.bubble_radius)

		#Find max length gap
		gap_start, gap_end = self.find_max_gap(bubbled_ranges)
		logger.info("gap_start{}".format(gap_start, gap_end))
		
		#Find the best point in the gap 
		desired_angle = self.find_best_point(gap_start, gap_end,bubbled_ranges)

		#Publish Drive message
		drive_msg = AckermannDriveStamped()
		drive_msg.header.stamp = self.get_clock().now().to_msg()
		drive_msg.header.frame_id = "laser"
		drive_msg.drive.steering_angle = desired_angle
		drive_msg.drive.speed = self.max_speed #slow constant velocity for now
		
		
		logger.info("desired_angle {}".format(desired_angle))
		self.drive_pub.publish(drive_msg)


def main(args=None):
	rclpy.init(args=args)
	rfg = ReactiveFollowGap()
	rclpy.spin(rfg)
	rclpy.shutdown()

if __name__ == '__main__':
	main()

if __name__ == '__main__':
	main(sys.argv)
