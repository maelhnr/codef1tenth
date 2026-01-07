#!/usr/bin/env python

import math
import threading

import numpy as np
import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from typing import Tuple

"""
Code inspired by the post https://www.nathanotterness.com/2019/04/the-disparity-extender-algorithm-and.html  -->  Disparity extender strategy
"""

# Topics used
DRIVE_TOPIC = "/drive"
SCAN_TOPIC = "/scan"
DISPARITY_TOPIC = "/treated_scan"


class DisparityExtenderRace(Node):
    """
    This node aims to find the most distant reachable point in the LIDAR scan within the
    specified angle range, to drive towards. Since the car has a width, we can not simply 
    take the most distant point, because the cars side might crash into a wall before 
    reaching the target point. In order to find the furthes reachable point disparities are 
    extended. This means that when the distance to 2 consecutive measurements differs by more 
    than the disparity threshold, the closer measurement is extended by half the car width. 
    
    The algorithm looks for the furtherst point, then extends the disparities around that 
    point, then looks for the new furthest point and repeats the process until furthest point 
    stays the same, i.e. no disparities are extended anymore."""

    def __init__(self):
        """Initializes node
        Units:
        - distances: meters
        - angles: rad (prints use degrees for better readibility, but internally rad is used)"""
        super().__init__("fast_disparity_extender")

        print("\nInitializing fast disparity extender...")

        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("publish_disparities", False),
                # Params for speed calculation
                ("min_speed", 0.8),
                ("max_speed", 4.0),
                ("absolute_max_speed", 6.5),
                ("min_distance", 0.35),
                ("max_distance", 3.0),
                ("no_obstacles_distance", 6.0),
                ("speed_mode", "straight_ahead"),
                # Params for car
                ("half_car_width", 0.5),
                ("wheel_base", 0.325),
                # Params for disparity extender
                ("disparity_threshold", 0.2),
                ("scan_width", 270.0),
                ("target_mode", "all"),
                # Params for turning
                ("turn_clearance", 0.3),
                ("max_steering_angle", 24.0),
                ("min_steering_angle", 1.0),
                ("turn_factor", 0.0),
            ],
        )

        self.lidar_msg = None

        self._read_params()
        print("\nTarget mode:", self.target_mode)
        print("Speed mode:", self.speed_mode)

        # We'll use this lock to essentially drop LIDAR packets if we're still
        # processing an older one.
        self.lock = threading.Lock()
        self.should_stop = False
        self.total_packets = 0
        self.dropped_packets = 0

        self.lidar_distances = None  # measured lidar distances
        self.masked_distances = None  # distances with disparities extended
        # Other lidar parameters
        self.angle_increment = None
        self.num_samples = None
        self.center_idx = None

        # Disparities
        # Right disparities hold the disparities where the right value is smaller,
        # left disparities hold the disparities where the left value is smaller
        self.right_disparities = None
        self.left_disparities = None

        # Calculate distance normalization vector
        angles = np.linspace(-self.scan_width / 2, self.scan_width / 2, self.num_samples)
        self.distance_normalizer = 1 / (2*np.sin(angles))
        
        # Subscrition to LIDAR data and publisher for drive commands
        self.create_subscription(LaserScan, SCAN_TOPIC, self.lidar_callback, 10)
        self.drive_pub = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 10)
        print("\nSubscribed to LaserScan on topic", SCAN_TOPIC)
        print("Publishing AckermannDriveStamped on topic", DRIVE_TOPIC)
        if self.publish_disparities:
            self.disparity_pub = self.create_publisher(LaserScan, DISPARITY_TOPIC, 10)
            print("Publishing LaserScan with disparities on topic", DISPARITY_TOPIC)

        print("Node initialized")

        
    def _read_params(self) -> None:
        """Reads the parameters from the parameter server
        Args:
            None
        Returns:
            None
        """
        self.publish_disparities = (
            self.get_parameter("publish_disparities").get_parameter_value().bool_value
        )
        # Params for disparity extender ######################################
        self.wheel_base = (
            self.get_parameter("wheel_base").get_parameter_value().double_value
        )
        # This is actually "half" of the car width, plus some tolerance.
        # Controls the amount disparities are extended by.
        self.half_car_width = (
            self.get_parameter("half_car_width").get_parameter_value().double_value
        )
        # This is the difference between two successive LIDAR scan points that
        # can be considered a "disparity". (As a note, at 7m there should be
        # ~0.04m between scan points.)
        self.disparity_threshold = (
            self.get_parameter("disparity_threshold").get_parameter_value().double_value
        )
        # This is the arc width of the full LIDAR scan data (rad)
        self.scan_width = np.radians(
            self.get_parameter("scan_width").get_parameter_value().double_value
        )
        # This is the mode for selecting the target point
        self.target_mode = (
            self.get_parameter("target_mode").get_parameter_value().string_value
        )
        # Params for speed calculation #######################################
        self.min_speed = (
            self.get_parameter("min_speed").get_parameter_value().double_value
        )
        self.max_speed = (
            self.get_parameter("max_speed").get_parameter_value().double_value
        )
        self.absolute_max_speed = (
            self.get_parameter("absolute_max_speed").get_parameter_value().double_value
        )
        # The forward distance at which the car will go its minimum speed.
        # If there's not enough clearance in front of the car it will stop.
        self.min_distance = (
            self.get_parameter("min_distance").get_parameter_value().double_value
        )
        # The forward distance over which the car will go its maximum speed.
        # Any distance between this and the minimum scales the speed linearly.
        self.max_distance = (
            self.get_parameter("max_distance").get_parameter_value().double_value
        )
        # The forward distance above which the car will go its *absolute
        # maximum* speed. This distance indicates there are no obstacles in
        # the near path of the car. Distance between this and the max_distance
        # scales the speed linearly.
        self.no_obstacles_distance = (
            self.get_parameter("no_obstacles_distance")
            .get_parameter_value()
            .double_value
        )
        # The mode for selecting the speed
        self.speed_mode = (
            self.get_parameter("speed_mode").get_parameter_value().string_value
        )
        # Params for turning #################################################
        # This is the radius to the left or right of the car that must be clear
        # when the car is attempting to turn left or right.
        self.turn_clearance = (
            self.get_parameter("turn_clearance").get_parameter_value().double_value
        )
        # This is the maximum steering angle of the car (rad)
        self.max_steering_angle = np.radians(
            self.get_parameter("max_steering_angle").get_parameter_value().double_value
        )
        # This is the minimum steering angle of the car (rad)
        self.min_steering_angle = np.radians(
            self.get_parameter("min_steering_angle").get_parameter_value().double_value
        )
        self.turn_factor = (
            self.get_parameter("turn_factor").get_parameter_value().double_value
        )

    def lidar_callback(self, lidar_data:LaserScan) -> None:
        """This is asynchronously called every time we receive new LIDAR data
        0. Get LIDAR data from message
        1. Determine target range
        2. Find disparities
        3. Find target
        4. calculate safe movement
        5. publish drive

        Args:
            lidar_data : données Lidar reçues
        Returns:
            None
        """
        print("****************************************************")
        self.total_packets += 1
        # If the lock is currently locked, then previous LIDAR data is still
        # being processed
        if not self.lock.acquire(False):
            self.dropped_packets += 1
            return
        if self.should_stop:
            return

        # Get the LIDAR data ################################################
        self.lidar_distances = np.array(lidar_data.ranges)
        self.angle_increment = lidar_data.angle_increment
        self.scan_width = lidar_data.angle_max - lidar_data.angle_min

        self.lidar_msg = lidar_data

        # lidar_data.header.frame_id = "/laser"
        # self.disparity_pub.publish(lidar_data)
        
        # Treat the LIDAR data ###############################################
        self.masked_distances = np.copy(self.lidar_distances)
        self.num_samples = len(self.lidar_distances)
        # self.angle_increment = self.num_samples / self.scan_width  # (rad)
        self.center_idx = int(self.num_samples // 2)

        # 1. Determine target range ##########################################
        target_range = self.find_target_range()

        # 2. Find disparities ###############################################
        # Save backup of disparities to restore them if needed
        self.right_disparities_backup, self.left_disparities_backup = self.find_disparities()
        self.right_disparities = np.copy(self.right_disparities_backup)
        self.left_disparities = np.copy(self.left_disparities_backup)

        # 3. Find target ####################################################
        target_idx = self.find_target(target_range)
        target_angle = self.angle_from_index(target_idx)

        if self.publish_disparities:
            self.publish_disparity_msg(self.masked_distances)

        # 4. Calculate safe movement ########################################
        # Use unmasked data to determine save speeds
        if self.speed_mode == "straight_ahead":
            desired_speed, drive_angle = self.move_straight_ahead(
                target_idx, target_angle
            )
        elif self.speed_mode == "arc_distance":
            desired_speed, drive_angle = self.move_arc_distance(
                target_idx, target_angle
            )
        self.lock.release()

        # 5. Publish drive ##################################################
        self.publish_drive_command(desired_speed, drive_angle)

    def find_target_range(self) -> Tuple[int, int]:
        """Ajusts the considered samples to search only in specific region

        TODO: Implement a heuristic to determine the target range, for example
        based on a map/track

        For now, only samples from -90 to 90 degrees are considered

        Args:
            None
        Returns:
            target_range (tuple[int,int]) : the start and end indices of the considered samples
        """
        return (0, self.num_samples - 1)

    def find_disparities(self) -> Tuple[np.array, np.array]:
        """Finds left and right disparities in the LIDAR data.
        Calculates where the distance between lidar measurements (disparities) are greater than the threshold.
        Arrays contain the indices of the smaller value of the disparity.

        If the left value is smaller: left disparity
        If the right value is smaller: right disparity

        Args:
            None
        Returns:
            right_disparities (np.array) : indices of the right disparities
            left_disparities (np.array) : indices of the left disparities
        """
        # Lidar measurements go from right to left
        diffs = np.diff(self.lidar_distances)

        right_disparities = np.where(diffs > self.disparity_threshold)[0]
        left_disparities = np.where(diffs < -self.disparity_threshold)[0] + 1
        return right_disparities, left_disparities

    def find_target(self, target_range: Tuple[int, int]) -> int:
        """Finds the target to follow.
        Expands disparities around the best candidate (the furthest measurment)
        until the best candidate is the furthest point considering the disparities

        Args:
            target_range (Tuple[int,int]) : the start and end indices of the considered samples
        Returns:
            target (int) : the index of the target to follow
        """
        # Using a subset of masked_distances to make the search faster, tr_<name> are variables in the subset range
        start_tr, end_tr = target_range
        tr_masked_distances = self.masked_distances[start_tr:end_tr]

        tr_candidate = np.argmax(tr_masked_distances)
        candidate = start_tr + tr_candidate
        while self.expand_disparities_around_candidate(candidate, self.half_car_width):
            tr_candidate = np.argmax(tr_masked_distances)
            candidate = start_tr + tr_candidate

        return candidate

    def expand_disparities_around_candidate(self, candidate: int, expand_width:float) -> None:
        """Expands disparities around the candidate.
        Also considers samples outside the bounds set by self.considered_samples_idx
        to avoid crashes.
        Args:
            candidate (int) : the index of the candidate to expand disparities around
        Returns:

        """
        lidar_distances = self.lidar_distances
        right_disparities = self.right_disparities
        left_disparities = self.left_disparities

        idx_right_disp = np.searchsorted(right_disparities, candidate, side="left") - 1
        idx_left_disp = np.searchsorted(left_disparities, candidate, side="right")

        disparity_extended = False

        # Iterate over disparities to the right of the candidate, starting from the closest
        while idx_right_disp >= 0: # Stop if we reach the beginning of the array
            disparity = right_disparities[idx_right_disp]
            distance = lidar_distances[disparity]
            samples_to_extend = self.samples_at_distance(distance, expand_width)
            # If the disparity is too far away to affect candidate, stop expanding
            if disparity + samples_to_extend < candidate:
                break
            self.expand_disparity(disparity, distance, samples_to_extend, direction=1)
            disparity_extended = True
            right_disparities = np.delete(right_disparities, idx_right_disp)
            idx_right_disp -= 1
        self.right_disparities = right_disparities

        # Iterate over disparities to the left of the candidate, starting from the closest
        while idx_left_disp < len(left_disparities): # Stop if we reach the end of the array
            disparity = left_disparities[idx_left_disp]
            distance = lidar_distances[disparity]
            samples_to_extend = self.samples_at_distance(distance, expand_width)
            # If the disparity is too far away to affect candidate, stop expanding
            if disparity - samples_to_extend > candidate:
                break
            self.expand_disparity(disparity, distance, samples_to_extend, direction=-1)
            disparity_extended = True
            left_disparities = np.delete(left_disparities, idx_left_disp)
            # here idx_left_disp can stay the same because the value at the next higher
            # index slides down to the current index when deleting#
        self.left_disparities = left_disparities

        return disparity_extended

    def samples_at_distance(self, distance:float, expand_width:float) -> int:
        """Returns the number of points in the LIDAR scan that will cover half of
        the width of the car along an arc at the given distance

        Args:
            distance (float): the closest distance of the disparity
            expand_width (float): the width to expand the disparities

        Returns:
            number (int): the number of lidar points that will cover half of the car width at the given distance
        """
        # This isn't exact, because it's really calculated based on the arc length
        # when it should be calculated based on the straight-line distance.
        # However, for simplicty we can just compensate for it by inflating the
        # "car width" slightly
        distance_between_samples = distance * self.angle_increment
        return int(math.ceil(self.half_car_width / distance_between_samples))

    def expand_disparity(
        self, disparity: int, distance: float, samples_to_extend: int, direction: int
    ) -> None:
        """Expands the disparity at the given index.

        Args:
            disparity (int) : the index of the disparity to expand
            distance (float) : the distance of the disparity
            samples_to_extend (int) : the number of samples to extend the disparity by
            direction (int) : the direction to extend the disparity in (1 for forward, -1 for backward)
        Returns:
            None
        """
        # Access the local lidar distances and local masked distances
        if direction == 1:
            start_idx = disparity
            end_idx = disparity + samples_to_extend + 1
        elif direction == -1:
            start_idx = disparity - samples_to_extend
            end_idx = disparity + 1
        local_masked_distances = self.masked_distances[start_idx:end_idx]

        # Expand the disparity:
        # Set all values to the distance of the disparity, if they are larger
        local_masked_distances[local_masked_distances > distance] = distance


    def publish_disparity_msg(self, masked_scan: np.array) -> None:
        message = self.lidar_msg
        message.ranges = masked_scan.astype(float).tolist()
        
        self.disparity_pub.publish(message)


    def move_straight_ahead(
        self, target_idx: int, target_angle: float
    ) -> Tuple[float, float]:
        """Calculates the safe movement to the target.
        Idea: Determine closest point on the slice of a circle between the car and the target point and drive to avoid that point.

        Args:
            target_idx (int) : the index of the target to follow
            target_angle (float) : the angle to the target
        Returns:
            desired_speed (float) : the desired speed (m/s)
            steering_angle (float) : the steering angle (rad)
        """
        center_idx = self.center_idx
        
        print("Determining straight ahead speed")
        # Get forward distance considering close obstacles
        self.masked_distances = np.copy(self.lidar_distances)
        self.right_disparities = np.copy(self.right_disparities_backup)
        self.left_disparities = np.copy(self.left_disparities_backup)

        self.expand_disparities_around_candidate(center_idx, 0.85*self.half_car_width)
        forward_distance = self.masked_distances[center_idx]

        # Get speed based on forward distance
        desired_speed = self.cal_forward_speed(forward_distance)

        # Get steering angle
        steering_angle = self.adjust_angle_for_car_side(
            target_angle
        )  # Avoid turning into walls

        steering_angle = self.adjust_small_or_large_angles(
            steering_angle
        )  # Adjust for too small or too large angles

        steering_angle = self.adjust_angle_for_speed(
            steering_angle, desired_speed
        )  # Adjust for speed

        print("\nTarget point", target_idx)
        print("Center point", self.center_idx)
        print("Target distance", self.masked_distances[target_idx])
        print("Forward distance", forward_distance)
        print("Velocity", desired_speed)
        print("Target angle", target_angle * 180 / np.pi)
        print("Steering angle", steering_angle * 180 / np.pi)

        return desired_speed, steering_angle
    
    def move_arc_distance(self, target_idx: int, target_angle: float) -> Tuple[float, float]:
        """Calculates movement to avoid closest obstacle on the arc between the car and the target
        
        Args:
            target_idx (int) : the index of the target to follow
            target_angle (float) : the angle to the target
        Returns:
            desired_speed (float) : the desired speed (m/s)
            steering_angle (float) : the steering angle (rad)
        """
        masked_distances = self.masked_distances
        target_distance = masked_distances[target_idx]
        center_idx = self.center_idx
        min_turn_radius = self.get_turn_rad_from_steer_angle(self.min_steering_angle)
        # Get closeset point idx and angle
        # if far enough go for it

        
        # Max safe turn radius based on lidar distances
        # distance_normalizer is used to map lidar distances to turn radii, that would be necessary to reach them.
        # Then the minimum turn radius can be determined. From this the half_car_width is substracted to account for the calculations
        # beeing based on the cars center
        if center_idx < target_idx:
            normalized_distances = masked_distances[center_idx:target_idx] * self.distance_normalizer[center_idx:target_idx]
        else:
            normalized_distances = masked_distances[target_idx:center_idx] * self.distance_normalizer[target_idx:center_idx]
        max_safe_radius = np.min(normalized_distances) - self.half_car_width

        if max_safe_radius > min_turn_radius:
            return 0, 0 # Stop the car
        
        # Max turn radius based on distance and angle to target
        max_target_radius = target_distance / (2* abs(np.sin(target_angle)))
        
        max_turn_radius = np.min([max_safe_radius, max_target_radius])

        turn_radius = min_turn_radius + (max_turn_radius-min_turn_radius) * self.turn_factor
        steer_angle = self.get_steer_angle_from_turn_rad(turn_radius)

        if steer_angle < self.min_steering_angle: # Remove noise from steering
            steer_angle = 0

        # Speed calculation ##############
        # Max speed based on forward distance
        forward_speed = self.cal_forward_speed(masked_distances[center_idx])
        # Max speed based on turn radius



        
        # Good steering angle

        # Get the distance to the target
        

        # Get drive_disparity around closest point (perhaps only in turn direction)

        # Get new closest point idx and angle   

        # Cal drive angle to avoid closest point

        # Cal speed based on drive angle

        # Safety check for speed

    def get_steer_angle_from_turn_rad(self, turn_radius: float) -> float:
        """Calculates the steering angle for following a turn with a give radius
        Args:
            turn_radius (float) : turn_radius (m)
        Returns:
            steer_angle (float) : steering angle (rad)
        """
        return np.arcsin(self.wheel_base / turn_radius)

    def get_turn_rad_from_steer_angle(self, steer_angle: float) -> float:
        """Calculates the turn radius for a given steering angle
        Args:
            steer_angle (float) : steering angle (rad)
        Returns:
            turn_radius (float : turn radius (m)
        """
        return self.wheel_base / np.sin(steer_angle)


    def scale_speed_linearly(
        self, speed_low, speed_high, distance, distance_low, distance_high
    ) -> float:
        """Scales the speed linearly in [speed_low, speed_high] based on the
        distance value, relative to the range [distance_low, distance_high]
        Args:
            speed_low (float): minimal speed
            speed_high (float): maximal speed
            distance (float): the distance to the point where you want to go
            distance_low (float): minimum range
            distance_high (float): maximum range
        Returns:
            speed (float): the speed scaled
        """
        ratio = (distance - distance_low) / (distance_high - distance_low)
        speed_range = speed_high - speed_low
        return speed_low + (speed_range * ratio)

    def cal_forward_speed(self, forward_distance):
        """Takes a forward distance and returns a duty cycle value to set the
        car's velocity. Fairly unprincipled, basically just scales the speed
        directly based on distance, and stops if the car is blocked

        Args:
            distance (float): the distance to the point where you want to go

        Returns:
            speed (float): the speed scaled based on the distance
        """
        # Return min speed if we're too close
        if forward_distance <= self.min_distance:
            return self.min_speed

        # Return absolute max speed if we're far enough away
        if forward_distance >= self.no_obstacles_distance:
            return self.absolute_max_speed

        # Scale speed linearly between min and max distance
        if (
            forward_distance > self.min_distance
            and forward_distance <= self.max_distance
        ):
            return self.scale_speed_linearly(
                speed_low=self.min_speed,
                speed_high=self.max_speed,
                distance=forward_distance,
                distance_low=self.min_distance,
                distance_high=self.max_distance,
            )

        # Scale speed linearly between max and no_obstacles distance
        if (
            forward_distance > self.max_distance
            and forward_distance < self.no_obstacles_distance
        ):
            return self.scale_speed_linearly(
                speed_low=self.max_speed,
                speed_high=self.absolute_max_speed,
                distance=forward_distance,
                distance_low=self.max_distance,
                distance_high=self.no_obstacles_distance,
            )

    def adjust_angle_for_car_side(self, steering_angle: float) -> float:
        """Ajusts the the steering angle to avoid turning into walls.
        Sets steering angle to 0 if the there is a close wall beside or behind the sensor.

        Args:
            steering_angle (float) : planned steering_angle (rad)
        Returns:
            steering_angle (float) : steering_angle (rad) adjusted for close obstacles to the side
        """
        # Use shorter handles
        scan_wdith = self.scan_width
        laser_distances = self.lidar_distances
        angle_increment = self.angle_increment
        turn_clearance = self.turn_clearance

        angle_behind_car = (scan_wdith - np.pi) / 2
        samples_behind_car = int(angle_behind_car / angle_increment)

        # Check to the right
        if steering_angle < 0 and any(
            laser_distances[:samples_behind_car] < turn_clearance
        ):
            steering_angle = 0.0
            print("\nWall on the right")
        # Check to the left
        if steering_angle > 0 and any(
            laser_distances[-samples_behind_car:] < turn_clearance
        ):
            steering_angle = 0.0
            print("\nWall on the left")
        return steering_angle

    def adjust_small_or_large_angles(self, steering_angle: float) -> float:
        """Adjusts the steering angle to be within the limits of the car's
        steering capabilities. Sets small angles to 0.0 to avoid jitter

        Args:
            steering_angle (float): steering angle (rad)
        Returns:
            steering_angle (float): adjusted steering angle (rad)
        """
        # Set small angles to 0.0 to avoid jitter
        if abs(steering_angle) < self.min_steering_angle:
            steering_angle = 0.0
        # Set large angles to the maximum turn angle
        elif abs(steering_angle) > self.max_steering_angle:
            if steering_angle > 0:
                steering_angle = self.max_steering_angle
            else:
                steering_angle = -self.max_steering_angle

        return steering_angle

    def adjust_angle_for_speed(self, steering_angle: float, speed: float) -> float:
        """Set the steering factor as a function the speed of the car in order to improve the high speed performances (stability),
         avoiding to turn too much at high speed
        Args:
            steering_angle (float): steering angle (rad)
            speed (float): speed (m/s)
        Returns:
            steering_angle (float): adjusted steering angle (rad)
        """
        # TODO Improve the adjustment of the steering angle based on the speed
        if speed >= 4.0:
            scalar = 0.4
        else:
            scalar = 0.7
        return steering_angle * scalar

    def publish_drive_command(
        self, desired_speed: float, steering_angle: float
    ) -> None:
        """Publishes the AckermannDriveStamped command to the car
        Args:
            desired_speed (float): desired speed value (m/s) (positive for forward, negative for backward)
            steering_angle (float): steering angle value (°) (positive for left, negative for right)
        Returns:
            None
        """
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.speed = desired_speed
        drive_msg.drive.steering_angle = steering_angle
        self.drive_pub.publish(drive_msg)

    def angle_from_index(self, idx: int) -> float:
        """Returns the angle (rad), corresponding to index i in the
        LIDAR samples

        Args :
            idx (int) : the index in the LIDAR samples
        Returns :
            angle (float): the angle (rad) corresponding to the index
        """
        min_angle = -(self.scan_width / 2.0)
        return min_angle + idx * self.angle_increment

    def index_from_angle(self, angle: float) -> int:
        """Returns the closest index in the LIDAR samples corresponding to the
        angle in degrees

        Args:
            angle (float): the angle (rad) to a sample in the LIDAR scan
        Returns:
            idx (int): the closest index to the angle in the LIDAR samples
        """
        return self.center_idx + round(angle / self.angle_increment)

    def destroy_node(self) -> None:
        print("\nStopping the car...")
        self.publish_drive_command(0.0, 0.0)
        print("\nTotal packets received:", self.total_packets)
        print("Dropped packets:", self.dropped_packets)
        print("\nExiting...")
        self.should_stop = True
        if self.lock.locked():
            self.lock.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DisparityExtenderRace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()


"""
Original Code for reference
"""
# def extend_disparities(self):
#     """For each disparity in the list of distances, extends the nearest
#     value by the car width in whichever direction covers up the more-
#     distant points. Puts the resulting values in self.masked_disparities

#     """
#     lidar_distances = self.lidar_distances
#     number_of_samples = len(lidar_distances)
#     masked_disparities = np.copy(lidar_distances)
#     disparities = self.find_disparities()
#     # Keep a list of disparity end points corresponding to safe driving
#     # angles directly past a disparity. We will find the longest of these
#     # constrained distances in situations where we need to turn towards a
#     # disparity
#     self.possible_disparity_indices = []
#     print(f"Got {number_of_samples:d} disparities.")

#     # for d in disparities:
#     #     if lidar_distances[d] < lidar_distances[d + 1]:
#     #         start_dist = lidar_distances[d]
#     #         start_idx = d
#     #         extend_direction = 1  # Forward/positive
#     #     else:
#     #         start_dist = lidar_distances[d + 1]
#     #         start_idx = d + 1
#     #         extend_direction = -1  # Backward/negative

#     #     samples_to_extend = self.half_car_samples_at_distance(start_dist)

#     #     # End index based on the car width in samples and the exten direction
#     #     end_idx = start_idx + extend_direction * samples_to_extend
#     #     # Make sure we don't go out of bounds
#     #     if end_idx < 0:
#     #         end_idx = 0
#     #     elif end_idx >= number_of_samples:
#     #         end_idx = number_of_samples - 1

#     #     for idx in range(start_idx,end_idx):

#     for d in disparities:
#         a = lidar_distances[d]
#         b = lidar_distances[d + 1]
#         # If extend_positive is true, then extend the nearer value to
#         # higher indices, otherwise extend it to lower indices
#         nearer_value = a
#         nearer_index = d
#         extend_direction = True
#         if b < a:
#             extend_direction = False
#             nearer_value = b
#             nearer_index = d + 1
#         samples_to_extend = self.half_car_samples_at_distance(nearer_value)
#         current_index = nearer_index
#         for i in range(samples_to_extend):
#             # Stop trying to "extend" the disparity point if we reach the
#             # end of the array
#             if current_index < 0:
#                 current_index = 0
#                 break
#             if current_index >= len(masked_disparities):
#                 current_index = len(masked_disparities) - 1
#                 break
#             # Don't overwrite values if we've already found a nearer point
#             if masked_disparities[current_index] > nearer_value:
#                 masked_disparities[current_index] = nearer_value
#             # Finally, move left or right depending on the direction of the
#             # disparity
#             if extend_direction:
#                 current_index += 1
#             else:
#                 current_index -= 1
#         self.possible_disparity_indices.append(current_index)
#     self.masked_distances = masked_disparities
#     return


# def find_widest_disparity_index(self):
#     """Returns the index of the distance corresponding to the "widest"
#     disparity that we can safely target"""
#     masked_disparities = self.masked_distances
#     # Keep this at 0.1 so that we won't identify noise as a disparity
#     max_disparity = 0.1
#     max_disparity_index = None
#     for d in self.possible_disparity_indices:
#         # Ignore disparities that are behind the car
#         angle = self.angle_from_index(d)
#         if (angle < self.min_considered_angle) or (
#             angle > self.max_considered_angle
#         ):
#             continue
#         angle = d * self.samples_per_degree
#         distance = masked_disparities[d]
#         prev = distance
#         after = distance
#         # The disparity must have been extended from one of the two
#         # directions, so we can calculate the distance of the disparity by
#         # checking the distance between the points on either side of the
#         # index (note that something on the endpoint won't matter here
#         # either. The inequalities are just for bounds checking, if either
#         # one is outside the array, then we already know the disparity was
#         # extended from a different direction
#         if (d - 1) > 0:
#             prev = masked_disparities[d - 1]
#         if (d + 1) < len(masked_disparities):
#             after = masked_disparities[d + 1]
#         difference = abs(prev - after)
#         if difference > max_disparity:
#             max_disparity = difference
#             max_disparity_index = d
#     return max_disparity_index

# def find_new_angle(self):
#     """Returns the angle of the farthest possible distance that can be reached
#     in a direct line without bumping into edges. Returns the distance in meters
#     and the angle in degrees"""
#     self.extend_disparities()
#     limited_values = self.masked_distances
#     max_distance = -1.0e10
#     angle = 0.0
#     # Constrain the arc of possible angles we consider.
#     min_sample_index = int(self.index_from_angle(self.min_considered_angle))
#     max_sample_index = int(self.index_from_angle(self.max_considered_angle))
#     limited_values = limited_values[min_sample_index:max_sample_index]
#     for i in range(len(limited_values)):
#         distance = limited_values[i]
#         if distance > max_distance:
#             angle = self.min_considered_angle + float(i) / self.samples_per_degree
#             max_distance = distance
#     return distance, angle
