from enum import Enum, auto
import numpy as np

import rclpy
from rclpy.node import Node

from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Bool, Float64
from custom_msgs.msg import TrackDescription, LapProgress

# Input topics
PURE_PURSUIT_TOPIC = '/pure_pursuit'
DISPARITY_EXTENDER_TOPIC = '/disparity_extender'
LAP_PROGRESS_TOPIC = '/lap_progress'
TRACK_DESCRIPTION_TOPIC = '/track_description'
BRAKE_TOPIC = '/commands/motor/brake'

# Output topics
DRIVE_TOPIC = '/drive'
TRACK_DESCRIPTION_TRIGGER_TOPIC = '/centerline_trigger'

class StrategyState(Enum):
    """
    Enumeration of possible states for the basic strategy manager node.
    
    States represent different operating modes and conditions that determine
    how the node processes and responds to incoming messages.
    
    States:
    - INITIALIZING: Initial state when the node starts up
    - WAITING_FOR_TRACK: Waiting to receive track description
    - RACING_WITHOUT_TRACK: Racing without a computed trajectory, using the disparity_extender or similar
    - RACING_ON_TRAJECTORY: Following the previously computed trajectory
    - ERROR: Error state when unexpected conditions occur
    - FINISHED: Strategy completed, we stop the vehicle
    """
    
    # Initialization states
    INITIALIZING = auto()
    WAITING_FOR_TRACK = auto()

    # Racing states
    RACING_WITHOUT_TRACK = auto()
    RACING_ON_TRAJECTORY = auto()
    
    # Special states
    ERROR = auto()
    FINISHED = auto()
    # PAUSED = auto()
    
    def is_racing_state(self):
        """Helper method to check if the current state is any racing state."""
        return self in (StrategyState.RACING_WITHOUT_TRACK, StrategyState.RACING_ON_TRAJECTORY)

class BasicStrategyManager(Node):
    def __init__(self):
        super().__init__('basic_strategy_manager_node')

        # Variables
        self.state = StrategyState.INITIALIZING
        self.current_lap = None
        self.laps_before_finish = (9999, 0.)
        self.first_drive_received = False

        # Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('number_of_laps_on_trajectory', 1),
                ('laps_before_track_request', 1.33),
                ('track_request_retries', 0.33),
            ]
        )

        self.number_of_laps_on_trajectory = self.get_parameter('number_of_laps_on_trajectory').get_parameter_value().integer_value # The robot will complete its strategy after ceil(current_lap + number_of_laps_on_trajectory) after the trajectory is successfully received
        self.laps_before_track_request = split_integer_and_fractional(self.get_parameter('laps_before_track_request').get_parameter_value().double_value) # (lap_number, fractionnal_part)
        self.track_request_retries = split_integer_and_fractional(self.get_parameter('track_request_retries').get_parameter_value().double_value) # Retry every (lap_number, fractionnal_part) if the track description computation failed

        # Topics
        self.create_subscription(AckermannDriveStamped, PURE_PURSUIT_TOPIC, self.pure_pursuit_callback, 1)
        self.create_subscription(AckermannDriveStamped, DISPARITY_EXTENDER_TOPIC, self.disparity_extender_callback, 1)
        self.create_subscription(LapProgress, LAP_PROGRESS_TOPIC, self.lap_progress_topic, 1)
        self.create_subscription(TrackDescription, TRACK_DESCRIPTION_TOPIC, self.track_description_callback, 1)

        self.drive_publisher = self.create_publisher(AckermannDriveStamped, DRIVE_TOPIC, 1)
        self.brake_publisher = self.create_publisher(Float64, BRAKE_TOPIC, 1)
        self.track_trigger = self.create_publisher(Bool, TRACK_DESCRIPTION_TRIGGER_TOPIC, 1)

        self.set_new_state(StrategyState.INITIALIZING)
    
    def pure_pursuit_callback(self, msg: AckermannDriveStamped):
        self.first_drive_received = True
        if self.state == StrategyState.RACING_ON_TRAJECTORY:
            self.drive_publisher.publish(msg)
        self.update_state()

    def disparity_extender_callback(self, msg: AckermannDriveStamped):
        self.first_drive_received = True
        if self.state == StrategyState.RACING_WITHOUT_TRACK:
            self.drive_publisher.publish(msg)
        self.update_state()
    
    def lap_progress_topic(self, msg: LapProgress):
        self.current_lap = (msg.lap_number, msg.lap_progress)
        self.update_state()

    def track_description_callback(self, msg: TrackDescription):
        if self.state != StrategyState.WAITING_FOR_TRACK:
            self.get_logger().error('Received unexpected track description message')
            return
        
        if not msg.success:
            self.get_logger().warn(f'Track description computation failed: retrying in {self.track_request_retries[0] + self.track_request_retries[1]:.2f} laps')
            self.laps_before_track_request = add_laps_with_frac_part(self.laps_before_track_request, self.track_request_retries)
            self.set_new_state(StrategyState.RACING_WITHOUT_TRACK)
            return
        
        self.get_logger().info('Track description received')
        self.set_new_state(StrategyState.RACING_ON_TRAJECTORY)
        
    def set_brakes(self, brake: True):
        if brake:
            ack_msg = AckermannDriveStamped()
            ack_msg.header.stamp = self.get_clock().now().to_msg()
            self.drive_publisher.publish(ack_msg)
        
        # Only useful on the actual car
        brk_msg = Float64() 
        brk_msg.data = 20_000. if brake else 0.
        self.brake_publisher.publish(brk_msg)


    def set_new_state(self, new_state: StrategyState):
        if self.state == StrategyState.ERROR:
            self.get_logger().error(f'Cannot transition from error state (to {new_state.name})')
            return 
        
        old_state = self.state
        self.get_logger().info(f'State transition: {self.state.name} -> {new_state.name}')
        self.state = new_state
        
        # Update the brakes
        self.set_brakes(not self.state.is_racing_state()) # Always set the brakes in non-racing states, maybe update in the future ?

        # Update the laps before finish
        if old_state == StrategyState.WAITING_FOR_TRACK and new_state == StrategyState.RACING_ON_TRAJECTORY:
            max_laps = np.ceil(self.current_lap[0] + self.current_lap[1] + self.number_of_laps_on_trajectory)
            self.laps_before_finish = (max_laps, 0.)

            self.get_logger().info(f'Will finish after a total of {int(max_laps)} laps')

    def update_state(self):
        if self.state == StrategyState.INITIALIZING: 
            # Waiting on variables initialization
            if self.current_lap is not None and self.first_drive_received:
                # If we're ready, start racing without a trajectory
                self.set_new_state(StrategyState.RACING_WITHOUT_TRACK)

        elif self.state == StrategyState.RACING_WITHOUT_TRACK:
            # Detect if we need to request the new trajectory
            if self.current_lap > self.laps_before_track_request:
                self.get_logger().info('Requesting track description...')
                self.set_new_state(StrategyState.WAITING_FOR_TRACK)
                self.track_trigger.publish(Bool(data=True))

        elif self.state == StrategyState.RACING_ON_TRAJECTORY:
            # Detect if we stop the vehicle
            if self.current_lap > self.laps_before_finish:
                self.get_logger().info('Completed the strategy, stopping...')
                self.set_new_state(StrategyState.FINISHED)

        # if self.state == StrategyState.INITIALIZING:
        #     self.state = StrategyState.WAITING_FOR_TRACK
        #     self.get_logger().info('Waiting for track description...')
        #     self.track_trigger.publish(Bool(data=True))
        # elif self.state == StrategyState.WAITING_FOR_TRACK:
        #     if self.current_lap[0] > self.laps_before_track_request[0] or self.current_lap[1] > self.laps_before_track_request[1]:
        #         self.state = StrategyState.ERROR
        #         self.get_logger().warn('Failed to receive track description in time')
        # elif StrategyState.is_racing_state(self.state):
        #     if self.current_lap[0] >= self.max_number_of_laps:
        #         self.state = StrategyState.ERROR
        #         self.get_logger().info('Finished all laps')

def add_laps_with_frac_part(lap1, lap2):
    total = lap1[0] + lap2[0] + lap1[1] + lap2[1]
    return split_integer_and_fractional(total)

def split_integer_and_fractional(total):
    return int(total), total - int(total)

def main():
    rclpy.init()
    node = BasicStrategyManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

