#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message

import numpy as np

import time as time
import sys
import threading
import yaml
import os
import sys
from ament_index_python.packages import get_package_share_directory


class TimerNode(Node):
    """
    A ROS2 node that tracks the time between messages on two specified topics.

    This node subscribes to two user-defined topics and calculates the time difference between incoming messages.
    It listens for keyboard input to display the average time difference and can reset to initial values.
    """
    def __init__(self):
        """
        Initializes the Timer node.
        - Logs that the node has started.
        - Loads parameters from the 'params.yaml' configuration file.
        - Declares and sets parameters for topic names, types, and minimum time difference.
        - Prints the tracking configuration to the console.
        - Subscribes to the specified topics with their respective message types.
        - Starts a background thread to listen for keyboard input.
        - Resets initial tracking variables.
        """
        super().__init__('timer')

        self.get_logger().info("Timer node started")
        self._initialize_parameters()
        self._setup_subscriptions()
        
        print("Tracking time between:")
        print(f"\tTopic 1 - Name: {self.get_parameter('topic1.name').value}")
        print(f"\tTopic 1 - Type: {self.get_parameter('topic1.type').value}")
        print(f"\tTopic 2 - Name: {self.get_parameter('topic2.name').value}")
        print(f"\tTopic 2 - Type: {self.get_parameter('topic2.type').value}")
        print(f"\tMaximum time difference for calculations: {self.get_parameter('max_time').value} milliseconds")
        
        
        # React to keyboard input to printout average time
        self.running = True
        self.thread = threading.Thread(target=self.keyboard_listener)
        self.thread.daemon = True
        self.thread.start()

        # Set initial values
        self.reset()

    def _initialize_parameters(self):
        """Load parameters from the parameter server."""
        # Path to the params.yaml file
        config = os.path.join(
           get_package_share_directory("timer"),
            'config',
            'params.yaml'
        )
        # Load parameters from params.yaml
        with open(config, 'r') as file:
            params = yaml.safe_load(file)['timer']['ros__parameters']

        # Declare parameters and set parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('topic1.name', params['topic1']['name']),
                ('topic1.type', params['topic1']['type']),
                ('topic2.name', params['topic2']['name']),
                ('topic2.type', params['topic2']['type']),
                ('max_time', params['max_time']),
            ]
        )
        self.set_parameters([
            rclpy.parameter.Parameter('topic1.name', rclpy.Parameter.Type.STRING, params['topic1']['name']),
            rclpy.parameter.Parameter('topic1.type', rclpy.Parameter.Type.STRING, params['topic1']['type']),
            rclpy.parameter.Parameter('topic2.name', rclpy.Parameter.Type.STRING, params['topic2']['name']),
            rclpy.parameter.Parameter('topic2.type', rclpy.Parameter.Type.STRING, params['topic2']['type']),
            rclpy.parameter.Parameter('max_time', rclpy.Parameter.Type.DOUBLE, params['max_time']),
        ])


    def _setup_subscriptions(self):
        """Create subscriptions to the relevant topics."""
        topic1 = self.get_parameter('topic1.name').value
        type1 = get_message(self.get_parameter('topic1.type').value)
        topic2 = self.get_parameter('topic2.name').value
        type2 = get_message(self.get_parameter('topic2.type').value)
        
        self.create_subscription(
            type1,
            topic1,
            self.callback_topic1,
            10
        )
        self.create_subscription(
            type2,
            topic2,
            self.callback_topic2,
            10
        )
        
    def callback_topic1(self, _) -> None:
        """Callback function to to track times when topic 1 is called."""
        
        # Only consider topic calls in order
        if self.last_called == 2:
            self.time1.append(time.time_ns())
            self.last_called = 1
            self.called1 = True
        
        # Check if both topics have been called in order
        if self.called1 and self.called2 and not self.ready:
            self.ready = True
            print("Both topics received in order, ready to calculate time difference...")

    def callback_topic2(self, _) -> None:
        """Callback function to to track times when topic 2 is called."""
        
        # Only consider topic calls in order
        if self.last_called == 1:
            self.time2.append(time.time_ns())
            self.last_called = 2
            self.called2 = True
        
        # Check if both topics have been called in order
        if self.called1 and self.called2 and not self.ready:
            self.ready = True
            print("Both topics received in order, ready to calculate time difference...")

    def compute_average_speed(self) -> None:
        length = min(len(self.time1), len(self.time2))

        # Calculate time differnce only when times have been logged
        if length > 0:
            # Assure both lists have the same length
            time1 = np.array(self.time1[:length])
            time2 = np.array(self.time2[:length])

            deltas = (time2 - time1) / 1e6  # Convert nanoseconds to milliseconds
            small_diff = deltas < self.get_parameter('max_time').value  # Indizes of small distances

            # Calculate average time difference if there are any small differences
            if sum(small_diff) > 0:
                average_delta = np.mean(deltas[small_diff])
                if self.get_parameter('topic1.name').value == self.get_parameter('topic2.name').value:
                    print(f'\tFrequency of topic: {1 / (average_delta / 1000):.6f} Hz')
                else:
                    print(f'\tAverage time difference between topics: {average_delta:.6f} ms')
            print(f"\tTotal number of intervals: {length}")
            print(f"\tNumber of considered intervals: {np.sum(small_diff)}")
        else:
            print("\tNot enough data to calculate average time difference")
        
        self.reset()
        
    def keyboard_listener(self) -> None:
        """Listen for keyboard input to display the average time difference and reset values."""
        while self.running:
            key = sys.stdin.read(1)
            if key:
                self.compute_average_speed()
                
    def reset(self) -> None:
        """Reset tracking variables to their initial values."""
        self.time1 = []
        self.time2 = []
        self.received_topic_2 = False
        self.called1 = False
        self.called2 = False
        self.ready = False
        self.last_called = 2
        self.total_time = 0.0
        self.message_count = 0
        print("\nListening for topics")


    def destroy_node(self) -> None:
        # Stop the keyboard listener thread
        self.running = False
        super().destroy_node()

            
def main(args=None):
    rclpy.init(args=args)
    node = TimerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nExiting ...")
    finally: # Cleanly shutdown node on Crtl+C
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
