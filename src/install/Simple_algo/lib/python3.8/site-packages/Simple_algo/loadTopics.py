#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


def load_topics(node):
    """ This function should be used to get a dictionary containing the topics of the car.
    This function takes into account if it is a simulation or not (specified in params.yaml)"""
    ros_topics = {}

    simulation = node.get_parameter("simulation").value
    ros_topics['simulation'] = simulation

    # Publishers and subscribers
    if simulation:
        simulation_topics = node.get_parameter("simulation_topics").value

        ros_topics['odom_topic'] = simulation_topics["odom_topic"]  # Use this topic to get odometry in simulation
        ros_topics['command_topic'] = simulation_topics["command_topic"]  # Use this topic to command directly the car in simulation
    else:
        car_topics = node.get_parameter("car_topics").value

        ros_topics['odom_topic'] = car_topics["odom_topic"]  # Use this topic to get odometry in the real car
        ros_topics['command_topic'] = car_topics["command_topic"]  # Use this topic to command directly the real car
        # ros_topics['command_topic'] = '/command_to_send' # Use this topic to send commands through a safety node

    return ros_topics
