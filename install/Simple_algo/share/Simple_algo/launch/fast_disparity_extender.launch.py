from launch import LaunchDescription
from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def launch_node_with_configuration(package, executable, name, mode, *args, **kwargs):
    """ Launch a node with the configuration corresponding to the specific mode (usually simulation or real) """
    config = os.path.join(
        get_package_share_directory(package),
        'config',
        f'{mode}.yaml'
    )
    return Node(
        package=package,
        executable=executable,
        name=name,
        parameters=[config],
        output='screen',
        shell=True,
        emulate_tty=True,
        *args,
        **kwargs
    )

def generate_launch_description():
    ld = LaunchDescription()

    # Define the mode
    mode = "simulation"

    disparity_extender_node = launch_node_with_configuration(
        package='Simple_algo', 
        executable='fast_disparity_extender', 
        name='fast_disparity_extender', 
        mode=mode,
    )

    # Register a shutdown event handler to handle the shutdown process gracefully
    ld.add_action(RegisterEventHandler(
        OnProcessExit(
            target_action=disparity_extender_node,
            on_exit=[Shutdown()]
        )
    ))

    # finalize
    ld.add_action(disparity_extender_node)
    return ld
