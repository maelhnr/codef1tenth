# basic_launch.launch.py
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node


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
        *args,
        **kwargs
    )

def generate_launch_description():
    ld = LaunchDescription()
    mode = "simulation"
    
    print()
    print(f'============= Using mode {mode} =============')
    print()

    if mode != "simulation":
        scan_filter_node = launch_node_with_configuration(
            package='Mapping',
            executable='scan_filter',
            name='scan_filter',
            mode=mode,
        )

    aeb_node = launch_node_with_configuration(
        package='AEB', 
        executable='aeb2', 
        name='aeb', 
        mode=mode,
    )

    multiplexer_node = launch_node_with_configuration(
        package='AEB', 
        executable='multiplexer', 
        name='multiplexer', 
        mode=mode,
    )

    disparity_extender_node = launch_node_with_configuration(
        package='Simple_algo', 
        executable='disparity_extender', 
        name='disparity_extender', 
        mode=mode,
        remappings=[('/drive', '/disparity_speed')] # Plug the output of the disparity extender into the multiplexern, instead of the /drive topic directly
    )

    # finalize
    if mode != "simulation":
        ld.add_action(scan_filter_node)
    ld.add_action(aeb_node)
    ld.add_action(multiplexer_node)
    ld.add_action(disparity_extender_node)
    return ld
