# basic_launch.launch.py
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    print(f'=============== Using mode {mode} ===============')
    print()

    # Launch the strategy nodes
    strategy_manager = launch_node_with_configuration(
        package='strategy',
        executable='basic_strategy_manager',
        name='basic_strategy_manager',
        mode=mode,
    )

    # Launch the "sensing" nodes
    if mode != "simulation":
        scan_filter_node = launch_node_with_configuration(
            package='Mapping',
            executable='scan_filter',
            name='scan_filter',
            mode=mode,
        )

    lap_tracker_node = launch_node_with_configuration(
        package='localisation',
        executable='lap_tracker',
        name='lap_tracker',
        mode=mode,
    )

    centerline_extraction_node = launch_node_with_configuration(
        package='Mapping',
        executable='centerline_extraction',
        name='centerline_extraction',
        mode=mode,
    )

    # Launch the SLAM
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('Mapping'), 'launch', 'slam.py')
        )
    )

    # Launch the "driving" nodes
    disparity_extender_node = launch_node_with_configuration(
        package='Simple_algo', 
        executable='disparity_extender', 
        name='disparity_extender', 
        mode=mode,
        remappings=[('/drive', '/disparity_extender')] # Plug the output of the disparity extender into the multiplexern, instead of the /drive topic directly
    )

    pure_pursuit_node = launch_node_with_configuration(
        package='Waypoints',
        executable='pure_pursuit',
        name='pure_pursuit',
        mode=mode,
        remappings=[('/drive', '/pure_pursuit')]
    )



    # finalize
    if mode != "simulation":
        ld.add_action(scan_filter_node)
    # ld.add_action(aeb_node)
    # ld.add_action(multiplexer_node)
    ld.add_action(strategy_manager)
    ld.add_action(lap_tracker_node)
    ld.add_action(centerline_extraction_node)
    ld.add_action(disparity_extender_node)
    ld.add_action(pure_pursuit_node)
    ld.add_action(slam_launch)
    return ld
