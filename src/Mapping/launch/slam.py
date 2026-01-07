from launch import LaunchDescription
from launch_ros.actions import Node

ODOM_TOPIC = '/odom'
QUIET = True

def generate_launch_description():
    ld = LaunchDescription()
    mode = "simulation"
    print()
    print(f'=============== Using mode {mode} for SLAM ===============')
    print()

    cartographer_node = Node(
        package='cartographer_ros', 
        executable='cartographer_node', 
        name='cartographer_node', 
        arguments=[
            '-configuration_directory', 'install/Mapping/share/Mapping/cartographer_config',
            '-configuration_basename', f'{mode}.lua',
        ],
        remappings=[
            ('odom', ODOM_TOPIC)
        ],
        output='screen' if not QUIET else 'own_log',
    )

    occupancy_grid_node = Node(
        package='cartographer_ros', 
        executable='occupancy_grid_node', 
        name='occupancy_grid_node', 
        arguments=[
            'resolution', '0.05',
            'publish_period_sec', '1.0',
        ],
        output='screen' if not QUIET else 'own_log',
    )

    pose_listener = Node(
        package='Mapping', 
        executable='pose_listener', 
        name='pose_listener',
        output='screen' if not QUIET else 'own_log',
    )

    # finalize
    ld.add_action(cartographer_node)
    ld.add_action(occupancy_grid_node)
    ld.add_action(pose_listener)
    return ld
