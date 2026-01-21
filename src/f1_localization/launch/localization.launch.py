from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Récupérer le chemin vers le dossier d'installation du package
    pkg_share = get_package_share_directory('f1_localization')
    
    # Définir le chemin vers le fichier de configuration YAML
    config_file = os.path.join(pkg_share, 'config', 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config_file]
        )
    ])