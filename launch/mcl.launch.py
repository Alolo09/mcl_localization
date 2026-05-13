import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path  = get_package_share_directory('mcl_localization')
    urdf_file = os.path.join(pkg_path, 'urdf', 'mcl_robot.urdf')
    world_file = os.path.join(pkg_path, 'worlds', 'mi_mundo.world')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # Gazebo
        ExecuteProcess(
            cmd=['ros2', 'launch', 'gazebo_ros', 'gazebo.launch.py',
                 f'world:={world_file}'],
            output='screen'
        ),
        # Publicar URDF en /robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'
        ),
        # Spawnear robot desde el topic
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-topic', '/robot_description',
                       '-entity', 'mcl_robot',
                       '-x', '0.0', '-y', '0.0', '-z', '0.1'],
            output='screen'
        ),
    ])