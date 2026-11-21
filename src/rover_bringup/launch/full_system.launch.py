from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    xacro_file = PathJoinSubstitution([FindPackageShare('rover_description'), 'urdf', 'rover.urdf.xacro'])
    terrain_params = PathJoinSubstitution([FindPackageShare('rover_bringup'), 'config', 'terrain.yaml'])
    safety_params = PathJoinSubstitution([FindPackageShare('rover_bringup'), 'config', 'safety.yaml'])
    return LaunchDescription([
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', output='screen',
             parameters=[{'robot_description': Command(['xacro ', xacro_file])}]),
        Node(package='rover_terrain_mapping', executable='terrain_mapper',
             name='terrain_mapper', output='screen', parameters=[terrain_params]),
        Node(package='rover_safety', executable='cmd_vel_watchdog',
             name='cmd_vel_watchdog', output='screen', parameters=[safety_params]),
    ])
