from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params=PathJoinSubstitution([FindPackageShare('rover_bringup'),'config','safety.yaml'])
    return LaunchDescription([Node(package='rover_safety',executable='cmd_vel_watchdog',output='screen',parameters=[params])])
