from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    params=PathJoinSubstitution([FindPackageShare('rover_bringup'),'config','terrain.yaml'])
    return LaunchDescription([Node(package='rover_terrain_mapping',executable='terrain_mapper',name='terrain_mapper',output='screen',parameters=[params])])
