from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    optimizer_sys = Node(
        package="rhex_control", 
        executable= "efficient_optimizer_system.py",
        output="screen",
    )

    return LaunchDescription([
        optimizer_sys,
    ])
