from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'joint_states_controller'],
        output='screen',
    )

    load_effort_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
             'effort_controller'],
        output='screen'
    )

    controller_node = Node(
        package="rhex_control",  
        executable="simple_controller_server_network.py",  
        output="screen",
        parameters=[
                {'cmd_tau': [10.0, 20.0, 30.0, 40.0, 50.0, 600.0]},
                {'cmd_vel': [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]},
                {'cmd_pos': [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]},
                {'cmd_kp': [0.001, 0.002, 0.003, 0.004, 0.005, 0.006]},
                {'cmd_kd': [0.0001, 0.0002, 0.0003, 0.0004, 0.0005, 0.0006]},
            ]
    )

    return LaunchDescription([
        load_joint_state_controller,
        load_effort_controller,
        controller_node,
    ])
