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
                {'cmd_tau': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_kp': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_kd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            ]
    )
    
    simple_sitter_node = Node(
        package="rhex_control",  
        executable="simple_sitter.py",  
        output="screen",
        parameters=[
                {'cmd_tau': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_kp': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_kd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            ]
    )
    
    simple_stander_node = Node(
        package="rhex_control",  
        executable="simple_stander.py",  
        output="screen",
        parameters=[
                {'cmd_tau': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_vel': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_pos': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_kp': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
                {'cmd_kd': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]},
            ]
    )
    
    simple_walker_node = Node(
        package="rhex_control",  
        executable="simple_walker.py",  
        output="screen",
    )

    return LaunchDescription([
        load_joint_state_controller,
        load_effort_controller,
        controller_node,
        simple_sitter_node,
        simple_stander_node,
        simple_walker_node
    ])
