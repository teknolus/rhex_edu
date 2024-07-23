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

    
    simple_walker_node = Node(
        package="rhex_control",  
        executable="simple_walker.py",  
        output="screen",
    )

    test_robot_node = Node(
        package="rhex_control",  
        executable="test_robot.py",  
        output="screen",
    )

    return LaunchDescription([
        load_joint_state_controller,
        load_effort_controller,
        test_robot_node,
        simple_walker_node
    ])
