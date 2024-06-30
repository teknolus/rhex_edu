from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
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
        executable="controller_server_network.py",
        output="screen",
        parameters=[
            {"host": LaunchConfiguration("host")},
            {"port": LaunchConfiguration("port")}
        ]
    )

    declare_host = DeclareLaunchArgument(
        name="host",
        default_value="127.0.0.1",
        description='Host address of the socket'
    )

    declare_port = DeclareLaunchArgument(
        "port",
        default_value="1256",
        description='Port of the socket'
    )

    return LaunchDescription([
        declare_host,
        declare_port,
        
        load_joint_state_controller,
        load_effort_controller,
        controller_node,
    ])
