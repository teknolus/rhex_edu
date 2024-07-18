import os
import xacro

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Define the parameters that can be set in the launch file and their default values
configurable_parameters = [
    {'name': 'robot_name',   'default': 'rhex',        'description': 'Name of the robot to be spawned in gazebo'},
    {'name': 'use_sim_time', 'default': 'true',        'description': 'Use simulation time or not'},
    {'name': 'world_name',   'default': 'empty.world', 'description': 'Name of the world to be loaded in gazebo'},
    {'name': 'pos_x',        'default': '0.0',         'description': 'Initial x position of the robot'},
    {'name': 'pos_y',        'default': '0.0',         'description': 'Initial y position of the robot'},
    {'name': 'pos_z',        'default': '0.1',         'description': 'Initial z position of the robot'}
]

# Function to declare the parameters in the launch file
def declare_configurable_parameters(parameters):
    return [DeclareLaunchArgument(param['name'], default_value=param['default'], description=param['description']) for param in parameters]

# Function to generate the launch description
def generate_launch_description():

    # Get the path to the robot description file
    robot_description_path = os.path.join(
        get_package_share_directory('rhex_description'),
        'urdf/robot.urdf.xacro'
    )

    # Start the robot state publisher node, which publishes the robot state to the tf tree
    start_node_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": xacro.process_file(robot_description_path).toxml(),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }]
    )

    # Include the gazebo launch file, which starts the gazebo simulator
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch/gazebo.launch.py")
        ),
        launch_arguments={
            "world": PathJoinSubstitution([
                get_package_share_directory('rhex_gazebo'),
                'worlds',
                LaunchConfiguration('world_name')
            ]),
            "verbose": "true",
        }.items()
    )

    # Start the spawn entity node, which spawns the robot in gazebo
    start_node_spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity", "rhex",
            "-topic", "/robot_description",
            "-robot_namespace", "",
            "-x", LaunchConfiguration('pos_x'),
            "-y", LaunchConfiguration('pos_y'),
            "-z", LaunchConfiguration('pos_z')
        ]
    )

 


    # Return the launch description, which includes the parameters and the nodes to be executed
    return LaunchDescription([
        *declare_configurable_parameters(configurable_parameters),
        start_node_robot_state_publisher,
        gazebo,
        start_node_spawn_entity
    ])

