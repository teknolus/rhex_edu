import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    start_script_supervisor = ExecuteProcess(
        cmd=['/home/rhex/mnt/scripts/start_rhex_supervisor.sh'],
        name='supervisor'
    )
    
    declare_world_name = DeclareLaunchArgument(
        name="world_name",
        default_value="empty.world",
        description='world to launch rhex in'
    )

    start_launch_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rhex_gazebo'),
            'launch',
            'start_sim.launch.py'
        )),
        launch_arguments={
            'world_name': LaunchConfiguration('world_name')
        }.items()
    )
    
    start_launch_controller_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rhex_control'),
            'launch',
            'start_controller_server.launch.py'
        ))
    )
    
    start_script_gui = ExecuteProcess(
        cmd=['/home/rhex/mnt/scripts/start_fltk_gui.sh'],
        name='fltk_gui'
    )
    
    return LaunchDescription([
        declare_world_name, 
        
        start_script_supervisor,
        start_launch_simulation,
        start_launch_controller_server,
        start_script_gui
    ])