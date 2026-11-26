from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    aloha_pkg = FindPackageShare('aloha')
    aloha_path = PathJoinSubstitution([aloha_pkg, 'launch'])


    return LaunchDescription([

        IncludeLaunchDescription(PythonLaunchDescriptionSource([aloha_path, '/aloha_bringup.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(['./remote.launch.py'])),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(['./state_manager.launch.py'])),
        
    ])
