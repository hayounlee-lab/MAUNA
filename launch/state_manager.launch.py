from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['python3', '/home/aloha/mauna_operation/internal/state_manager.py'],
            output='screen'
        )
    ])
