from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    # Command to kill any existing Gazebo processes
    # This ensures a clean start and avoids issues with lingering processes.
    clean_gazebo_processes = ExecuteProcess(
        cmd=['killall', '-9', 'gzserver', 'gzclient'],
        output='screen',
        # Set 'shell=True' if you encounter issues with the 'killall' command not being found.
        # This allows the command to be executed through a shell.
        # shell=True 
        # You might also want to add 'on_exit' to handle cases where no processes are running,
        # but for a simple "kill before start", this is usually sufficient.
    )

    gazebo = ExecuteProcess(
        cmd=['gazebo', '/home/senta/ros2_ws/src/world/world/world.sdf'],
        output='screen'
    )

    gazebo_vision_node = Node(
        package='simulasi',
        executable='gazebo_vision',
        name='gazebo_vision_node',
        output='screen'
    )

    return LaunchDescription([
        # Execute the clean-up command first
        clean_gazebo_processes,
        # Then launch Gazebo
        gazebo,
        # Finally, launch the gazebo_vision node after a delay
        TimerAction(
            period=10.0,
            actions=[gazebo_vision_node]
        )
    ])
