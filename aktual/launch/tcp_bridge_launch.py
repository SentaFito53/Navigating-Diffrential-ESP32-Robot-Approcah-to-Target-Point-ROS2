import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    urdf_path = os.path.join(
        get_package_share_directory('aktual'),
        'urdf',
        'model_robot.urdf'
    )

    with open(urdf_path, 'r') as urdf_file:
        robot_desc = urdf_file.read()

    return LaunchDescription([
        #Node(
        #    package='communication',
        #    executable='communication',
        #    name='ros2_tcp_bridge',
        #    parameters=['/home/senta/ros2_ws/src/aktual/config/robot.yaml'],
        #),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_base',
            arguments=['0', '0', '0.02385', '0', '0', '0', 'base_footprint', 'base_link'],
            output='screen'
        ),

        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            output='screen'
        ),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[
                {'use_sim_time': True},
                {'robot_description': robot_desc}
            ],
            output='screen'
        ),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                '/world/empty/model/urdf_model/joint_state@sensor_msgs/msg/JointState@gz.msgs.Model'
            ],
            remappings=[
                ('/world/empty/model/urdf_model/joint_state', '/joint_states')
            ],
            output='screen'
        ),

        Node(
            package='urdf_model',
            executable='differential_drive_odometry',
            name='differential_drive_odometry',
            output='screen'
        ),

        #Node(
        #    package='aktual',
        #    executable='vision',
        #    name='detection',
        #    output='screen'
        #),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(
                get_package_share_directory('urdf_model'),
                'rviz',
                'rviz_config.rviz'
            )],
            output='screen'
        )
    ])

