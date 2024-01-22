import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

from launch.actions import DeclareLaunchArgument

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_dec = DeclareLaunchArgument(
    'use_sim_time',
    default_value='false',
    description='Enables sim time for the follow node.')
    
    cmd_vel_topic = LaunchConfiguration('cmd_vel_topic')
    cmd_vel_topic_dec = DeclareLaunchArgument(
    'cmd_vel_topic',
    default_value='/base_controller/cmd_vel_unstamped',
    description='The name of the output command vel topic.')

    aruco_params = os.path.join(
        get_package_share_directory('ros2_aruco'),
        'config',
        'aruco_parameters.yaml'
        )
    
    # remappings = [('/cmd_vel', '/base_controller/cmd_vel_unstamped')]
    
    aruco_node = Node(
        package='ros2_aruco',
        executable='aruco_node',
        parameters=[aruco_params],
    )
    
    follow_node = Node(
            package='ros2_aruco',
            executable='follow_aruco',
            parameters=[aruco_params, {'use_sim_time': use_sim_time}],
            remappings=[('/base_controller/cmd_vel_unstamped', cmd_vel_topic)],
            # remappings=remappings
         )
    
    return LaunchDescription([
        use_sim_time_dec,
        cmd_vel_topic_dec,
        aruco_node,
        follow_node
    ])
