from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.substitutions import LaunchConfiguration
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    
    port = LaunchConfiguration('port')
    baud_rate = LaunchConfiguration('baud_rate')
    frame_id = LaunchConfiguration('frame_id')
    
    declare_port= DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyUSB0',
        description='port for lidar')

    declare_baud_rate= DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='baud rate for lidar')

    declare_frame_id= DeclareLaunchArgument(
        'frame_id',
        default_value='laser_frame',
        description='frame id for lidar msg')
    DeclareLaunchArgument(
                name="rvizconfig",
                default_value=os.path.join(get_package_share_directory("delta_lidar"), 'rviz', 'delta_lidar.rviz'),
                description="Absolute path to rviz config file",
            ),
            
    return LaunchDescription([
        declare_port,
        declare_frame_id,
        declare_baud_rate,
        launch_ros.actions.Node(
            package='delta_lidar',
            executable='delta_lidar_node',
            name='delta_2b_lidar_node',
            output='screen',
            parameters=[{'port':port},{'frame_id':frame_id},{'baud_rate':baud_rate}],
           ),
        launch_ros.actions.Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", LaunchConfiguration("rvizconfig")],
            ), 
    ])


