import copy
from launch import LaunchDescription
import launch_ros.actions
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, ThisLaunchFileDir, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import sys
import pathlib
import os
import yaml
sys.path.append(str(pathlib.Path(__file__).parent.absolute()))

def generate_launch_description():

    return LaunchDescription([
        IncludeLaunchDescription(
            PathJoinSubstitution([
                FindPackageShare('gz_drone_bringup'),
                'launch',
                'tf_tree_init_depth.launch.py'
            ]),
        ),
        launch_ros.actions.Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('leo_cnt_navstack'), 'config', 'leo_cnt.rviz')],
            output='screen',
            # usa `additional_env` per LD_PRELOAD
            additional_env={
                'LD_PRELOAD': '/usr/lib/x86_64-linux-gnu/liboctomap.so'
            }
        ),
        
         launch_ros.actions.Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', 'uav/camera/depth/points')
            ],
            parameters=[{
                'resolution': 0.03,
                'frame_id': 'map',
                'sensor_model/max_range': 10.0,
                'ground_filter/distance': 0.6,
                'point_cloud_min_x': -1.0,
                'point_cloud_max_x': 20.0,
                'point_cloud_min_y': -1.0,
                'point_cloud_max_y': 10.0,
                'point_cloud_min_z': -0.5,
                'point_cloud_max_z': 3.5
            }]
        ),
    ])
