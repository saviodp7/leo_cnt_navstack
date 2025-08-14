import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()

    # === LAUNCH CONFIGURATIONS ===
    namespace = LaunchConfiguration('namespace')
    robot_name = LaunchConfiguration('robot_name')
    use_sim_time = LaunchConfiguration('use_sim_time')
    moveit_config_package = LaunchConfiguration('moveit_config_package')
    
    # === LAUNCH ARGUMENTS ===
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='uav',
        description='Robot namespace'
    )
    
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='x500',
        description='Robot name for description package'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )
    
    moveit_config_package_arg = DeclareLaunchArgument(
        'moveit_config_package',
        default_value='x500_moveit_config',
        description='MoveIt config package name'
    )

    # === MOVEIT CONFIGURATION ===
    moveit_available = False
    moveit_config = None
    try:
        moveit_config = (
            # TODO: Sostituisci stringa hardcoded con robot_name ricavabile come stringa
            MoveItConfigsBuilder('x500')
            .robot_description(file_path="config/x500.urdf.xacro")
            .robot_description_semantic(file_path="config/x500.srdf")
            .robot_description_kinematics(file_path="config/kinematics.yaml")
            .robot_description_planning(file_path="config/joint_limits.yaml")
            .planning_scene_monitor(
                publish_robot_description=False,
                publish_robot_description_semantic=True,
                publish_planning_scene=True,
            )
            .trajectory_execution(file_path="config/moveit_controllers.yaml")
            .planning_pipelines(
                pipelines=["ompl"],
            )
            .sensors_3d(file_path="config/sensors_3d.yaml")
            .to_moveit_configs()
        )
        moveit_available = True
        print("[INFO] MoveIt config loaded successfully")         
    except Exception as e:
        print(f"[ERROR] MoveIt config not available: {e}")
        moveit_available = False

    # === MOVEIT COMPONENTS ===
    if moveit_available:
        # Move Group Node (Core MoveIt Planning)
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            namespace=[namespace],  # Fix: usa lista
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                # {'use_sim_time': use_sim_time}
            ],
            arguments=["--ros-args", "--log-level", "info"],
        )
        ld.add_action(move_group_node)

    start_rviz_cmd = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', os.path.join(get_package_share_directory('nav2_bringup'), 'rviz', 'leo_cnt_nav2_moveit.rviz'), '--ros-args', '--log-level', 'warn'],
    output='screen',
    parameters=[
        moveit_config.robot_description,
        moveit_config.robot_description_semantic,
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ],
    remappings=[
        ('/tf', 'tf'),
        ('/tf_static', 'tf_static'),
    ]
    )
    ld.add_action(start_rviz_cmd)



    # === POINT CLOUD PREPROCESSING (Opzionale ma utile) ===
    # Filtro per migliorare la qualità del point cloud
    # pointcloud_filter_node = Node(
    #     package='pcl_ros',
    #     executable='voxel_grid_filter_node',
    #     name='pointcloud_filter',
    #     namespace=[namespace],  # Fix: usa lista
    #     output='screen',
    #     remappings=[
    #         ('input', '/uav/camera/depth/points'),
    #         ('output', '/uav/camera/depth/points_filtered'),
    #     ],
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         'leaf_size': 0.03,  # Voxel size per downsampling
    #         'filter_field_name': 'z',
    #         'filter_limit_min': -2.0,
    #         'filter_limit_max': 5.0,
    #         'filter_limit_negative': False,
    #     }]
    # )
    # Comenta se non hai pcl_ros installato
    # ld.add_action(pointcloud_filter_node)

    # === ADD ARGUMENTS ===
    ld.add_action(namespace_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(moveit_config_package_arg)

    return ld