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
    
    # === LAUNCH ARGUMENTS ===
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
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
    
    # === MOVEIT CONFIGURATION ===
    moveit_available = False
    moveit_config = None
    try:
        moveit_config = (
            # TODO: Sostituisci stringa hardcoded con robot_name ricavabile come stringa
            MoveItConfigsBuilder('x500')
            .robot_description(file_path="config/x500.urdf.xacro")
            .robot_description_semantic(file_path="config/x500.srdf")
            .joint_limits(file_path="config/joint_limits.yaml")
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

    if moveit_available:
        move_group_node = Node(
            package="moveit_ros_move_group",
            executable="move_group",
            name="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
            ],
            arguments=["--ros-args", "--log-level", "info"],
        )
        ld.add_action(move_group_node)

    # === ADD ARGUMENTS ===
    ld.add_action(namespace_arg)
    ld.add_action(robot_name_arg)
    ld.add_action(use_sim_time_arg)

    return ld