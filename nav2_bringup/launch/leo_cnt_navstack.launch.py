import os
import tempfile
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            RegisterEventHandler)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    ld = LaunchDescription()
    # Get the launch directory
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')
    # TODO: Avvio pacchetto pubblicazione tf2 da PX4, convertire in nav2_slam
    drone_bringup_dir = get_package_share_directory('gz_drone_bringup')
    # TODO: Avvio simulazione da launchfile
    # sim_dir = get_package_share_directory('nav2_minimal_tb3_sim')
    param_file=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')

    params_file = LaunchConfiguration('params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value = param_file,
        description='Full path to the ROS2 parameters file to use for all launched nodes',
    )

    # declare_use_simulator_cmd = DeclareLaunchArgument(
    #     'use_simulator',
    #     default_value='True',
    #     description='Whether to start the simulator',
    # )

    # TODO: Dichiarazione mondo da lanciare
    # declare_world_cmd = DeclareLaunchArgument(
    #     'world',
    #     default_value=os.path.join(sim_dir, 'worlds', 'tb3_sandbox.sdf.xacro'),
    #     description='Full path to world model file to load',
    # )

    # TODO: Dichiarazione sdf robot
    # declare_robot_sdf_cmd = DeclareLaunchArgument(
    #     'robot_sdf',
    #     default_value=os.path.join(sim_dir, 'urdf', 'gz_waffle.sdf.xacro'),
    #     description='Full path to robot sdf file to spawn the robot in gazebo',
    # )

    # TODO: Personalizzazione posa iniziale
    # pose = {
    #     'x': LaunchConfiguration('x_pose', default='-2.00'),
    #     'y': LaunchConfiguration('y_pose', default='-0.50'),
    #     'z': LaunchConfiguration('z_pose', default='0.01'),
    #     'R': LaunchConfiguration('roll', default='0.00'),
    #     'P': LaunchConfiguration('pitch', default='0.00'),
    #     'Y': LaunchConfiguration('yaw', default='0.00'),
    # }

    param_file=os.path.join(bringup_dir, 'params', 'nav2_params.yaml')
    with open(param_file, 'r') as file:
        params = yaml.safe_load(file)
        namespace = params.get('nav2_launcher', {}).get('ros__parameters', {}).get('namespace', 'uav')
        slam = params.get('nav2_launcher', {}).get('ros__parameters', {}).get('slam', True)
        use_sim_time = params.get('nav2_launcher', {}).get('ros__parameters', {}).get('use_sim_time', False)
        robot_name = params.get('nav2_launcher', {}).get('ros__parameters', {}).get('robot_name', 'x500')
        use_robot_state_pub = params.get('nav2_launcher', {}).get('ros__parameters', {}).get('use_robot_state_pub', True)
        use_rviz = params.get('nav2_launcher', {}).get('ros__parameters', {}).get('use_rviz', True)
        rviz_config_file = params.get('nav2_launcher', {}).get('ros__parameters', {}).get('rviz_config_file', 'leo_cnt.rviz')

    urdf_file = os.path.join(get_package_share_directory(robot_name + '_description'), 'urdf', robot_name + '.urdf')
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()

    # TODO: Specifica robot state publisher custom
    if use_robot_state_pub:
        start_robot_tf_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(drone_bringup_dir, 'launch', 'tf_tree_init_depth.launch.py')),
            launch_arguments={
            # TODO: Modificare namespace gz_drone_bringup
            'namespace': "", 
        }.items(),
        )
        ld.add_action(start_robot_tf_publisher_cmd)

    if use_robot_state_pub:
        start_robot_state_publisher_cmd = Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                output='screen',
                parameters=[
                    {'robot_description': robot_description}
                ],
                remappings=[('/robot_description', '/' + namespace + '/robot_description')],
        )
        ld.add_action(start_robot_state_publisher_cmd)

    if use_rviz:
        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'rviz_launch.py')),
            launch_arguments={
                'use_sim_time': str(use_sim_time),
                'rviz_config': os.path.join(bringup_dir, 'rviz', rviz_config_file),
            }.items(),
        )
        ld.add_action(rviz_cmd)

    # RUNNING: bringup_launch.py
    # TODO: Aggiungi tutti gli argomenti
    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'bringup_launch.py')),
        launch_arguments={
            'namespace': namespace,
            # 'slam': slam,
            # 'map': map_yaml_file,
            'use_sim_time': str(use_sim_time),
            'params_file': params_file,
            'use_keepout_zones': 'False',
            'use_speed_zones': 'False',
        }.items(),
    )

    # TODO: Modifica per caricare mondo custom
    # The SDF file for the world is a xacro file because we wanted to
    # conditionally load the SceneBroadcaster plugin based on whether we're
    # running in headless mode. But currently, the Gazebo command line doesn't
    # take SDF strings for worlds, so the output of xacro needs to be saved into
    # a temporary file and passed to Gazebo.
    # world_sdf = tempfile.mktemp(prefix='nav2_', suffix='.sdf')
    # world_sdf_xacro = ExecuteProcess(
    #     cmd=['xacro', '-o', world_sdf, ['headless:=', headless], world])
    # if use_simulator:
    # gazebo_server = ExecuteProcess(
    #     cmd=['gz', 'sim', '-r', '-s', world_sdf],
    #     output='screen',
    # )
    # remove_temp_sdf_file = RegisterEventHandler(event_handler=OnShutdown(
    #     on_shutdown=[
    #         OpaqueFunction(function=lambda _: os.remove(world_sdf))
    #     ]))
    # if use_gazebo and not headless:
    # gazebo_client = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ros_gz_sim'),
    #                      'launch',
    #                      'gz_sim.launch.py')
    #     ),
    #     launch_arguments={'gz_args': ['-v4 -g ']}.items(),
    # )
    # gz_robot = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(sim_dir, 'launch', 'spawn_tb3.launch.py')),
    #     launch_arguments={'namespace': namespace,
    #                       'use_sim_time': str(use_sim_time),
    #                       'robot_name': robot_name,
    #                       'robot_sdf': robot_sdf,
    #                       'x_pose': pose['x'],
    #                       'y_pose': pose['y'],
    #                       'z_pose': pose['z'],
    #                       'roll': pose['R'],
    #                       'pitch': pose['P'],
    #                       'yaw': pose['Y']}.items())

    # TODO: Eliminare lancio dell'octomap per inserirlo nel giusto launchfile
    start_octomap_server_cmd = Node(
            package='octomap_server',
            executable='octomap_server_node',
            name='octomap_server',
            output='screen',
            remappings=[
                ('cloud_in', namespace + '/camera/depth/points')
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
        )

    # Declare the launch options
    ld.add_action(declare_params_file_cmd)
    # ld.add_action(declare_robot_sdf_cmd)
    # ld.add_action(declare_use_respawn_cmd)
    # ld.add_action(world_sdf_xacro)
    # ld.add_action(remove_temp_sdf_file)
    # ld.add_action(gz_robot)
    # ld.add_action(gazebo_server)
    # ld.add_action(gazebo_client)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(bringup_cmd)
    ld.add_action(start_octomap_server_cmd)

    return ld
