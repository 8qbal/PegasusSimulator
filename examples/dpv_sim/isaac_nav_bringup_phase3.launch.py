import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time',
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    launch_vision_arg = DeclareLaunchArgument(
        'launch_vision',
        default_value='false',
        description='Enable vision corrector (requires ZED VIO stub)',
    )
    launch_vision = LaunchConfiguration('launch_vision')

    bridge_params = os.path.join(
        get_package_share_directory('px4_ros2_bridge'),
        'config', 'bridge_params.yaml',
    )

    default_mission = os.path.expanduser(
        '~/ros2_ws/src/warehouse_gz_sim_ws/mission_files/0001_0001_0001.json')
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=default_mission,
        description='Path to mission JSON file',
    )
    mission_file = LaunchConfiguration('mission_file')

    gt_odom_script = os.path.join(
        os.path.expanduser('~/PegasusSimulator'),
        'examples', 'dpv_sim', 'gt_odom_pub.py')

    zed_vio_stub_script = os.path.join(
        os.path.expanduser('~/PegasusSimulator'),
        'examples', 'dpv_sim', 'zed_vio_stub.py')

    return LaunchDescription([
        use_sim_time_arg,
        launch_vision_arg,
        mission_file_arg,

        SetParameter(name='use_sim_time', value=True),

        # --- Phase 1: Core bringup ---

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_laser_link',
            arguments=['0', '0', '0.135', '0', '0', '0', 'base_link_fcu', 'laser_link'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_link_to_imu_base',
            arguments=['0', '0', '0', '0', '0', '0', 'laser_link', 'fcu_imu_base_link_for_laser'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_fcu',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom_fcu'],
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cartographer_slam_wrapper'),
                    'launch', 'cartographer_slam.launch.py',
                ])
            ]),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        Node(
            package='px4_ros2_bridge',
            executable='from_fcu_vehicle_local_position_node',
            name='from_fcu_vehicle_local_position',
            output='screen',
            parameters=[bridge_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='px4_ros2_bridge',
            executable='from_fcu_status_relay_node',
            name='from_fcu_status_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='px4_ros2_bridge',
            executable='to_fcu_vehicle_visual_odometry_node',
            name='to_fcu_vehicle_visual_odometry',
            output='screen',
            parameters=[
                bridge_params,
                {'use_sim_time': use_sim_time},
                {'to_fcu.input_topics.odometry': '/cartographer/laser_odom_at_fcu'},
            ],
        ),
        Node(
            package='px4_ros2_bridge',
            executable='to_fcu_command_relay_node',
            name='to_fcu_command_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
        Node(
            package='px4_ros2_bridge',
            executable='to_fcu_trajectory_relay_node',
            name='to_fcu_trajectory_relay',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),

        # Publishes /px4_ros2_bridge/odometry/fcu_odom_flu, which
        # cartographer_pose_transformer requires to initialize laser_odom_at_fcu.
        # (Replaces the old topic_tools relay from fcu_pose_at_imu — that topic is
        # also published only by this node, so the relay never had input.)
        Node(
            package='px4_ros2_bridge',
            executable='from_fcu_vehicle_odometry_node',
            name='from_fcu_vehicle_odometry',
            output='screen',
            parameters=[bridge_params, {'use_sim_time': use_sim_time}],
        ),

        ExecuteProcess(
            cmd=['python3', gt_odom_script, '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ),

        # --- Phase 2: Staggered correction + navigation ---

        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'pub', '-r', '1',
                '/battery_remaining_time_s',
                'std_msgs/msg/Float32', '{data: 100.0}',
            ],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('warehouse_auto_mission'),
                    'launch', 'warehouse_auto_mission.launch.py',
                ])
            ]),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # mission_file is threaded through as a real launch substitution
        # (previously this concatenated the Python-time default_mission string
        # instead, so `mission_file:=...` on the CLI was silently ignored —
        # same bug already fixed in the guidance/phase-4 launches). -w 2 avoids
        # the DDS-discovery race documented in load_mission.sh.
        TimerAction(
            period=2.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'bash', '-c',
                        ('ros2 topic pub --once -w 2 /onboard_command '
                         'warehouse_ros2_msgs/msg/MissionCommand '
                         '"{header: {stamp: {sec: 0, nanosec: 0}, frame_id: \'\'}, '
                         'cmd_type: 0, cmd_int: 0, '
                         'cmd_string: \'', mission_file, '\', '
                         'cmd_param1_int: 0}"'),
                    ],
                    output='screen',
                )
            ],
        ),

        TimerAction(
            period=4.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('warehouse_path_planner'),
                            'launch', 'path_planner.launch.py',
                        ])
                    ]),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                ),
            ],
        ),

        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('trajectory_generator'),
                            'launch', 'trajectory_generator.launch.py',
                        ])
                    ]),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                ),
            ],
        ),

        TimerAction(
            period=8.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('warehouse_pose_corrector_laser_based'),
                            'launch', 'laser_scan_processing.launch.py',
                        ])
                    ]),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                ),
            ],
        ),

        TimerAction(
            period=10.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('warehouse_pose_correction_filter'),
                            'launch', 'perception_fusion.launch.py',
                        ])
                    ]),
                    launch_arguments={'use_sim_time': 'true'}.items(),
                ),
            ],
        ),

        # --- Phase 3: Vision corrector (optional) ---

        # ZED VIO stub: converts cartographer laser odom into a format the
        # vision corrector accepts as ZED VIO odometry.
        ExecuteProcess(
            cmd=['python3', zed_vio_stub_script, '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
            condition=IfCondition(launch_vision),
        ),

        # Laser-based pose corrector (same Phase 2). Vision corrector is an
        # additional corrector that consumes /zed/zed_node/odom_zed_to_fcu
        # and its own beam inference pipeline. Only launch if the user opts in.
        TimerAction(
            period=12.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('warehouse_pose_corrector_vision_based'),
                            'launch', 'warehouse_pose_corrector_vision_based.launch.py',
                        ])
                    ]),
                    launch_arguments={
                        'use_sim_time': 'true',
                        'camera_name': 'zed',
                    }.items(),
                    condition=IfCondition(launch_vision),
                ),
            ],
        ),
    ])
