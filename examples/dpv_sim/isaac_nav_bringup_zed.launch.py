"""Phase 4 bringup: full GPS-denied mission stack with REAL ZED VIO as the EV source.

Differences vs isaac_nav_bringup_phase3.launch.py:
  - to_fcu_vehicle_visual_odometry runs on its REAL default input
    (/zed/zed_node/odom_zed_to_fcu from bridge_params.yaml) instead of the
    cartographer override — the same wiring as the drone, where "cartographer
    is retired from the stack" (see px4_ros2_bridge bridge.launch.py).
  - zed_odom_to_fcu.py (glue on the real zed_wrapper VIO) replaces
    zed_vio_stub.py. The wrapper itself runs in the start.sh tmux window "zed"
    (system ROS env; Isaac streams to it via the Stereolabs extension).
  - Cartographer still launches, but in SHADOW mode: nothing routes
    /cartographer/laser_odom_at_fcu into PX4 anymore. It keeps the laser
    corrector chain fed and gives an A/B reference against the ZED VIO
    (diagnose_ev_chain.py prints both).
  - Mission auto-load is opt-in (auto_load_mission:=true) and uses the launch
    argument via substitution + `-w 2` (DDS race fix) — default flow is:
    ev_ready.sh, then load_mission.sh load/start.
"""
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

THIS_DIR = os.path.abspath(os.path.dirname(__file__))
REPO_ROOT = os.path.dirname(os.path.dirname(THIS_DIR))


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
        description='Enable the vision corrector (consumes the real wrapper '
                    'depth on /zed/zed_node/depth/* — stretch goal)',
    )
    launch_vision = LaunchConfiguration('launch_vision')

    default_mission = os.path.join(
        REPO_ROOT, 'examples', 'mission', 'mission_file_sim_warehouse.json')
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=default_mission,
        description='Path to mission JSON file',
    )
    mission_file = LaunchConfiguration('mission_file')

    auto_load_mission_arg = DeclareLaunchArgument(
        'auto_load_mission',
        default_value='false',
        description='Scripted LOAD_MISSION at t=2s. Default off: gate on '
                    'ev_ready.sh, then drive with load_mission.sh.',
    )
    auto_load_mission = LaunchConfiguration('auto_load_mission')

    bridge_params = os.path.join(
        get_package_share_directory('px4_ros2_bridge'),
        'config', 'bridge_params.yaml',
    )

    gt_odom_script = os.path.join(THIS_DIR, 'gt_odom_pub.py')
    zed_glue_script = os.path.join(THIS_DIR, 'zed_odom_to_fcu.py')

    return LaunchDescription([
        use_sim_time_arg,
        launch_vision_arg,
        mission_file_arg,
        auto_load_mission_arg,

        SetParameter(name='use_sim_time', value=True),

        # --- Core TF + SLAM (cartographer in shadow mode) ---

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

        # --- px4_ros2_bridge ---

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
        # THE phase-4 difference: no input-topic override -> the node consumes
        # its bridge_params.yaml default /zed/zed_node/odom_zed_to_fcu (real
        # drone wiring). lever_arm: the sim ZED sits 0.355 m ahead of the FCU
        # origin (v1 nose); the drone config ships zeros (its 0.133 m offset is
        # commented out) — if EV innovations ever look yaw-dependent, zero this
        # too and accept the constant bias like the drone does.
        Node(
            package='px4_ros2_bridge',
            executable='to_fcu_vehicle_visual_odometry_node',
            name='to_fcu_vehicle_visual_odometry',
            output='screen',
            parameters=[
                bridge_params,
                {'use_sim_time': use_sim_time},
                {'to_fcu.lever_arm.x': 0.355},
                {'to_fcu.lever_arm.y': 0.0},
                {'to_fcu.lever_arm.z': 0.0},
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
        # cartographer_pose_transformer requires to initialize laser_odom_at_fcu
        # (still needed for the shadow cartographer + laser corrector chain).
        Node(
            package='px4_ros2_bridge',
            executable='from_fcu_vehicle_odometry_node',
            name='from_fcu_vehicle_odometry',
            output='screen',
            parameters=[bridge_params, {'use_sim_time': use_sim_time}],
        ),

        # --- ZED VIO glue: real zed_wrapper odom -> odom_zed_to_fcu ---
        ExecuteProcess(
            cmd=['python3', zed_glue_script, '--ros-args',
                 '-p', 'use_sim_time:=true', '-p', 'restamp:=false'],
            output='screen',
        ),

        ExecuteProcess(
            cmd=['python3', gt_odom_script, '--ros-args', '-p', 'use_sim_time:=true'],
            output='screen',
        ),

        # --- Mission / navigation chain (same staggering as phase 2/3) ---

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
                    condition=IfCondition(auto_load_mission),
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

        # --- Vision corrector (stretch goal, off by default) ---
        # With the real wrapper running there is no stub: the corrector's
        # depth inputs (/zed/zed_node/depth/depth_registered + camera_info)
        # and VIO input (/zed/zed_node/odom_zed_to_fcu) all exist for real.
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
