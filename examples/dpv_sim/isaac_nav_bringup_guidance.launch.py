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

    bridge_params = os.path.join(
        get_package_share_directory('px4_ros2_bridge'),
        'config', 'bridge_params.yaml',
    )

    default_mission = os.path.expanduser(
        '~/ros2_ws/src/warehouse_gz_sim_ws/mission_files/0001_0001_0001.json')
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=default_mission,
        description='Path to mission JSON file (used only when auto_load_mission:=true)',
    )
    mission_file = LaunchConfiguration('mission_file')

    # With RViz up, the WarehouseCommanderPanel is the natural way to pick a mission
    # file and drive it (Load -> Start), so the scripted load is off by default: it would
    # race the panel and leave the state machine READY on a file you did not choose.
    # Set true for a headless run that should load one specific mission by itself.
    auto_load_mission_arg = DeclareLaunchArgument(
        'auto_load_mission',
        default_value='false',
        description='Publish LOAD_MISSION for mission_file automatically at +2s '
                    '(headless); leave false to load from the RViz commander panel',
    )
    auto_load_mission = LaunchConfiguration('auto_load_mission')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz with the warehouse status/commander panels',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(os.path.dirname(os.path.realpath(__file__)),
                                   'guidance.rviz'),
        description='RViz config to use',
    )

    return LaunchDescription([
        use_sim_time_arg,
        mission_file_arg,
        auto_load_mission_arg,
        rviz_arg,
        rviz_config_arg,

        SetParameter(name='use_sim_time', value=True),

        # RViz carries the two rviz2_plugin panels from the real stack: WarehouseStatusPanel
        # (state machine / waypoint progress) and WarehouseCommanderPanel, whose
        # "Select Mission File" + Load/Start buttons publish warehouse_ros2_msgs/MissionCommand
        # on /onboard_command - the same message dpv_sim/load_mission.sh sends from a terminal.
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rviz_config')],
            parameters=[{'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),

        # --- GPS-fused bridge nodes (no lidar/Cartographer/EV reroute) ---

        Node(
            package='px4_ros2_bridge',
            executable='from_fcu_vehicle_local_position_node',
            name='from_fcu_vehicle_local_position',
            output='screen',
            parameters=[bridge_params, {'use_sim_time': use_sim_time}],
        ),
        Node(
            package='px4_ros2_bridge',
            executable='from_fcu_vehicle_odometry_node',
            name='from_fcu_vehicle_odometry',
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

        # --- Guidance stack: mission / planner / trajectory ---

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

        # Optional scripted LOAD_MISSION at +2 s, for headless runs (auto_load_mission:=true).
        # cmd_type 0 = GCS_COMMAND, cmd_int 0 = GCS_CMD_LOAD_MISSION - same message the RViz
        # commander panel's "Load" button sends. This only reaches READY; something still has
        # to send START_MISSION (cmd_int 2) to actually fly it.
        #
        # mission_file is passed as a substitution rather than baked in: the previous version
        # interpolated default_mission into the string at launch-file build time, so
        # `mission_file:=...` was silently ignored.
        TimerAction(
            period=2.0,
            condition=IfCondition(auto_load_mission),
            actions=[
                ExecuteProcess(
                    cmd=[
                        # -w 2: without it the publisher exits before DDS matches the
                        # subscribers and the command is silently dropped (see
                        # dpv_sim/load_mission.sh for the full explanation).
                        'ros2', 'topic', 'pub', '--once', '-w', '2', '/onboard_command',
                        'warehouse_ros2_msgs/msg/MissionCommand',
                        ['{cmd_type: 0, cmd_int: 0, cmd_string: "', mission_file,
                         '", cmd_param1_int: 0}'],
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
    ])
