import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    use_sim_time = LaunchConfiguration('use_sim_time')

    bridge_params = os.path.join(
        get_package_share_directory('px4_ros2_bridge'),
        'config', 'bridge_params.yaml',
    )

    return LaunchDescription([
        use_sim_time_arg,

        # --- Static transforms (sim geometry for V1) ---
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

        # --- Cartographer SLAM ---
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('cartographer_slam_wrapper'),
                    'launch', 'cartographer_slam.launch.py',
                ])
            ]),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # --- PX4 ROS 2 Bridge ---
        Node(
            package='px4_ros2_bridge',
            executable='from_fcu_vehicle_local_position_node',
            name='from_fcu_vehicle_local_position',
            output='screen',
            parameters=[bridge_params, {'use_sim_time': use_sim_time}],
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
    ])
