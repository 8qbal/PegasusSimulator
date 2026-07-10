# Copyright (c) 2023, Marcelo Jacinto
# All rights reserved.
#
# SPDX-License-Identifier: BSD-3-Clause
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig

# Sensors and dynamics setup
from pegasus.simulator.logic.dynamics import LinearDrag
from pegasus.simulator.logic.thrusters import QuadraticThrustCurve
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.graphical_sensors.monocular_camera import MonocularCamera
from pegasus.simulator.logic.graphical_sensors.lidar import Lidar

# Mavlink interface
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig

# ROS 2 interface (publishes ZED + RPLIDAR data and TF for the SLAM pipeline)
from pegasus.simulator.logic.backends.ros2_backend import ROS2Backend

# Get the location of the V1 asset
from pegasus.simulator.params import ROBOTS

class V1Config(MultirotorConfig):

    def __init__(self):

        # Initialize the base config first (sets graphical_sensors, graphs, etc.)
        super().__init__()

        # Stage prefix of the vehicle when spawning in the world
        self.stage_prefix = "quadrotor"

        # The USD file that describes the visual aspect of the vehicle (and some properties such as mass and moments of inertia).
        # The V1 asset is full scale (0.85 m span, 2.0 kg): rotors sit at the real hub positions
        # extracted from the V1 mesh, and each prop is its own rigid body so it spins visually.
        self.usd_file = ROBOTS["V1"]

        # Thrust curve matching the real V1 drone: 2.8 kgf max thrust per motor at 1100 rad/s
        # => rotor_constant = 2.8 * 9.80665 / 1100^2 ~= 2.27e-5  (T = k * omega^2)
        self.thrust_curve = QuadraticThrustCurve(config={
            "num_rotors": 4,
            "rotor_constant": [2.27e-5, 2.27e-5, 2.27e-5, 2.27e-5],
            "rolling_moment_coefficient": [2.66e-6, 2.66e-6, 2.66e-6, 2.66e-6],
            "rot_dir": [-1, -1, 1, 1],
            "min_rotor_velocity": [0, 0, 0, 0],
            "max_rotor_velocity": [1100, 1100, 1100, 1100],
        })
        self.drag = LinearDrag([0.50, 0.30, 0.0])

        # The default sensors for a quadrotor
        self.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]

        # Graphical sensors carried by the real V1 drone:
        # - ZED 2i stereo camera looking forward at the nose (modeled as a monocular RGB-D camera:
        #   per-eye 1920x1200 @ 30 fps, ~102 deg horizontal FOV -> fx = fy ~= 777 px)
        # - RPLIDAR C1 2D lidar on the upper body (RPLIDAR_S2E is the closest RTX lidar
        #   profile shipped with Isaac Sim - same SLAMTEC 2D 360 deg rotary family)
        self.graphical_sensors = [
            MonocularCamera("zed2i", config={
                "position": [0.355, 0.0, 0.0],
                "resolution": (1920, 1200),
                "frequency": 30,
                "intrinsics": [[777.0, 0.0, 960.0], [0.0, 777.0, 600.0], [0.0, 0.0, 1.0]],
                "depth": True,
            }),
            Lidar("rplidar_c1", config={
                "position": [0.0, 0.0, 0.135],
                # RPLIDAR_S2E (the closest bundled named profile to the real RPLIDAR C1) produces
                # zero output in this Isaac Sim build - confirmed via a targeted self-contained
                # repro: PointCloud2 and LaserScan both stay at 0 messages indefinitely with
                # RPLIDAR_S2E, while Example_Rotary_2D (same 2D rotary sensor class, generic
                # profile) reliably publishes both. Using the generic profile as the stand-in.
                "sensor_configuration": "Example_Rotary_2D",
                "frequency": 10.0,
                "show_render": False,
            }),
        ]

        # Backends: PX4 over MAVLink for flight control, plus a ROS 2 backend that publishes
        # the ZED image/depth/camera_info and RPLIDAR point cloud/laser scan (feeding SLAM),
        # ground-truth pose/twist/accel (for debugging/visualization), and a ground-truth TF
        # tree (map -> {namespace}_base_link). Note: this ground-truth TF is NOT what feeds
        # SLAM's odom prior - it exists only for debugging/rviz. SLAM's own map->odom->base_link
        # chain is a separate tree published by the SLAM node itself (see px4_slam_bridge).
        self.backends = [
            PX4MavlinkBackend(config=PX4MavlinkBackendConfig()),
            ROS2Backend(vehicle_id=0, config={
                "namespace": "v1_",
                "pub_tf": True,
                "pub_gps": False,
                "pub_gps_vel": False,
            }),
        ]

class V1(Multirotor):

    def __init__(self, id: int, world, init_pos=[0.0, 0.0, 0.15], init_orientation=[0.0, 0.0, 0.0, 1.0], config=V1Config()):
        super().__init__(config.stage_prefix, config.usd_file, id, world, init_pos, init_orientation, config=config)
