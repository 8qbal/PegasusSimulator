#!/usr/bin/env python
"""
| File: 12_px4_v1_vehicle.py
| License: BSD-3-Clause. Copyright (c) 2023, Marcelo Jacinto. All rights reserved.
| Description: Example on how to run a simulation with the custom V1 vehicle (models/V1.glb converted
| to USD, mounted on the Iris physics skeleton), controlled using the MAVLink control backend.
"""

# Imports to start Isaac Sim from this script
import carb
from isaacsim import SimulationApp

# Start Isaac Sim's simulation environment
# Note: this simulation app must be instantiated right after the SimulationApp import, otherwise the simulator will crash
# as this is the object that will load all the extensions and load the actual simulator.
simulation_app = SimulationApp({"headless": False})

# -----------------------------------
# The actual script should start here
# -----------------------------------
import omni.timeline
from isaacsim.core.api import World

# Import the Pegasus API for simulating drones
from pegasus.simulator.params import ROBOTS, SIMULATION_ENVIRONMENTS
from pegasus.simulator.logic.state import State
from pegasus.simulator.logic.backends.px4_mavlink_backend import PX4MavlinkBackend, PX4MavlinkBackendConfig
from pegasus.simulator.logic.vehicles.multirotor import Multirotor, MultirotorConfig
from pegasus.simulator.logic.vehicles.multirotors.v1 import V1Config
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# Auxiliary scipy and numpy modules
import os.path
from scipy.spatial.transform import Rotation

class PegasusApp:
    """
    A Template class that serves as an example on how to build a simple Isaac Sim standalone App.
    """

    def __init__(self):
        """
        Method that initializes the PegasusApp and is used to setup the simulation environment.
        """

        # Acquire the timeline that will be used to start/stop the simulation
        self.timeline = omni.timeline.get_timeline_interface()

        # Start the Pegasus Interface
        self.pg = PegasusInterface()

        # Acquire the World, .i.e, the singleton that controls that is a one stop shop for setting up physics,
        # spawning asset primitives, etc.
        self.pg._world = World(**self.pg._world_settings)
        self.world = self.pg.world

        # Launch one of the worlds provided by NVIDIA
        self.pg.load_environment(SIMULATION_ENVIRONMENTS["Warehouse"])

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        config_multirotor = V1Config()  # carries the real V1 thrust curve (2.8 kgf/motor, 2.0 kg)

        # GPS-denied setup for SLAM testing: no GPS sensor -> no HIL_GPS is sent to PX4.
        # Horizontal position must come from SLAM (e.g. vision/odometry into PX4); until then
        # fly in Stabilized/Altitude mode (baro height only).
        config_multirotor.sensors = [Barometer(), IMU(), Magnetometer()]
        # Create the multirotor configuration
        mavlink_config = PX4MavlinkBackendConfig({
            "vehicle_id": 0,
            "px4_autolaunch": True,
            "px4_dir": self.pg.px4_path,
            "px4_vehicle_model": self.pg.px4_default_airframe # CHANGE this line to 'iris' if using PX4 version bellow v1.14
        })
        # Replace ONLY the default PX4 backend that V1Config built (index 0) with this
        # properly-configured one (px4_dir / airframe / autolaunch). Crucially, keep the
        # ROS2Backend that V1Config created at index 1 - it is what publishes the ZED /
        # RPLIDAR data and vehicle state for the SLAM pipeline. Reassigning the whole list
        # (backends = [PX4]) drops the ROS2Backend, so the vehicle never publishes any
        # sensor/lidar/camera topics and SLAM gets no data.
        config_multirotor.backends[0] = PX4MavlinkBackend(mavlink_config)

        Multirotor(
            "/World/quadrotor",
            ROBOTS['V1'],
            0,
            [0.0, 0.0, 0.15],
            Rotation.from_euler("XYZ", [0.0, 0.0, 0.0], degrees=True).as_quat(),
            config=config_multirotor,
        )

        # Reset the simulation environment so that all articulations (aka robots) are initialized
        self.world.reset()

        # Auxiliar variable for the timeline callback example
        self.stop_sim = False

    def run(self):
        """
        Method that implements the application main loop, where the physics steps are executed.
        """

        # Start the simulation
        self.timeline.play()

        # The "infinite" loop
        while simulation_app.is_running() and not self.stop_sim:

            # Update the UI of the app and perform the physics step
            self.world.step(render=True)

        # Cleanup and stop
        carb.log_warn("PegasusApp Simulation App is closing.")
        self.timeline.stop()
        simulation_app.close()

def main():

    # Instantiate the template app
    pg_app = PegasusApp()

    # Run the application loop
    pg_app.run()

if __name__ == "__main__":
    main()
