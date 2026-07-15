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
from pegasus.simulator.logic.sensors import Barometer, IMU, Magnetometer, GPS
from pegasus.simulator.logic.interface.pegasus_interface import PegasusInterface
# Auxiliary scipy and numpy modules
import os
import os.path
import sys
from scipy.spatial.transform import Rotation

# The procedural DPV aisle world lives next to the rest of the dpv_sim tooling
sys.path.insert(0, os.path.abspath(os.path.dirname(__file__)) + '/dpv_sim')
from warehouse_aisle import build_aisle_stock

# Guidance mode (start.sh phase g) needs GPS reaching PX4 for EKF2 to fuse it -
# GPS-denied phases 1-3 must NOT have it, so vision/laser is the only position source.
GUIDANCE_MODE = os.environ.get("DPV_GUIDANCE_MODE") == "1"

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

        # The building shell: the empty "Warehouse" supplies a real concrete floor, walls
        # and lighting (24.0 x 38.8 x 9.3 m interior), but none of its own shelving.
        #
        # Loaded via load_asset() rather than load_environment() on purpose. The latter is
        # asyncio.ensure_future(...) - it only *schedules* the load, so the floor can still
        # be missing when the vehicle spawns and world.reset() runs below, dropping the V1
        # through empty space until the floor pops in underneath it. load_environment with
        # force_clear=False does nothing but call load_asset anyway, so this is the same
        # load, just guaranteed to finish first.
        self.pg.load_asset(SIMULATION_ENVIRONMENTS["Warehouse"], "/World/layout")

        # Yaw the shell 90 deg so its long axis (38.82 m) lies on X to receive the aisle,
        # which must run along +X to match the mission frame (see build_aisle_stock).
        # Unrotated the shell is only 24 m on X and the 35.24 m hall would burst through
        # the end walls. Rotating the building is cosmetic - it has no bearing on the
        # vehicle, whose spawn and the aisle are both placed in world coordinates.
        from pxr import UsdGeom
        UsdGeom.Xformable(self.world.stage.GetPrimAtPath("/World/layout")).AddRotateZOp().Set(90.0)

        # The racks: Isaac's stock pre-assembled shelf rows (from "Warehouse with
        # Shelves"), placed at the real site's aisle width from dpv_sim/aisle_spec.json.
        # The rack construction (levels, height) is the stock asset's - per the mentor,
        # only the aisle distance matters, and that is what the lidar scan-matches and
        # the mission flies down. The aisle runs along +Y (= PX4 North). To go back to
        # the spec-exact procedural racks (8 beam levels, measured pitch), swap this for
        # build_aisle(n_bays=8, with_pallets=True) from the same module.
        # rows_per_side=2 chains two stock rows end-to-end per side for a ~35.2 m hall.
        # That is the most the stock "Warehouse" shell holds (38.82 m interior on Y);
        # a third row would be 52.9 m and go through the end walls.
        aisle = build_aisle_stock(SIMULATION_ENVIRONMENTS["Warehouse with Shelves"],
                                  rows_per_side=2)
        carb.log_warn(
            "DPV aisle built (stock racks): {aisle_length_m:.2f} m long "
            "({rows_per_side} rows/side), {aisle_width_m:.3f} m wide, racks "
            "{rack_height_m:.2f} m tall, {rack_depth_m:.2f} m deep".format(**aisle)
        )

        # Create the vehicle
        # Try to spawn the selected robot in the world to the specified namespace
        # enable_zed_camera=True: the ZED's real-spec 1920x1200@30fps RGB-D render is by
        # far the most expensive thing Isaac does per frame (map geometry is not the
        # bottleneck - swapping warehouse USDs left RTF unchanged at ~0.52). With it on,
        # expect lower/jitterier RTF and slower EKF2 convergence than with it off - set
        # False if that becomes a problem while doing Phase 1-2 (examples/dpv_sim/PLAN.md)
        # localization-only testing. Note this does NOT drive the uXRCE timesync churn:
        # that comes from sim-time-vs-agent-wall-clock drift at any RTF != 1.0, and is
        # handled by PX4_PARAM_UXRCE_DDS_SYNCT=0 in dpv_sim/start.sh.
        config_multirotor = V1Config(enable_zed_camera=True)  # carries the real V1 thrust curve (2.8 kgf/motor, 2.0 kg)

        if GUIDANCE_MODE:
            # Guidance mode: keep GPS so PX4MavlinkBackend.send_gps_msgs() sends HIL_GPS
            # and EKF2 can fuse it (EKF2_GPS_CTRL is set GPS-enabled by start.sh phase g).
            config_multirotor.sensors = [Barometer(), IMU(), Magnetometer(), GPS()]
        else:
            # GPS-denied setup for SLAM testing: no GPS sensor -> no HIL_GPS is sent to PX4.
            # Horizontal position must come from SLAM (e.g. vision/odometry into PX4); until
            # then fly in Stabilized/Altitude mode (baro height only).
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
            # On the aisle centreline (Y=0, ~1.62 m off either rack face), 8.6 m in from
            # the west end of the hall (the racks span x = -17.62..+17.62). The aisle runs
            # along +X, and a mission waypoint's x is metres East of the spawn in the same
            # ENU frame (see build_aisle_stock), so mission x=6.5 flies from here to
            # x=-2.5 and mission x=10 to x=+1 - down the aisle, between the racks.
            #
            # z is the V1's *resting* height, NOT a hover-above-the-floor value. PX4 is
            # autolaunched when this Multirotor is constructed, but no IMU data flows until
            # timeline.play() below, so EKF2 sits waiting and then initialises its tilt from
            # the very first samples it ever sees. Spawn the vehicle any higher and those
            # first samples are a drop and bounce: free-fall reads 0 g (EKF2 rejects it) but
            # a ~2 g contact transient reads as specific force [0,0,+9.8] FRD - norm exactly
            # 1 g, so it passes EKF2's 0.8-1.2 g gate. That is antiparallel to the [0,0,-1]
            # in Ekf::initialiseTilt's Quatf(accel, {0,0,-1}), a degenerate case whose
            # "shortest rotation" is an arbitrary 180 deg flip about a horizontal axis. The
            # result is an EKF2 that is convinced the drone is upside down (roll ~180) with
            # cs_tilt_align true and no fault - and tilt is only ever initialised once, so at
            # rest it never recovers. Every downstream symptom follows: FD_FAIL_R trips
            # ("Preflight Fail: Attitude failure (roll)") and the mag heading, de-rotated
            # through the flipped tilt, comes out -90 instead of +90 ("heading estimate not
            # stable"), so arming is denied forever. Resting height measured from the sim.
            [-9.0, 0.0, 0.1124],
            # Identity: the nose points at Isaac +X = East, i.e. straight down the aisle,
            # which is also the mission frame's +x. (An earlier version yawed this 90 deg
            # left on the mistaken belief that a mission's x meant North; it does not -
            # the mission stack works in the ENU fcu_odom_flu frame, so x is East. With
            # the aisle on +X, East is down the hall and identity is correct.)
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
