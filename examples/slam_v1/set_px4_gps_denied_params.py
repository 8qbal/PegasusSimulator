#!/usr/bin/env python3
"""
| File: set_px4_gps_denied_params.py
| Description: Configure the running PX4 SITL for GPS-denied external-vision fusion.
|   Sets EKF2 parameters over the GCS MAVLink link (udp 18570) so EKF2 fuses the
|   vision odometry published by px4_vision_bridge.py instead of GPS.
| Run with the PegasusSimulator venv python (has pymavlink), while the sim + PX4 are running.
"""
import sys
import time
from pymavlink import mavutil

PARAMS = {
    "EKF2_GPS_CTRL": 0,    # disable GPS fusion entirely
    "EKF2_EV_CTRL": 11,    # fuse external vision: horizontal pos + vertical pos + yaw (bits 0|1|3)
    "EKF2_HGT_REF": 3,     # vision as the height reference
    "EKF2_EV_DELAY": 50.0, # vision measurement delay [ms] - SLAM at 10 Hz over DDS
}


def main():
    conn = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255)
    print("waiting for PX4 heartbeat on GCS link (udp 18570)...")
    conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
    hb = conn.wait_heartbeat(timeout=15)
    if hb is None:
        print("ERROR: no heartbeat - is the sim (and PX4 SITL) running?")
        sys.exit(1)

    for name, value in PARAMS.items():
        ptype = (
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            if isinstance(value, float)
            else mavutil.mavlink.MAV_PARAM_TYPE_INT32
        )
        conn.mav.param_set_send(1, 1, name.encode(), float(value), ptype)
        # PX4 broadcasts many unrelated PARAM_VALUE messages; wait for the ack
        # that matches this parameter name specifically.
        got = "NO ACK"
        deadline = time.time() + 5
        while time.time() < deadline:
            ack = conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
            if ack is not None and ack.param_id == name:
                got = f"{ack.param_value:g}"
                break
        print(f"  {name} -> {value}   (PX4 reports: {got})")
        time.sleep(0.2)

    print("done. EKF2 now expects vision odometry on /fmu/in/vehicle_visual_odometry.")
    print("note: restart PX4 (or reboot via QGC) if EKF2 was already running with GPS fused.")


if __name__ == "__main__":
    main()
