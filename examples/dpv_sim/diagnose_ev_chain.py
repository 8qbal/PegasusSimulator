#!/usr/bin/env python3
"""One-shot diagnostic for the external-vision (EV) aiding chain in the DPV sim.

Traces every link that must work for EKF2 to get a bounded position estimate and
stop the "High Accelerometer Bias / Attitude failure" divergence. Two EV sources
exist depending on the start.sh phase:

  [1] from_fcu_vehicle_odometry_node  -> /px4_ros2_bridge/odometry/fcu_odom_flu
  [2a] (phase 1-3, laser EV) cartographer_pose_transformer -> /cartographer/laser_odom_at_fcu
  [2b] (phase 4, ZED EV) zed_wrapper VIO -> /zed/zed_node/odom
       -> zed_odom_to_fcu.py -> /zed/zed_node/odom_zed_to_fcu
  [3] to_fcu_vehicle_visual_odometry  -> /fmu/in/vehicle_visual_odometry
  [4] EKF2                            -> /fmu/out/estimator_status_flags (cs_ev_* / reject_*)

`--wait [--timeout N]` blocks until EKF2 is mission-ready (EV position fused, not
rejected, attitude level) — gate `load_mission.sh load/start` on this. Exit code
0 = ready, 1 = timed out. See also examples/dpv_sim/ev_ready.sh.

Subscribes BEST_EFFORT + VOLATILE everywhere (maximally compatible: matches both
BEST_EFFORT and RELIABLE publishers, both VOLATILE and TRANSIENT_LOCAL) so a
non-zero rate reflects the real data flow, not a QoS mismatch on our side. Also
enumerates publishers per topic to catch stale/duplicate publishers (e.g. a
leftover slam_v1 px4_vision_bridge still writing a wrong-frame EV pose).

Run while the sim is up:
  source /opt/ros/humble/setup.bash
  source ~/PegasusSimulator/extensions/dpv-install/setup.bash
  python3 ~/PegasusSimulator/examples/dpv_sim/diagnose_ev_chain.py
"""
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry, EstimatorStatusFlags, VehicleAttitude

# Maximally compatible reader QoS.
PERMISSIVE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
)

DURATION = 12.0

TOPICS = {
    "fcu_odom": ("/px4_ros2_bridge/odometry/fcu_odom_flu", Odometry),
    "laser_odom": ("/cartographer/laser_odom_at_fcu", Odometry),
    "zed_odom": ("/zed/zed_node/odom", Odometry),
    "zed_ev": ("/zed/zed_node/odom_zed_to_fcu", Odometry),
    "ev_in": ("/fmu/in/vehicle_visual_odometry", VehicleOdometry),
    "flags": ("/fmu/out/estimator_status_flags", EstimatorStatusFlags),
    "att": ("/fmu/out/vehicle_attitude", VehicleAttitude),
}


def q_to_rpy_deg(q):
    """q = [w,x,y,z] -> (roll, pitch, yaw) in degrees."""
    w, x, y, z = q
    roll = math.degrees(math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)))
    sinp = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.degrees(math.asin(sinp))
    yaw = math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))
    return roll, pitch, yaw


def wait_ready(node, state, timeout):
    """Block until EKF2 is mission-ready: EV position fused, innovation gate not
    rejecting, attitude level (|roll| and |pitch| < 5 deg). Progress every 2 s."""
    t0 = time.time()
    last_print = -10.0
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)
        f = state["flags"]["last"]
        att = state["att"]["last"]
        fused = f is not None and f.cs_ev_pos and not f.reject_hor_pos
        rpy = None
        level = False
        if att is not None:
            r, p, _ = q_to_rpy_deg(att.q)
            rpy = (r, p)
            level = abs(r) < 5.0 and abs(p) < 5.0
        if fused and level:
            print(f"READY after {time.time() - t0:.1f}s: cs_ev_pos=True "
                  f"reject_hor_pos=False roll={rpy[0]:+.1f} pitch={rpy[1]:+.1f} deg")
            return True
        if time.time() - last_print >= 2.0:
            last_print = time.time()
            att_str = f"{rpy[0]:+.1f},{rpy[1]:+.1f} deg" if rpy else "n/a"
            print(f"waiting {time.time() - t0:5.1f}s  ev_fused={fused}  "
                  f"attitude={att_str}  flags={'yes' if f is not None else 'none yet'}")
    print(f"TIMEOUT after {timeout:.0f}s: EKF2 not mission-ready. "
          "Run without --wait for the full chain diagnosis.")
    return False


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--wait", action="store_true",
                        help="block until EKF2 is mission-ready (EV fused + attitude level)")
    parser.add_argument("--timeout", type=float, default=120.0,
                        help="--wait timeout in seconds (default 120)")
    args = parser.parse_args()

    rclpy.init()
    node = Node("diagnose_ev_chain")

    state = {k: {"n": 0, "last": None} for k in TOPICS}

    def mk(key):
        def cb(msg):
            state[key]["n"] += 1
            state[key]["last"] = msg
        return cb

    for key, (topic, msg_type) in TOPICS.items():
        node.create_subscription(msg_type, topic, mk(key), PERMISSIVE_QOS)

    if args.wait:
        ok = wait_ready(node, state, args.timeout)
        node.destroy_node()
        rclpy.shutdown()
        return 0 if ok else 1

    print(f"Listening for {DURATION:.0f}s...\n")
    t0 = time.time()
    while time.time() - t0 < DURATION:
        rclpy.spin_once(node, timeout_sec=0.1)

    def rate(key):
        return state[key]["n"] / DURATION

    # Publisher census (catches stale duplicates / QoS of the real publishers).
    print("=" * 64)
    print("PUBLISHER CENSUS")
    print("=" * 64)
    for key, (topic, _) in TOPICS.items():
        infos = node.get_publishers_info_by_topic(topic)
        print(f"{topic}")
        print(f"    publishers: {len(infos)}")
        for info in infos:
            rel = info.qos_profile.reliability.name
            dur = info.qos_profile.durability.name
            print(f"      - {info.node_name} [{rel}/{dur}]")
        if len(infos) > 1 and key in ("ev_in", "fcu_odom", "laser_odom"):
            print("      !!! MORE THAN ONE PUBLISHER — a stale node may be injecting a bad frame")

    print("\n" + "=" * 64)
    print("LINK-BY-LINK (each must be > 0 Hz for the chain to work)")
    print("=" * 64)
    print(f"[1]  from_fcu -> fcu_odom_flu           : {rate('fcu_odom'):6.1f} Hz  ({state['fcu_odom']['n']} msgs)")
    print(f"[2a] pose_transformer -> laser_odom     : {rate('laser_odom'):6.1f} Hz  ({state['laser_odom']['n']} msgs)  (EV source, phases 1-3)")
    print(f"[2b] zed_wrapper -> zed_node/odom       : {rate('zed_odom'):6.1f} Hz  ({state['zed_odom']['n']} msgs)")
    print(f"[2b] glue -> zed_node/odom_zed_to_fcu   : {rate('zed_ev'):6.1f} Hz  ({state['zed_ev']['n']} msgs)  (EV source, phase 4)")
    print(f"[3]  to_fcu -> vehicle_visual_odometry  : {rate('ev_in'):6.1f} Hz  ({state['ev_in']['n']} msgs)")
    print(f"[4]  EKF2 -> estimator_status_flags     : {rate('flags'):6.1f} Hz  ({state['flags']['n']} msgs)")

    # First broken link (informational; does not gate the EKF2 flag readout below).
    print("\n" + "=" * 64)
    print("DIAGNOSIS")
    print("=" * 64)
    ev_sources = []
    if state["laser_odom"]["n"] > 0:
        ev_sources.append("laser")
    if state["zed_ev"]["n"] > 0:
        ev_sources.append("zed")
    if state["fcu_odom"]["n"] == 0:
        print("BREAK AT [1]: from_fcu_vehicle_odometry_node not publishing.")
        print("  -> ros2 node list | grep from_fcu_vehicle_odometry")
        print("  -> Does /fmu/out/vehicle_odometry have data? (PX4 up + agent connected?)")
    elif not ev_sources:
        print("BREAK AT [2]: no EV source publishing (neither laser_odom_at_fcu nor")
        print("  odom_zed_to_fcu).")
        print("  -> phases 1-3: check bringup pane for 'Waiting for FCU odometry to initialize...'")
        print("  -> phase 4: is zed_wrapper connected to the Isaac stream (tmux window 'zed')")
        print("     and is zed_odom_to_fcu.py running? ros2 topic hz /zed/zed_node/odom")
    elif state["ev_in"]["n"] == 0:
        print("BREAK AT [3]: to_fcu not publishing /fmu/in/vehicle_visual_odometry.")
        print("  -> phase 4 note: to_fcu subscribes /zed/zed_node/odom_zed_to_fcu (the")
        print("     bridge_params.yaml default); phases 1-3 override it to laser_odom_at_fcu.")
    else:
        print(f"Links [1]-[3] all flowing (active EV source(s): {', '.join(ev_sources)}).")
        if len(ev_sources) == 2:
            print("  NOTE: both laser and ZED odom present — only the one to_fcu subscribes")
            print("  feeds EKF2; the other runs in shadow mode for A/B comparison.")

    # Always show EKF2 fusion state if we have it.
    f = state["flags"]["last"]
    if f is not None:
        print("\nEKF2 fusion intent flags:")
        print(f"  cs_ev_pos = {f.cs_ev_pos}   cs_ev_yaw = {f.cs_ev_yaw}   "
              f"cs_ev_hgt = {f.cs_ev_hgt}   cs_ev_vel = {f.cs_ev_vel}")
        print(f"  reject_hor_pos = {f.reject_hor_pos}   reject_ver_pos = {f.reject_ver_pos}   "
              f"reject_yaw = {f.reject_yaw}   cs_ev_yaw_fault = {f.cs_ev_yaw_fault}")
        if not f.cs_ev_pos:
            print("  >>> cs_ev_pos FALSE: EKF2 is NOT fusing EV position.")
            print("      Check `param show EKF2_EV_CTRL` in the px4 console — expect 1")
            print("      (horizontal pos only; phase-4 staged configs may use 3 or 11),")
            print("      and that the EV sample below has pose_frame=1 (NED) + finite position.")
        elif f.reject_hor_pos:
            print("  >>> cs_ev_pos TRUE but reject_hor_pos TRUE: EV fails the innovation gate")
            print("      (frame/sign mismatch or a jumping offset). See EV sample vs attitude below.")
        else:
            print("  >>> EV position IS being fused.")
    else:
        print("\nNo estimator_status_flags received (EKF2 down or DDS export issue).")

    # Samples
    print("\n" + "=" * 64)
    print("SAMPLES")
    print("=" * 64)
    ev = state["ev_in"]["last"]
    if ev is not None:
        er, ep, ey = q_to_rpy_deg(ev.q)
        print(f"EV: pose_frame={ev.pose_frame} (1=NED,2=FRD)  pos={[round(v,3) for v in ev.position]}")
        print(f"    q={[round(v,3) for v in ev.q]}  -> rpy=({er:+.1f},{ep:+.1f},{ey:+.1f}) deg")
        print(f"    pos_var={[round(v,4) for v in ev.position_variance]}")
    lo = state["laser_odom"]["last"]
    if lo is not None:
        p = lo.pose.pose.position
        o = lo.pose.pose.orientation
        lr, lp, ly = q_to_rpy_deg([o.w, o.x, o.y, o.z])
        print(f"laser_odom: frame={lo.header.frame_id} pos=({p.x:.3f},{p.y:.3f},{p.z:.3f}) "
              f"rpy=({lr:+.1f},{lp:+.1f},{ly:+.1f}) deg")
    att = state["att"]["last"]
    if att is not None:
        r, p, y = q_to_rpy_deg(att.q)
        print(f"EKF2 attitude: rpy=({r:+.1f},{p:+.1f},{y:+.1f}) deg  (roll/pitch ~0 if level)")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
