#!/usr/bin/env python3
"""Set EKF2 GPS-denied parameters via PX4 onboard MAVLink link (udpin:127.0.0.1:14540).

QGroundControl latches the GCS link (UDP 18570), so param changes must go through
the onboard link (UDP 14540) instead. Parameters persist in EEPROM and take effect
on the next PX4 reboot.

Profiles (--profile, default laser):
  laser  phases 1-3: EV = cartographer 2D laser odom (no valid Z -> horizontal only)
  zed    phase 4:    EV = real zed_wrapper VIO (stage D-1 starts identical to laser;
                     stage D-2/D-3 move to EV_CTRL=3 then 11 + HGT_REF=3 to match the
                     real drone once innovations look sane - use --ev-ctrl/--hgt-ref)
"""

import argparse
import time
from pymavlink import mavutil


ONBOARD_URL = 'udpin:127.0.0.1:14540'

# EKF2_EV_CTRL bits: 0=horiz pos, 1=vert pos, 2=velocity, 3=yaw.
# Horizontal position ONLY (=1): the 2D laser SLAM has no valid Z, so fusing EV
# vertical injected a ~6.8 km height and diverged the estimator. Height is baro
# (EKF2_HGT_REF=0). EV yaw is 90 deg off in NED and gets gated, so it's left off;
# mag provides heading. Re-enable laser heading later with EKF2_EV_CTRL=9.
# EKF2_MAG_CHECK=0: disable mag field-strength/inclination gate. GPS-denied it
# validates against a hardcoded average (not the sim's real Jakarta field), so in a
# clean sim it only risks spurious mag rejections that stall heading convergence.
# UXRCE_DDS_SYNCT=0: same timesync-churn fix start.sh applies at boot; kept here so
# re-applying params onboard cannot silently regress it.
COMMON = {
    'EKF2_GPS_CTRL': 0,
    'EKF2_EV_DELAY': 50,
    'EKF2_MAG_CHECK': 0,
    'UXRCE_DDS_SYNCT': 0,
}

PROFILES = {
    # Phases 1-3: cartographer laser odom as EV.
    'laser': {'EKF2_EV_CTRL': 1, 'EKF2_HGT_REF': 0},
    # Phase 4 stage D-1: ZED VIO as EV, same conservative fusion bits. Stage D-2:
    # --ev-ctrl 3 (ZED Z is sane, unlike 2D laser). Stage D-3 (real-drone eeprom
    # config): --ev-ctrl 11 --hgt-ref 3.
    'zed': {'EKF2_EV_CTRL': 1, 'EKF2_HGT_REF': 0},
}


def main():
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--profile', choices=sorted(PROFILES), default='laser')
    parser.add_argument('--ev-ctrl', type=int, default=None,
                        help='override EKF2_EV_CTRL (e.g. 3 or 11 for phase-4 staging)')
    parser.add_argument('--hgt-ref', type=int, default=None,
                        help='override EKF2_HGT_REF (3 = vision height, real-drone config)')
    args = parser.parse_args()

    params = dict(COMMON)
    params.update(PROFILES[args.profile])
    if args.ev_ctrl is not None:
        params['EKF2_EV_CTRL'] = args.ev_ctrl
    if args.hgt_ref is not None:
        params['EKF2_HGT_REF'] = args.hgt_ref
    print(f'Profile {args.profile}: '
          + ', '.join(f'{k}={v}' for k, v in sorted(params.items())))

    print(f'Connecting to PX4 onboard link at {ONBOARD_URL}...')
    mav = mavutil.mavlink_connection(ONBOARD_URL)

    try:
        mav.wait_heartbeat(timeout=10)
        print(f'Connected. System {mav.target_system}, component {mav.target_component}')

        for name, value in params.items():
            print(f'Setting {name} = {value}...')
            mav.mav.param_set_send(
                mav.target_system,
                mav.target_component,
                name.encode('utf-8'),
                float(value),
                mavutil.mavlink.MAV_PARAM_TYPE_INT32,
            )
            time.sleep(0.3)

        print('All params set. Reboot PX4 for changes to take effect.')
        mav.mav.command_long_send(
            mav.target_system,
            mav.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
            0, 1, 0, 0, 0, 0, 0, 0,
        )
        print('Reboot command sent.')

    except Exception as e:
        print(f'Error: {e}')
        return 1
    finally:
        mav.close()

    return 0


if __name__ == '__main__':
    exit(main())
