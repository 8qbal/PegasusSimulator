#!/usr/bin/env python3
"""Set EKF2 GPS-denied parameters via PX4 onboard MAVLink link (udpin:127.0.0.1:14540).

QGroundControl latches the GCS link (UDP 18570), so param changes must go through
the onboard link (UDP 14540) instead. Parameters persist in EEPROM and take effect
on the next PX4 reboot.
"""

import time
from pymavlink import mavutil


ONBOARD_URL = 'udpin:127.0.0.1:14540'

PARAMS = {
    'EKF2_GPS_CTRL': 0,
    'EKF2_EV_CTRL': 11,
    'EKF2_HGT_REF': 3,
    'EKF2_EV_DELAY': 50,
}


def main():
    print(f'Connecting to PX4 onboard link at {ONBOARD_URL}...')
    mav = mavutil.mavlink_connection(ONBOARD_URL)

    try:
        mav.wait_heartbeat(timeout=10)
        print(f'Connected. System {mav.target_system}, component {mav.target_component}')

        for name, value in PARAMS.items():
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
