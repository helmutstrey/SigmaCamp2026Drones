"""
Session 1 · Script 1 — Connect test  (NO FLIGHT)

Confirms the whole chain works: Crazyradio driver -> radio link -> drone.
Prints the link and the battery voltage, then disconnects.

Run:  python s1_01_connect_test.py
Prereq: 1 drone powered on, Crazyradio plugged in, config.DRONE set to its URI.
"""

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config


def main():
    cf_utils.init()
    uri = config.DRONE
    print('Connecting to', uri, '...')
    with cf_utils.make_scf(uri) as scf:
        print('  connected!')
        vbat = cf_utils.battery_voltage(scf)
        print('  battery: {:.2f} V'.format(vbat))
        if vbat < 3.7:
            print('  (low-ish — charge before flying)')
        print('Disconnecting.')


if __name__ == '__main__':
    main()
