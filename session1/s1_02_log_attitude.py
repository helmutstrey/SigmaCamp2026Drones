"""
Session 1 · Script 2 — Watch the sensors  (NO FLIGHT)

Streams the drone's estimated roll / pitch / yaw and battery for ~20 s.
Pick the drone up and tilt it — the numbers move. This is the IMU in action,
and it makes the "it senses its own motion" idea concrete.

Run:  python s1_02_log_attitude.py
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger

import cf_utils
import config


def main():
    cf_utils.init()
    print('Connecting to', config.DRONE, '— tilt the drone and watch roll/pitch move.')
    with cf_utils.make_scf(config.DRONE) as scf:
        log = LogConfig(name='attitude', period_in_ms=200)
        log.add_variable('stabilizer.roll', 'float')
        log.add_variable('stabilizer.pitch', 'float')
        log.add_variable('stabilizer.yaw', 'float')
        log.add_variable('pm.vbat', 'float')

        t0 = time.time()
        with SyncLogger(scf, log) as logger:
            for _, data, _ in logger:
                print('  roll={:+6.1f}  pitch={:+6.1f}  yaw={:+6.1f}   vbat={:.2f}V'.format(
                    data['stabilizer.roll'],
                    data['stabilizer.pitch'],
                    data['stabilizer.yaw'],
                    data['pm.vbat']))
                if time.time() - t0 > 20:
                    break
    print('Done.')


if __name__ == '__main__':
    main()
