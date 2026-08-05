"""
Session 1 · Script 5 — Forward and back  (FLIES — needs LPS)

Single-drone example built on session5/s5_04_swarm_shapes.py's setup pattern: arm, pin
TDoA2, reset + wait for position lock, takeoff, move forward, move
back to start, land. Use this to sanity-check one drone's LOCO
position hold before running the full swarm script.

Run:  python s1_05_fly_forward_back.py
Prereq: preflight (session2/s2_00_preflight_lps.py) passes; drone starts
        INSIDE the box with its NOSE ALONG +x — the estimator assumes that
        heading, and "forward" here is +x.
Safety: area clear, glasses on. Ctrl-C lands the drone (it does not cut the
        motors — cf_utils.emergency_stop is the hard kill).
"""
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger

import config

URI = config.DRONE
MOVE_DISTANCE = 0.5   # metres, along +x ("forward")
MOVE_TIME = 2.0        # seconds per leg
VAR_CONVERGED_THRESHOLD = 0.001
CONVERGE_TIMEOUT = 10


def arm(scf):
    scf.cf.supervisor.send_arming_request(True)
    time.sleep(0.1)


def pin_tdoa2(scf):
    try:
        scf.cf.param.set_value('loco.mode', str(config.LPS_MODE))
    except KeyError:
        pass  # older firmware: mode is set on the anchors, not the CF
    scf.cf.param.set_value('stabilizer.estimator', '2')


def reset_estimator(scf):
    scf.cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    scf.cf.param.set_value('kalman.resetEstimation', '0')

    log_config = LogConfig(name='Kalman Variance', period_in_ms=200)
    log_config.add_variable('kalman.varPX', 'float')
    log_config.add_variable('kalman.varPY', 'float')
    log_config.add_variable('kalman.varPZ', 'float')

    print(f'Waiting for position lock (<{VAR_CONVERGED_THRESHOLD} var, '
          f'timeout {CONVERGE_TIMEOUT}s) ...')
    start = time.time()
    with SyncLogger(scf, log_config) as logger:
        for log_entry in logger:
            data = log_entry[1]
            if data['kalman.varPX'] < VAR_CONVERGED_THRESHOLD and \
               data['kalman.varPY'] < VAR_CONVERGED_THRESHOLD and \
               data['kalman.varPZ'] < VAR_CONVERGED_THRESHOLD:
                print('  locked.')
                return True
            if time.time() - start > CONVERGE_TIMEOUT:
                print('  NOT locked, aborting takeoff.')
                return False


def safe_go_to(scf, x, y, z, yaw, duration_s):
    """Clamped go_to - clips to config.BOX so a typo can't send the
    drone past the anchors' convex hull."""
    box = config.BOX
    x = min(max(x, box['x_min']), box['x_max'])
    y = min(max(y, box['y_min']), box['y_max'])
    z = min(max(z, box['z_min']), box['z_max'])
    scf.cf.high_level_commander.go_to(x, y, z, yaw, duration_s)
    time.sleep(duration_s)


def main():
    cflib.crtp.init_drivers()
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        print('Connected. Pinning TDoA2 ...')
        pin_tdoa2(scf)

        if not reset_estimator(scf):
            print('Aborting: fix position estimate first '
                  '(see session2/s2_00_preflight_lps.py).')
            return

        x0, y0 = config.CENTER
        z0 = config.DEFAULT_HEIGHT
        commander = scf.cf.high_level_commander

        try:
            print('Arming ...')
            arm(scf)

            print('Taking off ...')
            commander.takeoff(z0, config.TAKEOFF_TIME)
            time.sleep(config.TAKEOFF_TIME)

            print('Moving forward ...')
            safe_go_to(scf, x0 + MOVE_DISTANCE, y0, z0, 0, MOVE_TIME)

            print('Moving back to start ...')
            safe_go_to(scf, x0, y0, z0, 0, MOVE_TIME)

        except KeyboardInterrupt:
            print('\nInterrupted - landing now.')
        except Exception as e:
            print(f'\nError during flight: {e} - landing now.')
        finally:
            print('Landing ...')
            commander.land(0.0, config.TAKEOFF_TIME)
            time.sleep(config.TAKEOFF_TIME)
            commander.stop()


if __name__ == '__main__':
    main()
