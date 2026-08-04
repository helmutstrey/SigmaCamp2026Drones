"""
Session 4 · Script 1 — Two drones, together  (FLIES)

Two drones take off, hover, and land in sync using the Swarm class.
parallel_safe() runs the same action on every drone at once — that parallelism
is what turns individuals into an ensemble.

Run:  python s4_01_two_drone_takeoff.py
Prereq: both drones in config.PAIR pass preflight; each on its assigned
        radio/channel; they are placed apart on the floor.
Safety: more drones = more caution. Ctrl-C cuts the script.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.swarm import CachedCfFactory, Swarm

import cf_utils
import config


def take_off(scf):
    scf.cf.high_level_commander.takeoff(config.DEFAULT_HEIGHT, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)


def land(scf):
    scf.cf.high_level_commander.land(0.0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)
    scf.cf.high_level_commander.stop()


def main():
    cf_utils.init()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(config.PAIR, factory=factory) as swarm:
        print('  preparing (reset estimators, enable high-level) ...')
        swarm.parallel_safe(cf_utils.prepare_for_flight)
        print('  take off')
        swarm.parallel_safe(take_off)
        print('  hover')
        time.sleep(3.0)
        print('  land')
        swarm.parallel_safe(land)
    print('  done.')


if __name__ == '__main__':
    main()
