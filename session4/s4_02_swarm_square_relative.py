"""
Session 4 · Script 2 — Swarm square (relative)  (FLIES)

Every drone flies the SAME square using relative moves, so they all trace the
shape in parallel regardless of where each one started. Place the drones spread
out on the floor and they'll fly parallel squares.

Run:  python s4_02_swarm_square_relative.py
Prereq: drones in config.FEW pass preflight and are placed >= ~0.8 m apart.
Note: relative squares keep spacing only if start spacing is respected —
      the drones cannot see each other.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.swarm import CachedCfFactory, Swarm

import cf_utils
import config


def take_off(scf):
    scf.cf.high_level_commander.takeoff(config.DEFAULT_HEIGHT, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)


def run_square(scf):
    hlc = scf.cf.high_level_commander
    for dx, dy in [(0.6, 0.0), (0.0, 0.6), (-0.6, 0.0), (0.0, -0.6)]:
        hlc.go_to(dx, dy, 0.0, 0.0, 2.5, relative=True)
        time.sleep(2.5)


def land(scf):
    scf.cf.high_level_commander.land(0.0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)
    scf.cf.high_level_commander.stop()


def main():
    uris = config.FEW
    cf_utils.init()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        print('  preparing {} drones ...'.format(len(uris)))
        swarm.parallel_safe(cf_utils.prepare_for_flight)
        swarm.parallel_safe(take_off)
        print('  flying squares')
        swarm.parallel_safe(run_square)
        swarm.parallel_safe(land)
    print('  done.')


if __name__ == '__main__':
    main()
