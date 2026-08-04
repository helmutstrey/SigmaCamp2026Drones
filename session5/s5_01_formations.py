"""
Session 5 · Script 1 — Formations  (FLIES)

The swarm takes off into altitude lanes, forms a LINE, morphs to a CIRCLE,
returns to a LINE, and lands. This is the core building block of the show:
move every drone to an assigned point at the same time.

Run:  python s5_01_formations.py
Prereq: drones in FLY (below) pass preflight; config.HOMES gives each a lane.
Tune: keep the fleet small until transitions look clean. The drones cannot see
      each other — spacing and lanes are your only collision avoidance.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.swarm import CachedCfFactory, Swarm

import cf_utils
import config
import formations

FLY = config.FEW          # which drones perform (edit to taste)
MOVE_TIME = 3.0           # seconds per transition


def take_off_lane(scf):
    z = formations.lane_z(scf.cf.link_uri, 0)
    scf.cf.high_level_commander.takeoff(z, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)


def go(scf, target):
    x, y, z = cf_utils.safe_xyz(*target)
    scf.cf.high_level_commander.go_to(x, y, z, 0.0, MOVE_TIME)
    time.sleep(MOVE_TIME)


def land(scf):
    scf.cf.high_level_commander.land(0.0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)
    scf.cf.high_level_commander.stop()


def move_to(swarm, positions):
    swarm.parallel_safe(go, args_dict={u: [positions[u]] for u in positions})


def main():
    line = formations.line(FLY)
    circle = formations.circle(FLY)

    cf_utils.init()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(FLY, factory=factory) as swarm:
        print('  prepare + take off into lanes')
        swarm.parallel_safe(cf_utils.prepare_for_flight)
        swarm.parallel_safe(take_off_lane)

        print('  formation: line');   move_to(swarm, line);   time.sleep(1.5)
        print('  formation: circle'); move_to(swarm, circle); time.sleep(1.5)
        print('  formation: line');   move_to(swarm, line);   time.sleep(1.0)

        print('  land')
        swarm.parallel_safe(land)
    print('  done.')


if __name__ == '__main__':
    main()
