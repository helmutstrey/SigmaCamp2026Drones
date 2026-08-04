"""
Session 5 · Script 2 — Ceremony show  (FLIES)  ★ the deliverable ★

A complete, phased routine you can run at the closing ceremony:

    take off -> LINE -> CIRCLE -> SPLIT (two rows) -> regroup LINE -> land

Every drone keeps its own altitude lane throughout, so paths stay vertically
separated. Edit the PHASES list to design your piece — add formations, change
the order, insert holds. Keep it simple and reliable; a clean 6-8 drone show
beats a nervous 10.

Run:  python s5_02_ceremony_show.py
Prereq: drones in FLY pass preflight; config.HOMES gives each a distinct lane.
Safety: rehearse the abort (s5_03) first. A flight director watches every run.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.swarm import CachedCfFactory, Swarm

import cf_utils
import config
import formations

FLY = config.FEW
MOVE_TIME = 3.0

# The show as a list of (label, positions-dict, hold_seconds).
# Build positions from the formation library; reorder freely.
def build_phases():
    return [
        ('line',        formations.line(FLY),      1.5),
        ('circle',      formations.circle(FLY),    2.0),
        ('split rows',  formations.two_rows(FLY),  2.0),
        ('regroup line', formations.line(FLY),     1.5),
    ]


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


def main():
    phases = build_phases()
    print('Ceremony show — {} drones, {} phases.'.format(len(FLY), len(phases)))

    cf_utils.init()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(FLY, factory=factory) as swarm:
        print('  prepare + take off')
        swarm.parallel_safe(cf_utils.prepare_for_flight)
        swarm.parallel_safe(take_off_lane)

        for label, positions, hold in phases:
            print('  -> {}'.format(label))
            swarm.parallel_safe(go, args_dict={u: [positions[u]] for u in positions})
            time.sleep(hold)

        print('  finale: land')
        swarm.parallel_safe(land)
    print('  show complete.')


if __name__ == '__main__':
    main()
