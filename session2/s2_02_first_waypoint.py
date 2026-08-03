"""
Session 2 · Script 2 — First waypoint  (FLIES)

Take off, move 0.5 m to one side, come back, land. Uses relative moves, so it
works no matter where on the floor the drone starts. This is the idea a
trajectory is built from: a sequence of moves.

Run:  python s2_02_first_waypoint.py
Prereq: preflight (s2_00) passes.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.positioning.motion_commander import MotionCommander

import cf_utils
import config


def main():
    cf_utils.init()
    with cf_utils.make_scf(config.DRONE) as scf:
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck — aborting.'); return
        cf_utils.prepare_for_flight(scf)

        with MotionCommander(scf, default_height=0.5) as mc:
            time.sleep(1.0)
            print('  right 0.5 m'); mc.right(0.5); time.sleep(0.5)
            print('  left 0.5 m (back to start)'); mc.left(0.5); time.sleep(0.5)
        print('  landed.')


if __name__ == '__main__':
    main()
