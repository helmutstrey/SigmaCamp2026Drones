"""
Session 1 · Script 3 — Triggered gentle hover  (FLIES — needs LPS)

Optional. If you don't have gamepads for manual flight, use this to let
students trigger a calm autonomous hover: the drone rises to 0.4 m, holds for
3 s, and lands. It also previews the autonomy you'll build on in Session 2.

Run:  python s1_03_hover_trigger.py
Prereq: LPS anchors on (TDoA2), LPS deck on the drone, drone INSIDE the box.
Safety: clear the area, glasses on, hand on the kill switch (Ctrl-C).
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.positioning.motion_commander import MotionCommander

import cf_utils
import config


def main():
    cf_utils.init()
    print('Connecting to', config.DRONE)
    with cf_utils.make_scf(config.DRONE) as scf:
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck detected — aborting for safety.')
            return
        cf_utils.prepare_for_flight(scf)      # kalman + high-level + converged position
        input('Position estimate is good. Press ENTER to hover, Ctrl-C to abort... ')

        # MotionCommander takes off on enter and lands on exit. All relative.
        with MotionCommander(scf, default_height=0.4) as mc:
            print('  hovering...')
            time.sleep(3.0)
        print('  landed.')


if __name__ == '__main__':
    main()
