"""
Session 2 · Script 1 — Hello, hover  (FLIES)

The first fully autonomous flight: take off to 0.5 m, hold 3 s, land.
Nobody touches a controller — we told it a height, not a thrust.

Run:  python s2_01_hello_hover.py
Prereq: preflight (s2_00) passes.
Safety: area clear, glasses on, Ctrl-C is the kill switch.
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
            print('  hovering at 0.5 m ...')
            time.sleep(3.0)
        print('  landed.')


if __name__ == '__main__':
    main()
