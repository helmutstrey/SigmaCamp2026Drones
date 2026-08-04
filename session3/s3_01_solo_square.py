"""
Session 3 · Script 1 — Solo square  (FLIES)

One drone flies a square in ABSOLUTE room coordinates using the high-level
commander. The waypoint list here is written out in full so students can see
exactly what a trajectory is: points + durations.

Run:  python s3_01_solo_square.py
Prereq: preflight (s2_00) passes.
"""

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config


def main():
    cx, cy = config.CENTER
    z = config.DEFAULT_HEIGHT
    # (x, y, z, duration_s) — a 1.2 m square centred on the room
    square = [
        (cx - 0.6, cy - 0.6, z, 2.5),
        (cx + 0.6, cy - 0.6, z, 2.5),
        (cx + 0.6, cy + 0.6, z, 2.5),
        (cx - 0.6, cy + 0.6, z, 2.5),
        (cx - 0.6, cy - 0.6, z, 2.5),
    ]

    cf_utils.init()
    with cf_utils.make_scf(config.DRONE) as scf:
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck — aborting.'); return
        cf_utils.prepare_for_flight(scf)
        print('  flying a square around the room centre ...')
        cf_utils.fly_figure(scf, square)
        print('  landed.')


if __name__ == '__main__':
    main()
