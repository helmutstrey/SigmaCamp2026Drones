"""
Session 3 · Script 3 — Timing is choreography  (FLIES)

Flies the same square twice: once slow (graceful), once fast (snappy). Same
path, different feel — the point being that 'duration' is an expressive choice,
not just a number.

Run:  python s3_03_timing_demo.py
Prereq: preflight (s2_00) passes.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config


def square(dur):
    cx, cy = config.CENTER
    z = config.DEFAULT_HEIGHT
    return [
        (cx - 0.6, cy - 0.6, z, dur),
        (cx + 0.6, cy - 0.6, z, dur),
        (cx + 0.6, cy + 0.6, z, dur),
        (cx - 0.6, cy + 0.6, z, dur),
        (cx - 0.6, cy - 0.6, z, dur),
    ]


def main():
    cf_utils.init()
    with cf_utils.make_scf(config.DRONE) as scf:
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck — aborting.'); return

        cf_utils.prepare_for_flight(scf)
        print('  slow square (3.0 s / side) ...')
        cf_utils.fly_figure(scf, square(3.0))

        time.sleep(2.0)   # swap a battery here if you like

        cf_utils.prepare_for_flight(scf)
        print('  fast square (1.2 s / side) ...')
        cf_utils.fly_figure(scf, square(1.2))
        print('  landed.')


if __name__ == '__main__':
    main()
