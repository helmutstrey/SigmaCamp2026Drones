"""
Session 2 · Script 3 — Watch positioning work  (NO FLIGHT)

Prints the estimated (x, y, z) for ~20 s. Carry the drone around the box by
hand and watch the numbers track your movement — this is TDoA2 locating the
drone in the room. Try walking it toward a corner, then outside the box, and
watch the estimate get noisy: the "stay inside the box" rule, made visible.

It also lists which of the 8 anchors the drone can hear right now. Stand
between the drone and one anchor and re-run it: losing an anchor is exactly
what a bad flight looks like from the drone's point of view.

Run:  python s2_03_print_position.py
"""

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config


def main():
    cf_utils.init()
    with cf_utils.make_scf(config.DRONE) as scf:
        scf.wait_for_params()
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck — aborting.'); return
        cf_utils.use_tdoa2_mode(scf)
        cf_utils.check_anchors(scf)
        cf_utils.use_kalman_estimator(scf)
        cf_utils.reset_estimator(scf)
        print('Move the drone by hand:')
        cf_utils.stream_position(scf, duration_s=20.0)
    print('Done.')


if __name__ == '__main__':
    main()
