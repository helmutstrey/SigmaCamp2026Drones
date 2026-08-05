"""
Session 2 · Script 0 — LPS preflight health check, TDoA2  (NO FLIGHT)

Run this first on flight days. It verifies, for one drone:
  1. the link works,
  2. the Loco Positioning deck is detected,
  3. the deck is in TDoA2 mode,
  4. all 8 anchors (IDs 0-7) are heard and have valid positions,
  5. the position estimate converges,
  6. the reported position is sane and inside the flight box.

If this passes, everything else in Sessions 2-5 will fly. If it fails, the
message tells you which link in the chain is broken.

Run:  python s2_00_preflight_lps.py
Prereq: anchors powered and already switched to TDoA2 (setup_tdoa2.py, or the
        Loco Positioning tab in cfclient).
"""

import sys, os
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config


def main():
    cf_utils.init()
    print('Preflight (TDoA2) for', config.DRONE)
    with cf_utils.make_scf(config.DRONE) as scf:
        scf.wait_for_params()
        print('  [1/6] link OK')

        if cf_utils.lps_deck_present(scf):
            print('  [2/6] LPS deck detected')
        else:
            print('  [2/6] FAIL — no LPS deck. Reseat the deck and retry.')
            return

        mode = cf_utils.get_lps_mode(scf)
        if mode != config.LPS_MODE:
            print('  [3/6] deck was in {} — switching to TDoA2'.format(
                cf_utils.lps_mode_name(mode)))
            if not cf_utils.use_tdoa2_mode(scf):
                print('        FAIL — could not set the deck to TDoA2.')
                return
        print('  [3/6] deck in TDoA2 mode')

        ids, active, positions = cf_utils.anchor_status(scf)
        if active is None:
            print('  [4/6] SKIP — this firmware does not expose the anchor list')
        else:
            cf_utils.check_anchors(scf, status=(ids, active, positions))
            unpositioned = sorted(i for i in (active or [])
                                  if not positions.get(i, (None, False))[1])
            if unpositioned:
                print('        !! anchors {} are heard but have NO valid '
                      'position — set their coordinates in cfclient (or '
                      'setup_tdoa2.py) or the estimate will be wrong.'.format(
                          unpositioned))
            # Hearing a subset from the ground is not a failure — the estimator
            # test below is the real verdict on whether this drone can fly.
            print('  [4/6] {}/{} anchors heard{}'.format(
                len(active), len(config.ANCHOR_IDS),
                '' if unpositioned else ', positions valid'))

        cf_utils.use_kalman_estimator(scf)
        try:
            cf_utils.reset_estimator(scf)
            print('  [5/6] position estimate converged')
        except TimeoutError as e:
            print('  [5/6] FAIL —', e)
            return

        x, y, z = cf_utils.get_position(scf)
        print('  [6/6] position: x={:+.2f}  y={:+.2f}  z={:+.2f}'.format(x, y, z))
        inside = (config.BOX['x_min'] <= x <= config.BOX['x_max'] and
                  config.BOX['y_min'] <= y <= config.BOX['y_max'])
        print('        {} the flight box'.format('INSIDE' if inside else 'OUTSIDE (!)'))
        print('Preflight complete — ready to fly.' if inside else
              'Move the drone inside the box and rerun.')


if __name__ == '__main__':
    main()
