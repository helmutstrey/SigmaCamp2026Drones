"""
setup_tdoa2.py  —  put the ANCHORS into TDoA2 mode  (NO FLIGHT)

Run this once per anchor set: after unboxing, after re-flashing an anchor,
or any time cfclient shows an anchor in the wrong mode. You can do exactly
the same thing by hand in cfclient's Loco Positioning tab — this script just
makes it repeatable and scriptable on a camp morning.

How it works: the drone is only the messenger. It broadcasts a short LPP
packet to each anchor; the anchor changes mode and resets itself. Anchors are
addressed from ID 7 down to 0 so the MASTER (anchor 0) switches last, and the
whole sweep repeats a few times because a resetting anchor can miss a packet.

Optionally also pushes anchor positions, but only if you filled in
config.ANCHOR_POSITIONS. Leave that empty and cfclient stays the single
source of truth for your calibration.

Run:  python setup_tdoa2.py
Prereq: all 8 anchors powered, one drone with an LPS deck powered and inside
        the anchor box so it can reach every anchor.
Note: anchors reboot as they switch — give them ~10 s to settle, then run
      session2/s2_00_preflight_lps.py to confirm all 8 come back.
"""

import sys, os, time
sys.path.append(os.path.dirname(__file__))

import cf_utils
import config


def main():
    cf_utils.init()
    print('Switching anchors {} to TDoA2 via {}'.format(
        config.ANCHOR_IDS, config.DRONE))

    with cf_utils.make_scf(config.DRONE) as scf:
        scf.wait_for_params()

        if not cf_utils.lps_deck_present(scf):
            print('  No LPS deck on this drone — it cannot reach the anchors.')
            return

        if config.ANCHOR_POSITIONS:
            print('  pushing {} anchor positions from config'.format(
                len(config.ANCHOR_POSITIONS)))
            from lpslib.lopoanchor import LoPoAnchor
            lopo = LoPoAnchor(scf.cf)
            for anchor_id, position in sorted(config.ANCHOR_POSITIONS.items()):
                lopo.set_position(anchor_id, position)
                time.sleep(0.05)
        else:
            print('  config.ANCHOR_POSITIONS is empty — leaving the anchor '
                  'geometry you set in cfclient alone.')

        print('  broadcasting TDoA2 mode (master last) ...')
        cf_utils.set_anchor_modes_tdoa2(scf)

        print('  waiting for anchors to reboot ...')
        time.sleep(10.0)

        cf_utils.use_tdoa2_mode(scf)
        cf_utils.check_anchors(scf)

    print('Done. Now run: python session2/s2_00_preflight_lps.py')


if __name__ == '__main__':
    main()
