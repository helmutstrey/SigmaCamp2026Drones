"""
Session 2 · Script 1 — Hello, hover  (FLIES)

The first fully autonomous flight: take off to 0.5 m, hold 3 s, land.
Nobody touches a controller — we told it a height, not a thrust.

Run:  python s2_01_hello_hover.py
Prereq: preflight (s2_00) passes.
Safety: area clear, glasses on, Ctrl-C is the kill switch.
Note: this flies with the high-level commander rather than MotionCommander.
      MotionCommander is built on send_hover_setpoint(), which commands x/y
      VELOCITY — right for the Flow deck, which measures velocity directly,
      but on LPS nothing closes the loop on x/y, so "hold still" becomes a
      slow drift that ends at a wall. takeoff() and land() hold the x-y
      position they started from.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config

HEIGHT = 0.5      # m above the LPS floor
HOLD_S = 3.0      # how long to hover


def main():
    cf_utils.init()
    with cf_utils.make_scf(config.DRONE) as scf:
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck — aborting.'); return
        cf_utils.prepare_for_flight(scf)

        hlc = scf.cf.high_level_commander
        try:
            print('  taking off to {:.1f} m ...'.format(HEIGHT))
            hlc.takeoff(HEIGHT, config.TAKEOFF_TIME, yaw=None)   # yaw=None: keep heading
            time.sleep(config.TAKEOFF_TIME)

            print('  hovering at {:.1f} m ...'.format(HEIGHT))
            time.sleep(HOLD_S)

            print('  landing ...')
            hlc.land(0.0, config.TAKEOFF_TIME, yaw=None)
            time.sleep(config.TAKEOFF_TIME)
            hlc.stop()
            print('  landed.')
        except KeyboardInterrupt:
            # The high-level commander keeps flying its last trajectory even if
            # we stop talking to it, so Ctrl-C has to cut the motors itself.
            cf_utils.emergency_stop(scf)
            print('\n  aborted — motors off.')


if __name__ == '__main__':
    main()
