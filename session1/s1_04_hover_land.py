"""
Session 1 · Script 4 — Hover, then a positive landing  (FLIES — needs LPS)

Same idea as s1_03_hover_trigger.py, but it lands on its own terms: the drone
rises to 0.4 m, holds for 3 s, then descends 0.8 m — twice the hover height,
so the commanded height ends BELOW the floor and the drone is firmly down —
and only then are the motors cut.

Run:  python s1_04_hover_land.py
Prereq: LPS anchors on (TDoA2), LPS deck on the drone, drone INSIDE the box.
Safety: clear the area, glasses on, hand on the kill switch (Ctrl-C).
Note: this script sends its own hover setpoints instead of using
      MotionCommander. MotionCommander's land() only descends to z = 0 and
      then calls send_notify_setpoint_stop(), which hands the drone over to
      the high-level commander — that hand-over, right as the link closes, is
      what can make a drone lift off again instead of staying down. Here the
      last thing the drone hears is a throttle-zero stop, and nothing follows it.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config

DRONE_NUMBER = 9          # index into config.URIS (0 = drone 1, so 9 = drone 10)

HOVER_HEIGHT = 0.4        # m above the LPS floor (z = 0 of the anchor frame)
HOLD_S = 3.0              # how long to hover
DESCENT_M = 0.8           # descend this far, ending at -0.4 m: below the floor
CLIMB_RATE = 0.3          # m/s going up
DESCENT_RATE = 0.4        # m/s coming down
SETPOINT_PERIOD = 0.1     # s between setpoints; the firmware wants < 0.5 s


def send_height(cf, z):
    """One hover setpoint: no horizontal motion, no yaw, commanded height z.

    In a hover setpoint the height is ABSOLUTE in the estimator frame (the LPS
    floor), not a step — so we resend it continuously and the controller flies
    to it.
    """
    cf.commander.send_hover_setpoint(0.0, 0.0, 0.0, z)


def ramp_height(cf, z_from, z_to, rate):
    """Walk the commanded height from z_from to z_to at `rate` m/s."""
    duration = abs(z_to - z_from) / rate
    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        if elapsed >= duration:
            break
        send_height(cf, z_from + (z_to - z_from) * (elapsed / duration))
        time.sleep(SETPOINT_PERIOD)
    send_height(cf, z_to)


def hold_height(cf, z, seconds):
    """Sit at one commanded height, resending the setpoint so it doesn't time out."""
    t0 = time.time()
    while time.time() - t0 < seconds:
        send_height(cf, z)
        time.sleep(SETPOINT_PERIOD)


def main():
    uri = config.URIS[DRONE_NUMBER]
    cf_utils.init()
    print('Connecting to', uri)
    with cf_utils.make_scf(uri) as scf:
        if not cf_utils.lps_deck_present(scf):
            print('No LPS deck detected — aborting for safety.')
            return
        cf_utils.prepare_for_flight(scf)      # kalman + high-level + converged position
        input('Position estimate is good. Press ENTER to hover, Ctrl-C to abort... ')

        cf = scf.cf
        floor_z = HOVER_HEIGHT - DESCENT_M    # -0.4 m: below the floor on purpose
        try:
            print('  taking off to {:.2f} m ...'.format(HOVER_HEIGHT))
            ramp_height(cf, 0.0, HOVER_HEIGHT, CLIMB_RATE)
            print('  hovering...')
            hold_height(cf, HOVER_HEIGHT, HOLD_S)
            print('  descending {:.2f} m (commanded height {:+.2f} m) ...'.format(
                DESCENT_M, floor_z))
            ramp_height(cf, HOVER_HEIGHT, floor_z, DESCENT_RATE)
        finally:
            # Throttle to zero — always, including on Ctrl-C. emergency_stop()
            # also stops the high-level commander, so nothing is left holding a
            # setpoint that could fire once we stop talking to the drone.
            cf_utils.emergency_stop(scf)
            time.sleep(0.5)    # let the stop packet get out before the link closes
        print('  motors off.')


if __name__ == '__main__':
    main()
