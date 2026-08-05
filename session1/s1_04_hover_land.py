"""
Session 1 · Script 4 — Hover, then a positive landing  (FLIES — needs LPS)

Same idea as s1_03_hover_trigger.py, but it holds a POSITION and it lands on
its own terms: the drone notes where it is standing, rises to 0.4 m over that
spot, holds for 3 s, then descends 0.8 m — twice the hover height, so the
commanded height ends BELOW the floor and the drone is firmly down — and only
then are the motors cut.

It is also the noisy one: it prints the LPS mode, the anchors it hears, and
the position the drone thinks it is at, before and throughout the flight. If
something misbehaves, read those numbers first.

Run:  python s1_04_hover_land.py
Prereq: LPS anchors on (TDoA2), LPS deck on the drone, drone INSIDE the box.
Safety: clear the area, glasses on, hand on the kill switch (Ctrl-C).

Why this script does not use MotionCommander, twice over:

  1. Position, not velocity. MotionCommander is built on
     send_hover_setpoint(), which commands x/y VELOCITY in body coordinates —
     it is designed for the Flow deck, which measures velocity directly.
     Asking an LPS drone for "zero velocity" does not ask it to stay put:
     nothing closes the loop on x/y, so any bias in the estimated velocity
     integrates into a drift that keeps going until it meets a wall. This
     script sends send_position_setpoint() instead — absolute world x/y/z,
     which is what the LPS estimate actually measures.
  2. Landing. MotionCommander's land() only descends to z = 0 and then calls
     send_notify_setpoint_stop(), handing the drone to the high-level
     commander — a hand-over right as the link closes is what can make a drone
     lift off again. Here the last thing the drone hears is a throttle-zero
     stop, and nothing follows it.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.log import LogConfig

import cf_utils
import config

DRONE_NUMBER = 9          # index into config.URIS (0 = drone 1, so 9 = drone 10)

HEADING_DEG = 0.0         # which way the drone's NOSE points, 0 = the LPS +x
                          # axis. The estimator has no compass indoors: it
                          # believes this number. Get it wrong and every
                          # correction is rotated by the error — 180 deg off
                          # and the drone accelerates away from its target.
                          # Easiest is to leave this 0 and physically point
                          # every drone along +x before takeoff.

HOVER_HEIGHT = 0.4        # m above the LPS floor (z = 0 of the anchor frame)
HOLD_S = 3.0              # how long to hover
DESCENT_M = 0.8           # descend this far, ending at -0.4 m: below the floor
CLIMB_RATE = 0.3          # m/s going up
DESCENT_RATE = 0.4        # m/s coming down
SETPOINT_PERIOD = 0.1     # s between setpoints; the firmware wants < 0.5 s
PRINT_PERIOD = 0.5        # s between position printouts while flying


class PositionLog:
    """Background log of the position the DRONE thinks it is at.

    cf_utils.get_position() opens a fresh log block per reading, which is fine
    for a one-shot check but too slow to call inside the setpoint loop. This
    keeps one block running and just remembers the latest sample.
    """

    def __init__(self, scf, period_ms=100):
        self._latest = None
        self._log = LogConfig(name='s1_04_pos', period_in_ms=period_ms)
        self._log.add_variable('stateEstimate.x', 'float')
        self._log.add_variable('stateEstimate.y', 'float')
        self._log.add_variable('stateEstimate.z', 'float')
        self._log.add_variable('stateEstimate.yaw', 'float')
        self._log.data_received_cb.add_callback(self._on_data)
        scf.cf.log.add_config(self._log)

    def _on_data(self, timestamp, data, logconf):
        self._latest = (data['stateEstimate.x'],
                        data['stateEstimate.y'],
                        data['stateEstimate.z'],
                        data['stateEstimate.yaw'])

    def start(self):
        self._log.start()

    def stop(self):
        try:
            self._log.stop()
            self._log.delete()
        except Exception:
            pass      # tearing down a log block must never block the motor cut

    def latest(self):
        """(x, y, z, yaw) of the most recent sample, or None."""
        return self._latest

    def wait_for_sample(self, timeout_s=5.0):
        t0 = time.time()
        while self._latest is None:
            if time.time() - t0 > timeout_s:
                raise TimeoutError('No position samples from the drone.')
            time.sleep(0.05)
        return self._latest

    def text(self):
        if self._latest is None:
            return 'position: (no sample yet)'
        return 'position: x={:+.2f}  y={:+.2f}  z={:+.2f}'.format(*self._latest[:3])


def send_target(cf, x, y, z, yaw):
    """One absolute position setpoint, in LPS world coordinates.

    Unlike a hover setpoint, x and y here are a PLACE, not a velocity — this is
    what makes the drone fight a drift instead of riding it.
    """
    cf.commander.send_position_setpoint(x, y, z, yaw)


def _print_progress(pos, x, y, z, last_print):
    """Print commanded vs. measured every PRINT_PERIOD seconds."""
    now = time.time()
    if pos is None or now - last_print < PRINT_PERIOD:
        return last_print
    print('    target x={:+.2f} y={:+.2f} z={:+.2f}   {}'.format(x, y, z, pos.text()))
    return now


def ramp_height(cf, x, y, yaw, z_from, z_to, rate, pos=None):
    """Hold (x, y) while walking the commanded height from z_from to z_to."""
    duration = abs(z_to - z_from) / rate
    t0 = time.time()
    last_print = 0.0
    while True:
        elapsed = time.time() - t0
        if elapsed >= duration:
            break
        z = z_from + (z_to - z_from) * (elapsed / duration)
        send_target(cf, x, y, z, yaw)
        last_print = _print_progress(pos, x, y, z, last_print)
        time.sleep(SETPOINT_PERIOD)
    send_target(cf, x, y, z_to, yaw)


def hold_position(cf, x, y, z, yaw, seconds, pos=None):
    """Sit on one target, resending it so the setpoint doesn't time out."""
    t0 = time.time()
    last_print = 0.0
    while time.time() - t0 < seconds:
        send_target(cf, x, y, z, yaw)
        last_print = _print_progress(pos, x, y, z, last_print)
        time.sleep(SETPOINT_PERIOD)


def check_positioning(scf):
    """Confirm the deck really is ranging in TDoA2 off all 8 anchors.

    Done BEFORE prepare_for_flight() so a wrong mode aborts here with a clear
    message, instead of surfacing 20 s later as an estimator timeout.
    """
    if not cf_utils.lps_deck_present(scf):
        print('  No LPS deck detected — aborting for safety.')
        return False

    cf_utils.use_tdoa2_mode(scf)                 # pin loco.mode = config.LPS_MODE
    mode = cf_utils.get_lps_mode(scf)
    print('  loco.mode = {} ({})'.format(mode, cf_utils.lps_mode_name(mode)))
    if mode != config.LPS_MODE:
        print('  Deck is NOT in TDoA2 — aborting. Run setup_tdoa2.py (anchors) '
              'or session2/s2_00_preflight_lps.py to see which link is broken.')
        return False

    if not cf_utils.check_anchors(scf):
        print('  Anchors are not all heard — aborting before takeoff.')
        return False
    return True


def main():
    uri = config.URIS[DRONE_NUMBER]
    cf_utils.init()
    print('Connecting to', uri)
    with cf_utils.make_scf(uri) as scf:
        scf.wait_for_params()                    # needed before reading params / cf.mem
        if not check_positioning(scf):
            return

        # kalman + high-level + initial heading + converged position
        cf_utils.prepare_for_flight(scf, initial_yaw_deg=HEADING_DEG)

        pos = PositionLog(scf)
        pos.start()
        try:
            x, y, z, yaw = pos.wait_for_sample()
            print('  resting position: x={:+.2f}  y={:+.2f}  z={:+.2f}  yaw={:+.1f}'.format(
                x, y, z, yaw))
            inside = (config.BOX['x_min'] <= x <= config.BOX['x_max'] and
                      config.BOX['y_min'] <= y <= config.BOX['y_max'])
            print('  {} the flight box (x {}..{}, y {}..{})'.format(
                'INSIDE' if inside else '!! OUTSIDE',
                config.BOX['x_min'], config.BOX['x_max'],
                config.BOX['y_min'], config.BOX['y_max']))
            if abs(z) > 0.1:
                print('  !! resting z is {:+.2f}, not ~0 — the commanded heights below '
                      'are absolute in the LPS frame, so this drone would fly '
                      '{:+.2f} m off from what you asked for.'.format(z, -z))

            print('  CHECK THE NOSE: this drone must be facing {:.0f} deg '
                  '({}), or it will fly the wrong way.'.format(
                      HEADING_DEG,
                      'the +x axis' if HEADING_DEG == 0.0 else
                      '{:.0f} deg counter-clockwise from +x'.format(HEADING_DEG)))
            input('Position estimate is good. Press ENTER to hover, Ctrl-C to abort... ')

            # Take the spot to hold from the drone itself, clamped into the box.
            # Only x/y are clamped: the descent below deliberately goes under
            # BOX['z_min'] to press the drone onto the floor, and safe_xyz()
            # would clamp that away.
            hold_x, hold_y, _ = cf_utils.safe_xyz(*pos.latest()[:2], HOVER_HEIGHT)
            hold_yaw = pos.latest()[3]
            print('  holding x={:+.2f}  y={:+.2f}  yaw={:+.1f}'.format(
                hold_x, hold_y, hold_yaw))

            cf = scf.cf
            floor_z = HOVER_HEIGHT - DESCENT_M   # -0.4 m: below the floor on purpose
            print('  taking off to {:.2f} m ...'.format(HOVER_HEIGHT))
            ramp_height(cf, hold_x, hold_y, hold_yaw, 0.0, HOVER_HEIGHT, CLIMB_RATE, pos)
            print('  hovering...')
            hold_position(cf, hold_x, hold_y, HOVER_HEIGHT, hold_yaw, HOLD_S, pos)
            print('  descending {:.2f} m (commanded height {:+.2f} m) ...'.format(
                DESCENT_M, floor_z))
            ramp_height(cf, hold_x, hold_y, hold_yaw, HOVER_HEIGHT, floor_z,
                        DESCENT_RATE, pos)
        finally:
            # Throttle to zero — always, including on Ctrl-C. emergency_stop()
            # also stops the high-level commander, so nothing is left holding a
            # setpoint that could fire once we stop talking to the drone.
            cf_utils.emergency_stop(scf)
            pos.stop()
            time.sleep(0.5)    # let the stop packet get out before the link closes
        print('  motors off. final {}'.format(pos.text()))


if __name__ == '__main__':
    main()
