"""
Session 2 · Script 6 — Four drones, one square  (FLIES — 4 drones, one radio)

The first flight with the whole starter fleet. Each drone takes off in turn and
slides to its own corner of a square centred in the box, halfway up the z
range; once all four are parked they hold for 2 s, then land one at a time with
the slow two-stage descent from s2_04.

All four drones share ONE Crazyradio here — that is the point of the exercise.
At 10 setpoints/s each that is 40 packets/s through a single dongle. If you see
drones stutter or drop, that is radio bandwidth, not the positioning system
(s4_03_swarm_scale_test.py explores exactly that limit).

Run:  python s2_06_square.py
Prereq: s2_04 flies cleanly. All four drones INSIDE the box, each standing
        near the corner printed at launch, NOSE ALONG +x.
Safety: area clear, glasses on, Ctrl-C cuts all four.
Note: only one drone ever moves at a time — the other three hold station. But
      all four sit at the SAME altitude here, so the square's side length is
      the only thing keeping them apart; the swarm sessions use distinct
      altitude lanes (config.HOMES) instead. Keep SIDE generous.
"""

import sys, os, time
from contextlib import ExitStack
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config

DRONE_NUMBERS = [0, 1, 2, 3]   # indices into config.URIS (0 = drone 1)

SIDE = 0.8                # m — length of the square's side
HOLD_S = 2.0              # s — how long all four hover together
MOVE_TIME = 3.0           # s to slide from the takeoff spot to a corner
CLIMB_RATE = 0.3          # m/s going up
SETPOINT_PERIOD = 0.1     # s between setpoints; the firmware wants < 0.5 s
YAW_DEG = 0.0             # nose along +x. send_position_setpoint takes DEGREES.

# Landing: same two-stage profile as s2_04/s2_05. The floor lies under the
# anchors' volume where z is least trustworthy, so creep past it rather than
# aiming at it and cutting the motors.
LAND_APPROACH_Z = 0.15
LAND_FLOOR_Z = -0.25
LAND_APPROACH_RATE = 0.15
LAND_TOUCH_RATE = 0.06
LAND_SETTLE_S = 0.7


def _send(cf, point):
    x, y, z = point
    cf.commander.send_position_setpoint(x, y, z, YAW_DEG)


class Fleet:
    """Every airborne drone's current target, plus the setpoint stream.

    A drone that stops receiving setpoints trips the firmware watchdog, so
    while one drone moves the others must still be fed their standing target
    every tick. Keeping all targets in one dict is what makes that automatic
    however many drones are up.
    """

    def __init__(self):
        self.targets = {}          # cf -> (x, y, z), airborne drones only

    def _tick(self):
        for cf, point in self.targets.items():
            _send(cf, point)

    def hold(self, seconds):
        """Everyone holds station."""
        t0 = time.time()
        while time.time() - t0 < seconds:
            self._tick()
            time.sleep(SETPOINT_PERIOD)

    def move(self, cf, start, end, duration):
        """Walk one drone's target from start to end; the rest hold station."""
        duration = max(duration, 1.0)
        self.targets[cf] = start
        t0 = time.time()
        while True:
            elapsed = time.time() - t0
            if elapsed >= duration:
                break
            f = elapsed / duration
            self.targets[cf] = tuple(a + (b - a) * f for a, b in zip(start, end))
            self._tick()
            time.sleep(SETPOINT_PERIOD)
        self.targets[cf] = end
        self._tick()

    def land(self, cf, label):
        """Two-stage descent for one drone, then cut its motors."""
        x, y, z = self.targets[cf]

        print('  {}: descending to {:.2f} m ...'.format(label, LAND_APPROACH_Z))
        self.move(cf, (x, y, z), (x, y, LAND_APPROACH_Z),
                  abs(z - LAND_APPROACH_Z) / LAND_APPROACH_RATE)
        self.hold(LAND_SETTLE_S)

        touch_s = abs(LAND_APPROACH_Z - LAND_FLOOR_Z) / LAND_TOUCH_RATE
        print('  {}: touching down over {:.1f} s ...'.format(label, touch_s))
        self.move(cf, (x, y, LAND_APPROACH_Z), (x, y, LAND_FLOOR_Z), touch_s)

        cf.commander.send_stop_setpoint()
        del self.targets[cf]       # on the ground: stop feeding it setpoints
        print('  {}: motors off.'.format(label))


def square_corners(side, z):
    """The four corners, counter-clockwise from the -x/-y one."""
    cx, cy = config.CENTER
    h = side / 2.0
    return [(cx - h, cy - h, z), (cx + h, cy - h, z),
            (cx + h, cy + h, z), (cx - h, cy + h, z)]


def corners_fit(corners):
    """True if every corner is inside config.BOX, with a printed verdict."""
    box = config.BOX
    ok = all(box['x_min'] <= x <= box['x_max'] and
             box['y_min'] <= y <= box['y_max'] and
             box['z_min'] <= z <= box['z_max'] for x, y, z in corners)
    print('  square spans x {:+.2f}..{:+.2f}  y {:+.2f}..{:+.2f}  at z={:.2f} — {}'
          .format(corners[0][0], corners[1][0], corners[0][1], corners[2][1],
                  corners[0][2], 'inside the box' if ok else 'OUTSIDE THE BOX'))
    return ok


def same_radio(uris):
    """True if every URI goes through one dongle — 'radio://N/...' with equal N."""
    radios = {uri.split('/')[2] for uri in uris}
    if len(radios) == 1:
        print('  all {} drones on radio {} — one dongle, as intended.'.format(
            len(uris), sorted(radios)[0]))
        return True
    print('  !! these URIs span radios {} — this script is the ONE-radio '
          'exercise. Use config.URIS (all radio 0), not config.URIS_DIST.'
          .format(sorted(radios)))
    return False


def main():
    if len(set(DRONE_NUMBERS)) != len(DRONE_NUMBERS):
        print('DRONE_NUMBERS repeats a drone — aborting.')
        return

    uris = [config.URIS[n] for n in DRONE_NUMBERS]
    labels = ['D{}'.format(n + 1) for n in DRONE_NUMBERS]

    square_z = (config.BOX['z_min'] + config.BOX['z_max']) / 2.0
    corners = square_corners(SIDE, square_z)

    print('Square of side {:.2f} m, centred in the box at z={:.2f}:'.format(
        SIDE, square_z))
    for label, uri, corner in zip(labels, uris, corners):
        print('  {}  {}  ->  x={:+.2f}  y={:+.2f}'.format(label, uri, *corner[:2]))
    if not (corners_fit(corners) and same_radio(uris)):
        print('  Fix the above before flying — aborting before takeoff.')
        return

    cf_utils.init()
    with ExitStack() as stack:
        scfs = [stack.enter_context(cf_utils.make_scf(uri)) for uri in uris]

        resting = []
        for scf, label in zip(scfs, labels):
            if not cf_utils.lps_deck_present(scf):
                print('  {}: no LPS deck detected — aborting for safety.'.format(label))
                return
            print('  {}: preparing ...'.format(label))
            cf_utils.prepare_for_flight(scf)
            at = cf_utils.get_position(scf)
            resting.append(at)
            print('  {}: resting at x={:+.2f}  y={:+.2f}  z={:+.2f}'.format(label, *at))

        print('  CHECK THE NOSES: all four drones must face the +x axis.')
        input('All estimates are good. Press ENTER to fly, Ctrl-C to abort... ')

        fleet = Fleet()
        cfs = [scf.cf for scf in scfs]
        try:
            # One at a time: climb straight up from where it stands, then slide
            # to its corner. The drones already parked hold station throughout.
            for cf, label, at, corner in zip(cfs, labels, resting, corners):
                over_start = (at[0], at[1], square_z)
                print('  {}: taking off to {:.2f} m ...'.format(label, square_z))
                fleet.move(cf, at, over_start, abs(square_z - at[2]) / CLIMB_RATE)
                print('  {}: moving to its corner ...'.format(label))
                fleet.move(cf, over_start, corner, MOVE_TIME)

            print('  square formed — holding {:.0f} s ...'.format(HOLD_S))
            fleet.hold(HOLD_S)

            for cf, label in zip(reversed(cfs), reversed(labels)):
                fleet.land(cf, label)      # last up, first down
            print('  all four landed.')
        except KeyboardInterrupt:
            for scf in scfs:
                cf_utils.emergency_stop(scf)
            print('\n  aborted — motors off on all four.')


if __name__ == '__main__':
    main()
