"""
Session 2 · Script 5 — Two drones orbiting each other  (FLIES)

Picks up where s2_04_two_drone_hover.py stops. A takes off, then B; both slide
onto opposite ends of a 0.3 m line and then walk that line around its own
middle — a full turn, each drone tracing a half-circle of radius 0.15 m while
staying exactly 0.3 m from the other. Then they land one at a time, slowly.

Run:  python s2_05_two_drone_orbit.py
Prereq: s2_04 flies cleanly first. Both drones INSIDE the box, near the start
        points printed at launch, NOSE ALONG +x.
Safety: area clear, glasses on, Ctrl-C cuts both motors.

How the orbit is flown, and why not with go_to():

  The high-level commander's go_to() warns against overlapping commands — a
  new one arriving mid-trajectory can produce a wild polynomial — and it
  decelerates to a stop at every waypoint, so a circle chopped into go_to
  steps comes out as a stutter. Instead this script streams
  send_position_setpoint() to BOTH drones every 0.1 s, walking each target
  around the circle. The separation is exact by construction: the two targets
  are always antipodal, so it is 2 x radius at every instant, not something
  the controllers have to negotiate.

  Note the units trap: send_position_setpoint() takes yaw in DEGREES, while
  the high-level commander's go_to()/takeoff() take radians. Both drones hold
  yaw 0 (nose along +x) for the whole flight — they translate around the
  circle, they do not pirouette.

  SPACING is 0.3 m at the same altitude, tighter than this camp's usual
  altitude-lane separation, and now the drones are moving relative to each
  other. Fly it once at low height with plenty of room before trusting it.
"""

import sys, os, math, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config

DRONE_A_NUMBER = 8        # index into config.URIS (0 = drone 1)
DRONE_B_NUMBER = 9

HOVER_HEIGHT = 0.4        # m, same for both — they never share a spot in x/y
SPACING = 0.3             # m between the drones, held for the whole orbit
ORBIT_TURNS = 1.0         # full turns around the shared centre
ORBIT_TIME = 20.0         # s for those turns — slow is the point
MOVE_TIME = 3.0           # s to slide from the takeoff spot onto the circle
CLIMB_RATE = 0.3          # m/s going up
SETPOINT_PERIOD = 0.1     # s between setpoints; the firmware wants < 0.5 s
YAW_DEG = 0.0             # nose along +x. DEGREES here — see the docstring.

# Landing, same two-stage profile as s2_04: the floor sits under the anchors'
# volume where z is least trustworthy, so creep past it rather than aiming at
# it and cutting.
LAND_APPROACH_Z = 0.15    # m — low hover to pause at on the way down
LAND_FLOOR_Z = -0.25      # m — final target, deliberately below the floor
LAND_APPROACH_RATE = 0.15 # m/s down to the low hover
LAND_TOUCH_RATE = 0.06    # m/s for the last stretch — this is the gentle one
LAND_SETTLE_S = 0.7       # s to steady at the low hover before touching down


def _send(cf, point):
    x, y, z = point
    cf.commander.send_position_setpoint(x, y, z, YAW_DEG)


def _on_circle(centre, radius, angle_rad, z):
    cx, cy = centre
    return (cx + radius * math.cos(angle_rad),
            cy + radius * math.sin(angle_rad),
            z)


def fly_legs(legs, duration):
    """Move every drone in a straight line from its start point to its end.

    legs is [(cf, start_xyz, end_xyz), ...]. Drones not moving still belong
    here with start == end: a drone that stops receiving setpoints hits the
    firmware's watchdog, so the one holding station has to be fed too.
    """
    duration = max(duration, 1.0)
    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        if elapsed >= duration:
            break
        f = elapsed / duration
        for cf, p0, p1 in legs:
            _send(cf, tuple(a + (b - a) * f for a, b in zip(p0, p1)))
        time.sleep(SETPOINT_PERIOD)
    for cf, _p0, p1 in legs:
        _send(cf, p1)


def orbit(cf_a, cf_b, centre, radius, z, duration, turns):
    """Walk both targets around `centre`, half a turn apart, for `turns` turns."""
    t0 = time.time()
    while True:
        elapsed = time.time() - t0
        if elapsed >= duration:
            break
        angle = 2.0 * math.pi * turns * (elapsed / duration)
        _send(cf_a, _on_circle(centre, radius, angle, z))
        _send(cf_b, _on_circle(centre, radius, angle + math.pi, z))
        time.sleep(SETPOINT_PERIOD)


def land_drone(cf, at_xy, holders, label):
    """Two-stage descent for one drone while `holders` keep their station.

    holders is [(cf, point), ...] — every other airborne drone, which must
    keep receiving setpoints throughout.
    """
    x, y = at_xy
    held = [(hcf, p, p) for hcf, p in holders]

    print('  {}: descending to {:.2f} m ...'.format(label, LAND_APPROACH_Z))
    fly_legs([(cf, (x, y, HOVER_HEIGHT), (x, y, LAND_APPROACH_Z))] + held,
             abs(HOVER_HEIGHT - LAND_APPROACH_Z) / LAND_APPROACH_RATE)
    fly_legs([(cf, (x, y, LAND_APPROACH_Z), (x, y, LAND_APPROACH_Z))] + held,
             LAND_SETTLE_S)

    touch_s = abs(LAND_APPROACH_Z - LAND_FLOOR_Z) / LAND_TOUCH_RATE
    print('  {}: touching down over {:.1f} s ...'.format(label, touch_s))
    fly_legs([(cf, (x, y, LAND_APPROACH_Z), (x, y, LAND_FLOOR_Z))] + held,
             touch_s)

    cf.commander.send_stop_setpoint()      # motors off, drone already resting
    print('  {}: motors off.'.format(label))


def circle_fits(centre, radius):
    """True if the whole orbit stays inside config.BOX, with a printed verdict."""
    cx, cy = centre
    box = config.BOX
    ok = (box['x_min'] <= cx - radius and cx + radius <= box['x_max'] and
          box['y_min'] <= cy - radius and cy + radius <= box['y_max'])
    print('  orbit spans x {:+.2f}..{:+.2f}  y {:+.2f}..{:+.2f}  — {}'.format(
        cx - radius, cx + radius, cy - radius, cy + radius,
        'inside the box' if ok else 'OUTSIDE THE BOX'))
    return ok


def main():
    if DRONE_A_NUMBER == DRONE_B_NUMBER:
        print('DRONE_A_NUMBER and DRONE_B_NUMBER are the same drone — aborting.')
        return

    uri_a = config.URIS[DRONE_A_NUMBER]
    uri_b = config.URIS[DRONE_B_NUMBER]

    centre = config.CENTER
    radius = SPACING / 2.0
    start_a = _on_circle(centre, radius, 0.0, HOVER_HEIGHT)
    start_b = _on_circle(centre, radius, math.pi, HOVER_HEIGHT)

    print('Orbit centre x={:+.2f} y={:+.2f}, radius {:.2f} m, '
          'drones {:.2f} m apart:'.format(centre[0], centre[1], radius, SPACING))
    print('  A  {}  starts at x={:+.2f}  y={:+.2f}'.format(uri_a, *start_a[:2]))
    print('  B  {}  starts at x={:+.2f}  y={:+.2f}'.format(uri_b, *start_b[:2]))
    if not circle_fits(centre, radius):
        print('  Move the orbit or shrink SPACING — aborting before takeoff.')
        return

    cf_utils.init()
    with cf_utils.make_scf(uri_a) as scf_a, cf_utils.make_scf(uri_b) as scf_b:
        resting = {}
        for scf, label in ((scf_a, 'A'), (scf_b, 'B')):
            if not cf_utils.lps_deck_present(scf):
                print('  {}: no LPS deck detected — aborting for safety.'.format(label))
                return
            print('  {}: preparing ...'.format(label))
            cf_utils.prepare_for_flight(scf)
            resting[label] = cf_utils.get_position(scf)
            print('  {}: resting at x={:+.2f}  y={:+.2f}  z={:+.2f}'.format(
                label, *resting[label]))

        print('  CHECK THE NOSES: both drones must face the +x axis.')
        input('Both estimates are good. Press ENTER to fly, Ctrl-C to abort... ')

        cf_a, cf_b = scf_a.cf, scf_b.cf
        # Take off from where each drone actually stands, not from an assumed
        # (x, y, 0) — that is what keeps the climb vertical.
        rest_a, rest_b = resting['A'], resting['B']
        up_a = (rest_a[0], rest_a[1], HOVER_HEIGHT)
        up_b = (rest_b[0], rest_b[1], HOVER_HEIGHT)

        try:
            print('  A: taking off to {:.2f} m ...'.format(HOVER_HEIGHT))
            fly_legs([(cf_a, rest_a, up_a)],
                     abs(HOVER_HEIGHT - rest_a[2]) / CLIMB_RATE)

            print('  B: taking off to {:.2f} m ...'.format(HOVER_HEIGHT))
            fly_legs([(cf_a, up_a, up_a),                    # A holds station
                      (cf_b, rest_b, up_b)],
                     abs(HOVER_HEIGHT - rest_b[2]) / CLIMB_RATE)

            print('  both sliding onto the circle ...')
            fly_legs([(cf_a, up_a, start_a), (cf_b, up_b, start_b)], MOVE_TIME)

            print('  orbiting {:.0f} deg over {:.0f} s ...'.format(
                360.0 * ORBIT_TURNS, ORBIT_TIME))
            orbit(cf_a, cf_b, centre, radius, HOVER_HEIGHT, ORBIT_TIME, ORBIT_TURNS)

            # A full turn ends where it began, so the start points double as
            # the landing spots.
            land_drone(cf_b, start_b[:2], [(cf_a, start_a)], 'B')
            land_drone(cf_a, start_a[:2], [], 'A')
            print('  both landed.')
        except KeyboardInterrupt:
            cf_utils.emergency_stop(scf_a)
            cf_utils.emergency_stop(scf_b)
            print('\n  aborted — motors off on both.')


if __name__ == '__main__':
    main()
