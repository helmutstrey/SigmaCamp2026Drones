"""
Session 2 · Script 4 — Two drones, one after the other  (FLIES)

The first two-aircraft flight. Drone A takes off, slides to its slot and
hovers at 0.4 m; only then does drone B do the same. Both hold for 3 s, then
they land one at a time, last up first down. Sequential on purpose: one drone
in motion at a time is one thing to watch.

The landing is two-stage and slow — see the LAND_* constants — because the
floor lies below the anchors' volume, where the z estimate is at its worst.

Pick the two aircraft with DRONE_A_NUMBER / DRONE_B_NUMBER below — they are
indices into config.URIS, so 0 is drone 1.

Run:  python s2_04_two_drone_hover.py
Prereq: preflight (s2_00) passes for BOTH drones, and both start INSIDE the
        box, near the slots this script prints at startup.
Safety: area clear, glasses on, Ctrl-C cuts both motors.

Two warnings worth reading before you fly this:

  * NOSE ALONG +x. Neither drone has a compass indoors — the estimator assumes
    each one faces the +x axis when its estimate is reset. Point them both
    that way or they will fly the wrong direction. (cf_utils.prepare_for_flight
    takes initial_yaw_deg if you need another heading; this script leaves it at
    the default so it also runs on an older checkout of cf_utils.)
  * SPACING is 0.3 m at the SAME altitude, which is tighter than this camp's
    usual separation — the rest of the material keeps drones apart with
    distinct altitude lanes (config.HOMES) because they cannot sense each
    other. 0.3 m is roughly three airframes' width, and it is within the LPS
    position error, so expect them to look close. Fly it low the first time.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config

DRONE_A_NUMBER = 8        # index into config.URIS (0 = drone 1)
DRONE_B_NUMBER = 9

HOVER_HEIGHT = 0.4        # m above the LPS floor, same for both
SPACING = 0.3             # m between the two hover slots
HOLD_S = 3.0              # how long both hover together
MOVE_TIME = 2.0           # s to slide from the takeoff spot to the slot

# Landing. The floor sits UNDER the volume the anchors enclose, and z is the
# least trustworthy axis down there, so "descend to z = 0 and cut" can end the
# trajectory while the drone is still in the air — that is the drop you feel.
# Instead: come down to a low hover, settle, then creep well past the floor so
# the drone is already resting on it before the motors stop. Pushing into the
# ground for a second is gentler than a 15 cm fall.
LAND_APPROACH_Z = 0.15    # m — low hover to pause at on the way down
LAND_FLOOR_Z = -0.25      # m — final target, deliberately below the floor
LAND_APPROACH_RATE = 0.15 # m/s down to the low hover
LAND_TOUCH_RATE = 0.06    # m/s for the last stretch — this is the gentle one
LAND_SETTLE_S = 0.7       # s to steady at the low hover before touching down


def hover_at(scf, target, label):
    """Take off and slide to an absolute slot. Blocks until it is parked."""
    x, y, z = target
    hlc = scf.cf.high_level_commander

    print('  {}: taking off to {:.2f} m ...'.format(label, z))
    hlc.takeoff(z, config.TAKEOFF_TIME, yaw=None)     # yaw=None: keep heading
    time.sleep(config.TAKEOFF_TIME)

    print('  {}: moving to x={:+.2f}  y={:+.2f} ...'.format(label, x, y))
    hlc.go_to(x, y, z, 0.0, MOVE_TIME)                # go_to yaw is RADIANS; 0 = +x
    time.sleep(MOVE_TIME)


def _descend(hlc, from_z, to_z, rate):
    """One vertical leg at `rate` m/s. Never quicker than a second."""
    duration = max(abs(from_z - to_z) / rate, 1.0)
    hlc.land(to_z, duration, yaw=None)     # land() = descend from current x-y
    time.sleep(duration)
    return duration


def land_drone(scf, label, from_z=HOVER_HEIGHT):
    """Two-stage descent: down to a low hover, settle, then creep onto the floor."""
    hlc = scf.cf.high_level_commander

    print('  {}: descending to {:.2f} m ...'.format(label, LAND_APPROACH_Z))
    _descend(hlc, from_z, LAND_APPROACH_Z, LAND_APPROACH_RATE)
    time.sleep(LAND_SETTLE_S)

    secs = _descend(hlc, LAND_APPROACH_Z, LAND_FLOOR_Z, LAND_TOUCH_RATE)
    print('  {}: touching down over {:.1f} s ...'.format(label, secs))
    hlc.stop()                             # motors off, drone already resting


def main():
    if DRONE_A_NUMBER == DRONE_B_NUMBER:
        print('DRONE_A_NUMBER and DRONE_B_NUMBER are the same drone — aborting.')
        return

    uri_a = config.URIS[DRONE_A_NUMBER]
    uri_b = config.URIS[DRONE_B_NUMBER]

    # Two slots straddling the middle of the box, SPACING apart along y.
    # Swap the offsets onto cx if you would rather separate them along x.
    cx, cy = config.CENTER
    target_a = cf_utils.safe_xyz(cx, cy - SPACING / 2.0, HOVER_HEIGHT)
    target_b = cf_utils.safe_xyz(cx, cy + SPACING / 2.0, HOVER_HEIGHT)

    print('Hover slots ({:.2f} m apart) — place each drone near its own, '
          'nose along +x:'.format(SPACING))
    print('  A  {}  ->  x={:+.2f}  y={:+.2f}  z={:.2f}'.format(uri_a, *target_a))
    print('  B  {}  ->  x={:+.2f}  y={:+.2f}  z={:.2f}'.format(uri_b, *target_b))

    cf_utils.init()
    with cf_utils.make_scf(uri_a) as scf_a, cf_utils.make_scf(uri_b) as scf_b:
        # Not a Swarm: every step here is deliberately one drone at a time, and
        # Swarm.parallel_safe exists to do the opposite.
        for scf, label in ((scf_a, 'A'), (scf_b, 'B')):
            if not cf_utils.lps_deck_present(scf):
                print('  {}: no LPS deck detected — aborting for safety.'.format(label))
                return
            print('  {}: preparing ...'.format(label))
            cf_utils.prepare_for_flight(scf)

        input('Both estimates are good. Press ENTER to fly, Ctrl-C to abort... ')

        try:
            hover_at(scf_a, target_a, 'A')
            hover_at(scf_b, target_b, 'B')

            print('  both hovering at {:.2f} m for {:.0f} s ...'.format(
                HOVER_HEIGHT, HOLD_S))
            time.sleep(HOLD_S)

            land_drone(scf_b, 'B')        # last up, first down
            land_drone(scf_a, 'A')
            print('  both landed.')
        except KeyboardInterrupt:
            # The high-level commander keeps flying its last trajectory even
            # once we stop talking to it, so Ctrl-C has to cut both itself.
            cf_utils.emergency_stop(scf_a)
            cf_utils.emergency_stop(scf_b)
            print('\n  aborted — motors off on both.')


if __name__ == '__main__':
    main()
