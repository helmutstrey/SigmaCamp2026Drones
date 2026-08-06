"""
Session 2 · Script 6 — Four drones, one square  (FLIES — 4 drones, 2 radios)

The first flight with the whole starter fleet. Each drone takes off in turn and
flies to its own corner of a square centred in the box, halfway up the z range;
once all four are parked they hold for 2 s, then land one at a time with the
slow two-stage descent from s2_04.

Run:  python s2_06_square.py
Prereq: s2_04 flies cleanly. TWO Crazyradio dongles plugged in (set RADIOS = 1
        if you only have one). All four drones INSIDE the box, each standing
        near the corner printed at launch, NOSE ALONG +x.
Safety: area clear, glasses on, Ctrl-C cuts all four.

Two dongles, and where the URIs come from:

  The drones' URIs are taken from config.URIS and only their DONGLE INDEX is
  rewritten — the first half of DRONE_NUMBERS flies on radio 0, the second
  half on radio 1. Channel, datarate and address are left exactly as config
  has them, because those are settings inside each drone's own firmware: a URI
  that disagrees with them will not find the drone at all.

  That also means both dongles here sit on channel 80. You gain a second USB
  pipe and a second set of link threads, which is the part that matters, but
  the two dongles still share the air. Truly separate channels would mean
  reconfiguring each drone in cfclient to match — a bench job, not something a
  URI can do.

Why the high-level commander here, when s2_05 streams setpoints:

  cflib gives each link an outgoing queue exactly ONE packet deep, and every
  link runs its own thread competing for its dongle. Streaming setpoints at
  10 Hz to four drones means 40 packets/s pushed into four depth-1 queues;
  when a link cannot drain in time, send_packet() blocks for two seconds and
  then kills the link with "could not send packet to copter" — a queue-full
  error, mid-flight. That is exactly what this script used to do.

  So: no streaming. takeoff()/go_to()/land() are one-shot commands, and the
  Crazyflie runs the trajectory onboard. A drone parked at its corner needs
  ZERO packets to stay there, and the whole flight costs a few dozen packets
  instead of thousands. This is why the swarm sessions use the high-level
  commander too. Streaming is for continuous motion with few drones (s2_05).

  Splitting across two dongles and using one-shot commands are independent
  wins, and both are in play here — which is why this has plenty of headroom.
  If you still see radio trouble, set RATE_LIMIT_HZ below: cflib accepts a
  '?rate_limit=N' URI query that throttles each link's thread.

Note: only one drone ever moves at a time. But all four sit at the SAME
      altitude, so the square's side length is the only thing keeping them
      apart; the swarm sessions use distinct altitude lanes (config.HOMES)
      instead. Keep SIDE generous.
"""

import sys, os, time
from contextlib import ExitStack
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cf_utils
import config

DRONE_NUMBERS = [0, 1, 2, 3]   # indices into config.URIS (0 = drone 1). Any
                               # four will do — the list order decides both the
                               # corner each drone flies to and its dongle.
RADIOS = 2                     # Crazyradio dongles to spread the fleet over.
                               # The list is dealt out in contiguous halves:
                               # the first two drones on radio 0, the last two
                               # on radio 1. Set to 1 for the single-dongle
                               # version of this exercise.

SIDE = 0.8                # m — length of the square's side
HOLD_S = 2.0              # s — how long all four hover together
MOVE_TIME = 3.0           # s to fly from above the takeoff spot to a corner
CLIMB_RATE = 0.3          # m/s going up
YAW_RAD = 0.0             # nose along +x. go_to takes RADIANS (unlike the
                          # low-level position setpoint, which takes degrees).
RATE_LIMIT_HZ = None      # e.g. 100 to throttle each link; None leaves the URI
                          # alone. Only needed if the shared dongle struggles.

# Landing: same two-stage profile as s2_04/s2_05. The floor lies under the
# anchors' volume where z is least trustworthy, so creep past it rather than
# aiming at it and cutting the motors.
LAND_APPROACH_Z = 0.15
LAND_FLOOR_Z = -0.25
LAND_APPROACH_RATE = 0.15
LAND_TOUCH_RATE = 0.06
LAND_SETTLE_S = 0.7


def with_rate_limit(uri):
    """Append cflib's '?rate_limit=N' query, if RATE_LIMIT_HZ is set."""
    if not RATE_LIMIT_HZ:
        return uri
    joiner = '&' if '?' in uri else '?'
    return '{}{}rate_limit={}'.format(uri, joiner, RATE_LIMIT_HZ)


def _descend(hlc, from_z, to_z, rate):
    """One vertical leg at `rate` m/s. Never quicker than a second."""
    duration = max(abs(from_z - to_z) / rate, 1.0)
    hlc.land(to_z, duration, yaw=None)     # land() = descend from current x-y
    time.sleep(duration)
    return duration


def land_drone(scf, label, from_z):
    """Two-stage descent: down to a low hover, settle, then creep onto the floor."""
    hlc = scf.cf.high_level_commander

    print('  {}: descending to {:.2f} m ...'.format(label, LAND_APPROACH_Z))
    _descend(hlc, from_z, LAND_APPROACH_Z, LAND_APPROACH_RATE)
    time.sleep(LAND_SETTLE_S)

    secs = _descend(hlc, LAND_APPROACH_Z, LAND_FLOOR_Z, LAND_TOUCH_RATE)
    print('  {}: touching down over {:.1f} s ...'.format(label, secs))
    hlc.stop()                             # motors off, drone already resting
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


def radios_ready(uris, labels):
    """Print the dongle assignment, and check that many dongles are plugged in."""
    groups = {}
    for uri, label in zip(uris, labels):
        groups.setdefault(uri.split('/')[2], []).append(label)
    for devid in sorted(groups):
        print('  radio {}: {}'.format(devid, ', '.join(groups[devid])))

    wanted = len(groups)
    found = cf_utils.count_radios()
    if found is None:
        print('  (could not count dongles — make sure {} are plugged in)'.format(wanted))
        return True
    if found < wanted:
        print('  !! this needs {} Crazyradio dongles but only {} {} plugged in.'
              .format(wanted, found, 'is' if found == 1 else 'are'))
        return False
    print('  {} dongles plugged in, {} needed.'.format(found, wanted))
    return True


def main():
    if len(set(DRONE_NUMBERS)) != len(DRONE_NUMBERS):
        print('DRONE_NUMBERS repeats a drone — aborting.')
        return

    # Take the drones' own URIs from config and only swap the dongle index:
    # the first half of the list flies on radio 0, the second half on radio 1.
    uris = cf_utils.split_across_radios(
        [config.URIS[n] for n in DRONE_NUMBERS], RADIOS)
    labels = ['D{}'.format(n + 1) for n in DRONE_NUMBERS]

    square_z = (config.BOX['z_min'] + config.BOX['z_max']) / 2.0
    corners = square_corners(SIDE, square_z)

    print('Square of side {:.2f} m, centred in the box at z={:.2f}:'.format(
        SIDE, square_z))
    for label, uri, corner in zip(labels, uris, corners):
        print('  {}  {}  ->  x={:+.2f}  y={:+.2f}'.format(label, uri, *corner[:2]))
    if not (corners_fit(corners) and radios_ready(uris, labels)):
        print('  Fix the above before flying — aborting before takeoff.')
        return

    cf_utils.init()
    with ExitStack() as stack:
        scfs = [stack.enter_context(cf_utils.make_scf(with_rate_limit(uri)))
                for uri in uris]

        for scf, label in zip(scfs, labels):
            if not cf_utils.lps_deck_present(scf):
                print('  {}: no LPS deck detected — aborting for safety.'.format(label))
                return
            print('  {}: preparing ...'.format(label))
            cf_utils.prepare_for_flight(scf)
            at = cf_utils.get_position(scf)
            print('  {}: resting at x={:+.2f}  y={:+.2f}  z={:+.2f}'.format(label, *at))

        print('  CHECK THE NOSES: all four drones must face the +x axis.')
        input('All estimates are good. Press ENTER to fly, Ctrl-C to abort... ')

        climb_s = max(square_z / CLIMB_RATE, 1.0)
        try:
            # One at a time. takeoff() climbs from the drone's current x-y and
            # holds there; go_to() then flies it to its corner. Drones already
            # parked need no further commands — the onboard commander keeps
            # them in place, which is what keeps the radio quiet.
            for scf, label, corner in zip(scfs, labels, corners):
                hlc = scf.cf.high_level_commander

                print('  {}: taking off to {:.2f} m ...'.format(label, square_z))
                hlc.takeoff(square_z, climb_s, yaw=None)    # yaw=None: keep heading
                time.sleep(climb_s)

                print('  {}: moving to x={:+.2f}  y={:+.2f} ...'.format(
                    label, *corner[:2]))
                hlc.go_to(corner[0], corner[1], corner[2], YAW_RAD, MOVE_TIME)
                time.sleep(MOVE_TIME)

            print('  square formed — holding {:.0f} s ...'.format(HOLD_S))
            time.sleep(HOLD_S)

            for scf, label in zip(reversed(scfs), reversed(labels)):
                land_drone(scf, label, square_z)      # last up, first down
            print('  all four landed.')
        except KeyboardInterrupt:
            for scf in scfs:
                cf_utils.emergency_stop(scf)
            print('\n  aborted — motors off on all four.')


if __name__ == '__main__':
    main()
