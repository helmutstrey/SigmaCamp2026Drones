"""
cf_utils.py  —  shared helpers used by every session script.

These wrap the boring-but-critical parts of flying a Crazyflie with the Loco
Positioning System in **TDoA2** mode: putting the deck in the right mode,
confirming all 8 anchors are heard, waiting for a valid position before
take-off, enabling the high-level commander, and keeping targets inside the
flight box. Read this file once — the rest of the scripts stay short because
the care lives here.
"""

import math
import threading
import time

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.crazyflie.mem import MemoryElement

import config


# --------------------------------------------------------------------------
# Connection
# --------------------------------------------------------------------------
def init():
    """Initialise the radio drivers. Call once at the top of every script."""
    cflib.crtp.init_drivers()


def make_scf(uri):
    """A SyncCrazyflie with a TOC cache (faster reconnects)."""
    return SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache'))


def on_radio(uri, devid):
    """Rewrite a URI to go through Crazyradio dongle number `devid`.

    ONLY the dongle index changes. Channel, datarate and address stay as they
    are: those are the drone's own firmware settings, and a URI that disagrees
    with them simply will not find the drone.
    """
    scheme, _, rest = uri.partition('://')
    parts = rest.split('/')
    parts[0] = str(devid)
    return '{}://{}'.format(scheme, '/'.join(parts))


def split_across_radios(uris, radios=2):
    """Deal a list of URIs into contiguous groups, one dongle each.

    With four drones and two dongles the first two go to radio 0 and the last
    two to radio 1. Splitting the fleet gives each dongle its own USB pipe and
    its own set of link threads, which is what relieves the one-packet-deep
    outgoing queue cflib gives every link.

    Note what this does NOT fix: dongles on the same channel still share the
    air. Separate channels need each drone reconfigured to match, so that is a
    cfclient job, not something a URI can do.
    """
    if radios < 1:
        raise ValueError('radios must be at least 1')
    return [on_radio(uri, (i * radios) // len(uris))
            for i, uri in enumerate(uris)]


def count_radios():
    """How many Crazyradio dongles are plugged in (None if it cannot tell)."""
    try:
        from cflib.drivers import crazyradio
        return len(crazyradio.get_serials())
    except Exception:
        return None


# --------------------------------------------------------------------------
# Loco Positioning mode (TDoA2)
# --------------------------------------------------------------------------
# Values of the 'loco.mode' parameter on the LPS deck.
LPS_MODE_AUTO = 0
LPS_MODE_TWR = 1
LPS_MODE_TDOA2 = 2
LPS_MODE_TDOA3 = 3
LPS_MODE_NAMES = {-1: 'unknown', 0: 'auto', 1: 'TWR', 2: 'TDoA2', 3: 'TDoA3'}

# The deck tears down and restarts its ranging algorithm on a mode change, so
# give it a moment before asking the estimator to converge.
MODE_SETTLE_S = 2.0


def get_lps_mode(scf):
    """Current 'loco.mode' as an int, or -1 if the param isn't readable."""
    try:
        return int(scf.cf.param.get_value('loco.mode'))
    except Exception:
        return -1


def lps_mode_name(mode):
    return LPS_MODE_NAMES.get(mode, 'mode {}'.format(mode))


def use_tdoa2_mode(scf, settle_s=MODE_SETTLE_S):
    """Pin this drone's LPS deck to TDoA2 (config.LPS_MODE).

    Leaving the deck on 'auto' lets it guess the mode from the traffic it
    hears, which is fine until one drone guesses differently from the rest.
    Setting it explicitly costs nothing and removes a whole class of
    'why is only this one drone drifting?' mysteries.

    Returns True if the deck is in the wanted mode afterwards.
    """
    wanted = config.LPS_MODE
    current = get_lps_mode(scf)
    if current == wanted:
        return True
    if current == -1:
        print('  (no loco.mode param — is the LPS deck attached? '
              'skipping mode check)')
        return False
    try:
        scf.cf.param.set_value('loco.mode', str(wanted))
        time.sleep(settle_s)          # deck restarts ranging; let it re-lock
        print('  loco.mode: {} -> {}'.format(
            lps_mode_name(current), lps_mode_name(wanted)))
        return True
    except Exception as e:
        print('  (could not set loco.mode to {}: {})'.format(
            lps_mode_name(wanted), e))
        return False


def anchor_status(scf, timeout_s=5.0):
    """Ask the deck which anchors it knows about and which it currently hears.

    Returns (anchor_ids, active_ids, positions) where positions maps
    anchor id -> ((x, y, z), is_valid). In TDoA2 you want all 8 of IDs 0-7 in
    both lists: the schedule is fixed, so a silent anchor is a hole in it, not
    something the system routes around.

    Returns (None, None, None) if the deck doesn't expose the anchor memory.
    """
    mems = scf.cf.mem.get_mems(MemoryElement.TYPE_LOCO2)
    if not mems:
        return (None, None, None)
    mem = mems[0]

    def _sync(request):
        done = threading.Event()
        request(lambda _mem: done.set())
        if not done.wait(timeout_s):
            raise TimeoutError('Timed out reading anchor memory from the deck.')

    _sync(mem.update_id_list)
    _sync(mem.update_active_id_list)
    positions = {}
    if mem.nr_of_anchors > 0:
        _sync(mem.update_data)
        positions = {i: (a.position, a.is_valid)
                     for i, a in mem.anchor_data.items()}
    return (list(mem.anchor_ids), list(mem.active_anchor_ids), positions)


def check_anchors(scf, expected=None, verbose=True, status=None, require_all=False):
    """Report which anchors the deck currently hears. Warns, does not veto.

    Hearing a subset is normal on the ground: a drone parked outside the
    anchor volume, or shadowed by a body or a table leg, picks up only some of
    them and fills the list in once it is inside the box. So a missing anchor
    prints a warning and this still returns True.

    Pass require_all=True to get the strict behaviour back — then a missing
    anchor returns False. Anchor 0 is called out either way: it is the TDoA2
    master, and without it nothing ranges at all. Pass `status` from a previous
    anchor_status() call to avoid a second round-trip to the deck.
    """
    expected = list(config.ANCHOR_IDS if expected is None else expected)
    ids, active, _positions = status if status else anchor_status(scf)
    if active is None:
        if verbose:
            print('  (deck does not expose the anchor list — skipping check)')
        return True

    missing = [i for i in expected if i not in active]
    extra = [i for i in active if i not in expected]

    if verbose:
        print('  anchors heard: {}/{}  {}'.format(
            len(active), len(expected),
            sorted(active) if active else '(none)'))
    if not missing:
        if extra and verbose:
            print('  !! anchors {} are outside the TDoA2 range 0-7 — TDoA2 '
                  'ignores them. Renumber or power them down.'.format(sorted(extra)))
        return True

    if verbose:
        print('  !! anchors not heard yet: {}'.format(missing))
        if 0 in missing:
            print('     anchor 0 is the TDoA2 MASTER — nothing ranges without '
                  'it. Check its power first.')
        print('     Usually fine if the drone is still outside the box — it '
              'should hear the rest once it is inside. If the list does not '
              'fill in, check power and that each anchor can see the others '
              '(TDoA2 needs one shared schedule, not just line-of-sight to '
              'the drone).')
    return not require_all


def set_anchor_modes_tdoa2(scf, anchor_ids=None, repeats=5):
    """Broadcast a 'switch to TDoA2' command to the anchors themselves.

    The drone is only the messenger here — the anchors receive the LPP packet,
    change mode and reset. Anchors are addressed from the highest ID down so
    the master (0) switches LAST, and the sequence is repeated because a
    resetting anchor can miss a packet. Same order cfclient uses.

    Requires the drone to be talking to the anchors already (any LPS mode).
    """
    from lpslib.lopoanchor import LoPoAnchor

    anchor_ids = list(config.ANCHOR_IDS if anchor_ids is None else anchor_ids)
    lopo = LoPoAnchor(scf.cf)
    for _ in range(repeats):
        for anchor_id in sorted(anchor_ids, reverse=True):
            lopo.set_mode(anchor_id, LoPoAnchor.MODE_TDOA)   # MODE_TDOA = TDoA2
            time.sleep(0.05)


# --------------------------------------------------------------------------
# Firmware setup for autonomous flight
# --------------------------------------------------------------------------
def use_kalman_estimator(scf):
    """Force the Kalman (EKF) estimator. Modern firmware auto-selects this when
    an LPS deck is present, but setting it is harmless and explicit."""
    try:
        scf.cf.param.set_value('stabilizer.estimator', '2')
    except Exception:
        pass  # older/newer param name differences shouldn't stop a flight


def activate_high_level_commander(scf):
    """Enable the onboard high-level commander (needed for takeoff/go_to/land)."""
    scf.cf.param.set_value('commander.enHighLevel', '1')


def set_initial_yaw(scf, yaw_deg):
    """Tell the estimator which way the drone is FACING, in degrees, 0 = +x.

    Indoors the Kalman filter has no absolute heading reference. On a reset it
    simply assumes the drone points along 'kalman.initialYaw' (0 by default,
    i.e. the LPS +x axis) and leaks the yaw estimate back to that angle while
    the drone sits on the ground. Get it wrong and every position correction
    comes out rotated by the error: 90 degrees off makes the drone orbit its
    target, 180 degrees off makes it push exactly the wrong way and accelerate
    away. This is the usual explanation for 'the position looks right but it
    flew into a wall'.

    Must be set BEFORE reset_estimator(). Returns True if the parameter
    existed and was set; older firmware without it needs the drone physically
    pointed along +x instead.
    """
    try:
        scf.cf.param.set_value('kalman.initialYaw', str(math.radians(yaw_deg)))
        return True
    except Exception as e:
        print('  (could not set kalman.initialYaw: {} — point the drone along '
              'the +x axis before takeoff instead)'.format(e))
        return False


def prepare_for_flight(scf, initial_yaw_deg=None):
    """Everything a drone needs before it lifts off: LPS deck in TDoA2, Kalman
    estimator on, high-level commander on, and a fresh, converged position
    estimate.

    Order matters — the mode change restarts ranging, so it has to happen
    before we ask the estimator to converge, and the initial yaw has to be set
    before the reset that consumes it.

    Pass initial_yaw_deg if the drone does NOT start facing the +x axis.
    """
    use_tdoa2_mode(scf)
    use_kalman_estimator(scf)
    activate_high_level_commander(scf)
    if initial_yaw_deg is not None:
        set_initial_yaw(scf, initial_yaw_deg)
    reset_estimator(scf)


# --------------------------------------------------------------------------
# Position estimator
# --------------------------------------------------------------------------
def reset_estimator(scf):
    """Reset the Kalman filter, then block until the position estimate has
    converged (low variance). Skipping the wait is the #1 cause of a lurch at
    take-off."""
    cf = scf.cf
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    wait_for_position_estimator(scf)


def wait_for_position_estimator(scf, timeout_s=20.0):
    """Block until the estimate variance is small in x, y and z."""
    log = LogConfig(name='Kalman Variance', period_in_ms=100)
    log.add_variable('kalman.varPX', 'float')
    log.add_variable('kalman.varPY', 'float')
    log.add_variable('kalman.varPZ', 'float')

    var_x, var_y, var_z = [1000.0] * 10, [1000.0] * 10, [1000.0] * 10
    threshold = 0.001
    t0 = time.time()

    with SyncLogger(scf, log) as logger:
        for _, data, _ in logger:
            var_x.append(data['kalman.varPX']); var_x.pop(0)
            var_y.append(data['kalman.varPY']); var_y.pop(0)
            var_z.append(data['kalman.varPZ']); var_z.pop(0)
            if (max(var_x) - min(var_x) < threshold and
                    max(var_y) - min(var_y) < threshold and
                    max(var_z) - min(var_z) < threshold):
                return True
            if time.time() - t0 > timeout_s:
                raise TimeoutError(
                    'Position estimator did not converge. Check that the LPS '
                    'deck is attached, that all 8 anchors (IDs 0-7, including '
                    'the master, 0) are powered and in TDoA2, and that the '
                    'drone is INSIDE the anchor box. Run '
                    'session2/s2_00_preflight_lps.py to see which check fails.')


def get_position(scf, timeout_s=5.0):
    """Return the current estimated (x, y, z) as a tuple of metres."""
    log = LogConfig(name='pos', period_in_ms=100)
    log.add_variable('stateEstimate.x', 'float')
    log.add_variable('stateEstimate.y', 'float')
    log.add_variable('stateEstimate.z', 'float')
    with SyncLogger(scf, log) as logger:
        for _, data, _ in logger:
            return (data['stateEstimate.x'],
                    data['stateEstimate.y'],
                    data['stateEstimate.z'])
    raise TimeoutError('No position received.')


def stream_position(scf, duration_s=15.0, period_ms=200):
    """Print the live estimated position — handy for the 'move it by hand and
    watch it track' demo in Session 2."""
    log = LogConfig(name='pos', period_in_ms=period_ms)
    log.add_variable('stateEstimate.x', 'float')
    log.add_variable('stateEstimate.y', 'float')
    log.add_variable('stateEstimate.z', 'float')
    t0 = time.time()
    with SyncLogger(scf, log) as logger:
        for _, data, _ in logger:
            print('  x={:+.2f}  y={:+.2f}  z={:+.2f}'.format(
                data['stateEstimate.x'],
                data['stateEstimate.y'],
                data['stateEstimate.z']))
            if time.time() - t0 > duration_s:
                return


# --------------------------------------------------------------------------
# Health checks
# --------------------------------------------------------------------------
def lps_deck_present(scf):
    """True if the Loco Positioning deck is detected. Falls back to True with a
    warning if this firmware/cflib doesn't expose the param."""
    try:
        val = scf.cf.param.get_value('deck.bcDWM1000')
        return str(val) == '1'
    except Exception as e:
        print('  (could not read deck.bcDWM1000: {} — skipping deck check)'.format(e))
        return True


def battery_voltage(scf):
    """One-shot battery voltage in volts (full ~4.2 V, land by ~3.2 V)."""
    log = LogConfig(name='pm', period_in_ms=200)
    log.add_variable('pm.vbat', 'float')
    with SyncLogger(scf, log) as logger:
        for _, data, _ in logger:
            return data['pm.vbat']


# --------------------------------------------------------------------------
# Safety: keep every target inside the flight box
# --------------------------------------------------------------------------
def safe_xyz(x, y, z, box=None):
    """Clamp a target to the flight box and warn loudly if it was outside.
    Better a clamped point than a drone darting at a wall."""
    b = box or config.BOX
    cx = min(max(x, b['x_min']), b['x_max'])
    cy = min(max(y, b['y_min']), b['y_max'])
    cz = min(max(z, b['z_min']), b['z_max'])
    if (cx, cy, cz) != (x, y, z):
        print('  !! target ({:+.2f},{:+.2f},{:+.2f}) was OUTSIDE the box — '
              'clamped to ({:+.2f},{:+.2f},{:+.2f})'.format(x, y, z, cx, cy, cz))
    return cx, cy, cz


# --------------------------------------------------------------------------
# Emergency stop
# --------------------------------------------------------------------------
def emergency_stop(scf):
    """Cut motors immediately. This is the software kill switch."""
    try:
        scf.cf.high_level_commander.stop()
    except Exception:
        pass
    scf.cf.commander.send_stop_setpoint()


# --------------------------------------------------------------------------
# High-level trajectory helper (sessions 3 & 5)
# --------------------------------------------------------------------------
def fly_figure(scf, waypoints, yaw=0.0):
    """Fly a list of ABSOLUTE (x, y, z, duration_s) waypoints with the
    high-level commander, then land. Assumes prepare_for_flight() already ran.

    The drone takes off straight up, flies to the first waypoint, then walks
    the rest of the list. Every point is clamped to the flight box.
    """
    hlc = scf.cf.high_level_commander

    x0, y0, z0, _ = waypoints[0]
    x0, y0, z0 = safe_xyz(x0, y0, z0)
    hlc.takeoff(z0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME)
    hlc.go_to(x0, y0, z0, yaw, 2.0)       # slide to the start of the figure
    time.sleep(2.0)

    for (x, y, z, dur) in waypoints[1:]:
        x, y, z = safe_xyz(x, y, z)
        dur = max(dur, 1.0)
        hlc.go_to(x, y, z, yaw, dur)
        time.sleep(dur)

    hlc.land(0.0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME)
    hlc.stop()
