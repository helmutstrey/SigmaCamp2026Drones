"""
Session 5 · Script 4 — Swarm shapes, the student exercise  (FLIES — 8 drones)

8-drone swarm takeoff over TDoA2 / Loco Positioning System.

Everything up through "hovering at home positions" is done: arm, pin
TDoA2 mode, reset + wait for the position estimator on all 8 drones in
parallel, synchronized takeoff into a grid of home positions/altitude
lanes. Your job is fly_shapes() below - that's the only gap.

Before running: physically place drone i near the home position printed
at startup (uses config.HOMES if you've filled it in, otherwise an
auto-generated grid inside config.BOX).

Ctrl+C, or any exception (including ones from your own shape code),
lands the whole swarm - it does not fall out of the sky.

Run:  python s5_04_swarm_shapes.py
Prereq: preflight (session2/s2_00_preflight_lps.py) passes on EVERY drone;
        each one placed near its printed home with its NOSE ALONG +x.
        Sanity-check a single drone with session1/s1_05_fly_forward_back.py
        first.
Safety: area clear, glasses on. Eight aircraft is the largest fleet in this
        material — the drones cannot sense each other, so the altitude lanes
        in the printed home list are the only thing keeping them apart.
"""
import math
import os
import sys
import time

sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

import cflib.crtp
from cflib.crazyflie.swarm import CachedCfFactory, Swarm

import config

URIS = config.URIS[:8]


def build_homes(uris):
    """Per-URI {'home': (x, y), 'z': z}. Uses config.HOMES where set,
    otherwise spreads the rest across config.BOX on a grid with a
    distinct altitude lane each (see config.py section 4 for why)."""
    box = config.BOX
    n = len(uris)
    cols = math.ceil(math.sqrt(n))
    rows = math.ceil(n / cols)

    def lerp(lo, hi, t):
        return lo + (hi - lo) * t

    homes = {}
    for i, uri in enumerate(uris):
        configured = config.HOMES.get(uri)
        if configured:
            homes[uri] = {'home': tuple(configured['home']), 'z': configured['z']}
            continue

        row, col = divmod(i, cols)
        tx = col / (cols - 1) if cols > 1 else 0.5
        ty = row / (rows - 1) if rows > 1 else 0.5
        tz = i / (n - 1) if n > 1 else 0.5

        x = lerp(box['x_min'], box['x_max'], tx)
        y = lerp(box['y_min'], box['y_max'], ty)
        z = lerp(box['z_min'], box['z_max'], tz)
        homes[uri] = {'home': (x, y), 'z': z}
    return homes


def arm(scf):
    scf.cf.supervisor.send_arming_request(True)
    time.sleep(0.1)


def pin_tdoa2(scf):
    try:
        scf.cf.param.set_value('loco.mode', str(config.LPS_MODE))
    except KeyError:
        pass  # older firmware: mode is set on the anchors, not the CF
    scf.cf.param.set_value('stabilizer.estimator', '2')


def take_off(scf, home):
    x, y = home['home']
    z = home['z']
    commander = scf.cf.high_level_commander
    commander.takeoff(z, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME)
    commander.go_to(x, y, z, 0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME)


def land(scf, home):
    commander = scf.cf.high_level_commander
    commander.land(0.0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME)
    commander.stop()


def safe_go_to(scf, x, y, z, yaw, duration_s):
    """Clamped go_to for use inside fly_shapes() - clips to config.BOX
    so a typo in your shape math can't send a drone past the anchors."""
    box = config.BOX
    x = min(max(x, box['x_min']), box['x_max'])
    y = min(max(y, box['y_min']), box['y_max'])
    z = min(max(z, box['z_min']), box['z_max'])
    scf.cf.high_level_commander.go_to(x, y, z, yaw, duration_s)
    time.sleep(duration_s)


# =============================================================================
# YOUR CHOREOGRAPHY GOES HERE
# =============================================================================
def fly_shapes(swarm, homes):
    """
    Called once all 8 drones are hovering at their home (x, y, z).
    `homes` is {uri: {'home': (x, y), 'z': z}} - each drone's own
    starting point, so you can compute offsets/formations from it.

    Drones do not sense each other - any path that crosses another
    drone's altitude lane at the same time/place is your responsibility.

    Example - send every drone up 0.2m and back down:

        def bounce(scf, home):
            x, y = home['home']
            z = home['z']
            safe_go_to(scf, x, y, z + 0.2, 0, 1.5)
            safe_go_to(scf, x, y, z, 0, 1.5)

        args_dict = {uri: [homes[uri]] for uri in homes}
        swarm.parallel_safe(bounce, args_dict)
    """
    pass  # <-- put your shape code here


def main():
    cflib.crtp.init_drivers()
    homes = build_homes(URIS)

    print('Home positions (place drones here before takeoff):')
    for uri, h in homes.items():
        x, y = h['home']
        print(f'  {uri}: x={x:.2f} y={y:.2f} z={h["z"]:.2f}')

    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(URIS, factory=factory) as swarm:
        print('\nPinning TDoA2 ...')
        swarm.parallel_safe(pin_tdoa2)

        print('Resetting estimators, waiting for position lock on all 8 ...')
        swarm.reset_estimators()

        args_dict = {uri: [homes[uri]] for uri in URIS}
        try:
            print('Arming ...')
            swarm.parallel_safe(arm)

            print('Taking off ...')
            swarm.parallel_safe(take_off, args_dict)

            print('At home positions. Running choreography ...')
            fly_shapes(swarm, homes)

        except KeyboardInterrupt:
            print('\nInterrupted - landing now.')
        except Exception as e:
            print(f'\nError during flight: {e} - landing now.')
        finally:
            print('Landing all drones ...')
            swarm.parallel_safe(land, args_dict)


if __name__ == '__main__':
    main()
