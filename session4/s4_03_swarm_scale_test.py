"""
Session 4 · Script 3 — Scale / bandwidth test  (FLIES)

Every drone in config.HOMES takes off straight up to its OWN altitude lane,
hovers in place, and lands. Because nothing moves horizontally, there is no
collision risk — this is purely about how many drones a radio can carry.

Use it to find your limit: add more URIs to config.HOMES ON THE SAME CHANNEL
and watch when flight gets shaky. Then spread them across more radios/channels
and watch it recover. That's the bandwidth lesson, live.

The positioning system is NOT the bottleneck here. In TDoA2 the anchors run a
fixed broadcast schedule and the drones only listen — a hundred drones would
cost the anchors nothing. Everything you see degrade is Crazyradio bandwidth
between your laptop and the drones.

Run:  python s4_03_swarm_scale_test.py
Prereq: each URI in config.HOMES passes preflight and has a distinct 'z' lane.
"""

import sys, os, time
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.swarm import CachedCfFactory, Swarm

import cf_utils
import config


def lane_height(scf):
    return config.HOMES.get(scf.cf.link_uri, {}).get('z', config.DEFAULT_HEIGHT)


def take_off_lane(scf):
    scf.cf.high_level_commander.takeoff(lane_height(scf), config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)


def land(scf):
    scf.cf.high_level_commander.land(0.0, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)
    scf.cf.high_level_commander.stop()


def main():
    uris = list(config.HOMES.keys())
    print('Scale test with {} drones (each in its own altitude lane):'.format(len(uris)))
    for u in uris:
        print('   {}  ->  z = {:.1f} m'.format(u, config.HOMES[u]['z']))

    cf_utils.init()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(uris, factory=factory) as swarm:
        swarm.parallel_safe(cf_utils.prepare_for_flight)
        swarm.parallel_safe(take_off_lane)
        print('  hovering — watch for stability')
        time.sleep(5.0)
        swarm.parallel_safe(land)
    print('  done.')


if __name__ == '__main__':
    main()
