"""
Session 5 · Script 3 — Abort drill  (FLIES)

Rehearses the software kill switch. The swarm takes off and hovers; when you
press ENTER (or after a timeout), every motor cuts immediately. Practise this
until calling and executing an all-stop is reflexive — it is the most important
thing the flight director does.

Run:  python s5_03_abort_demo.py
Prereq: drones in config.PAIR pass preflight.
Note: an abort DROPS the drones — they do not land gently. Do it low (0.5 m)
      and over a clear, soft area. That is the point of the drill.
"""

import sys, os, time, threading
sys.path.append(os.path.join(os.path.dirname(__file__), '..'))

from cflib.crazyflie.swarm import CachedCfFactory, Swarm

import cf_utils
import config


def take_off(scf):
    scf.cf.high_level_commander.takeoff(0.5, config.TAKEOFF_TIME)
    time.sleep(config.TAKEOFF_TIME + 0.5)


def main():
    cf_utils.init()
    factory = CachedCfFactory(rw_cache='./cache')
    with Swarm(config.PAIR, factory=factory) as swarm:
        swarm.parallel_safe(cf_utils.prepare_for_flight)
        print('  take off to 0.5 m')
        swarm.parallel_safe(take_off)

        # Wait for an abort trigger: ENTER, or auto-abort after 6 s.
        print('  HOVERING. Press ENTER to ABORT (auto-abort in 6 s) ...')
        triggered = threading.Event()
        threading.Thread(target=lambda: (input(), triggered.set()), daemon=True).start()
        triggered.wait(timeout=6.0)

        print('  *** ABORT — cutting motors ***')
        swarm.parallel_safe(cf_utils.emergency_stop)
    print('  motors cut. Drill complete.')


if __name__ == '__main__':
    main()
