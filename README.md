# Drone Camp — Ground-Truth Flight Scripts

Runnable Crazyflie/`cflib` scripts covering every session, so you can test each
step yourself before the lecture. Everything shares one config file and one
helper library, so the flight logic stays short and readable.

Verified against **cflib 0.1.32**: every script byte-compiles, imports cleanly,
and every API call it makes exists in the library. What isn't verified here is
the flying itself — that needs your hardware and calibrated anchors.

---

## Prerequisites

1. **Software** (Mac or Windows). With [uv](https://docs.astral.sh/uv/), from
   this folder:
   ```
   uv sync                    # creates .venv and installs cflib, pinned by uv.lock
   uv sync --extra gui        # ...plus cfclient, if you need the GUI on this machine
   ```
   Then prefix every command with `uv run` (`uv run python session2/...`), or
   activate the environment once with `source .venv/bin/activate`.

   Without uv, `pip install cflib` into a virtualenv of your own works too —
   nothing here depends on uv at runtime.
2. **Positioning up:** exactly **8 anchors**, IDs **0–7**, placed, calibrated,
   and switched to **TDoA2** (in cfclient's Loco Positioning tab — see the
   Installation Guide — or by running `python setup_tdoa2.py` once).
3. **Per drone:** LPS deck attached, firmware current, a **unique address**, and
   it starts **inside the flight box**. The scripts pin each deck to TDoA2
   themselves, so you don't have to set `loco.mode` per drone by hand.
4. **Edit `config.py`** — this is the only file you must change:
   - `URIS` to your real radio/channel/address list,
   - `BOX` to your measured anchor volume (a bit smaller than the anchor box),
   - `HOMES` to give each swarm drone a distinct home + altitude lane.

Run scripts from inside the `drone_camp/` folder, e.g.:
```
cd drone_camp
uv run python session2/s2_00_preflight_lps.py
```
The scripts add the project root to `sys.path` themselves and write their
Crazyflie TOC cache to `./cache`, so the working directory matters.

---

## Safety (every flight)

- Glasses on in the flight area; props are sharp.
- **`Ctrl-C` is the kill switch** for any script. `session5/s5_03_abort_demo.py`
  rehearses the hard motor-cut.
- Start with **few drones** (`config.FEW` = first 4) and scale up only once a
  routine looks clean. The drones **cannot see each other** — lanes and spacing
  are your only collision avoidance.
- Run `s2_00_preflight_lps.py` on each drone at the start of every flight day.

---

## What to run, session by session

**Session 1 — first flight (mostly no-fly)**
- `s1_01_connect_test.py` — link + battery check. Proves the radio chain works.
- `s1_02_log_attitude.py` — tilt the drone by hand, watch roll/pitch/yaw. The IMU, live.
- `s1_03_hover_trigger.py` — *optional* gentle autonomous hover if you have no gamepads. (Flies; needs LPS.)
- `s1_04_hover_land.py` — same hover, but it descends past the floor and cuts the motors itself instead of letting `MotionCommander` land and hand over to the high-level commander. Use this one if a drone lifts off again as the script disconnects. (Flies; needs LPS.)
- `s1_05_fly_forward_back.py` — one drone out 0.5 m along +x and back, in absolute coordinates. The sanity check for a single drone's position hold before you scale up. (Flies; needs LPS.)

**Session 2 — positioning + first autonomous flight**
- `s2_00_preflight_lps.py` — **run first.** Verifies deck, TDoA2 mode, all 8 anchors, estimator, position. Your health check.
- `s2_01_hello_hover.py` — take off, hover 3 s, land.
- `s2_02_first_waypoint.py` — out to a point and back (relative moves).
- `s2_03_print_position.py` — carry the drone around, watch TDoA2 track it; walk it outside the box to see the estimate degrade. Also lists the anchors currently heard.
- `s2_04_two_drone_hover.py` — two drones, one at a time: A takes off to its slot and hovers at 0.4 m, then B, then both land. Pick the pair by index at the top of the file. Slots are 0.3 m apart at the *same* height, which is tighter than the altitude-lane separation the swarm scripts use — read the warning in the docstring first.
- `s2_05_two_drone_orbit.py` — the same pair, now moving: they slide onto opposite ends of a 0.3 m line and walk it once around its middle, then land gently. Streams position setpoints to both drones rather than using `go_to`, so the circle is smooth and the separation is exact by construction. Fly `s2_04` first.
- `s2_06_square.py` — four drones on **one** radio: each takes off in turn to its corner of a 0.8 m square centred in the box at mid-height, all hold 2 s, then land one at a time. 40 packets/s through a single dongle — stutter here is radio bandwidth, not LPS.

**Session 3 — trajectories (single drone, absolute coordinates)**
- `s3_01_solo_square.py` — a square from an explicit waypoint list.
- `s3_02_figure.py [name]` — fly any figure in `config.FIGURES` (`square`/`triangle`/`spiral`). Template for student figures.
- `s3_03_timing_demo.py` — same square slow then fast; duration = choreography.

**Session 4 — swarming**
- `s4_01_two_drone_takeoff.py` — two drones take off / hover / land in sync.
- `s4_02_swarm_square_relative.py` — the whole group flies parallel squares.
- `s4_03_swarm_scale_test.py` — each drone hovers in its own altitude lane, in place. Add drones to one channel to find the radio limit safely.

**Session 5 — the show**
- `s5_01_formations.py` — line → circle → line transitions.
- `s5_02_ceremony_show.py` — ★ the full phased routine. Edit `build_phases()` to design your piece.
- `s5_03_abort_demo.py` — the abort drill (drones drop — do it low over a soft area).
- `s5_04_swarm_shapes.py` — ★ *student exercise.* All 8 drones take off into a grid of homes + altitude lanes; `fly_shapes()` is left empty for your choreography. Ctrl-C or any exception in your code lands the whole swarm. Largest fleet in the material — sanity-check one drone with `s1_05` first. (Flies; 8 drones.)

---

## Files

```
drone_camp/
  pyproject.toml    # dependencies (cflib; cfclient behind the 'gui' extra)
  uv.lock           # exact resolved versions — commit this
  .python-version   # 3.12
  config.py         # ← EDIT: URIs, LPS mode, flight box, homes/lanes, figures
  cf_utils.py       # shared helpers: connect, TDoA2 mode, anchor + estimator checks, box clamp, stop
  formations.py     # per-drone target geometry (line / circle / two rows)
  setup_tdoa2.py    # one-time: switch the ANCHORS into TDoA2 (same as cfclient's LPS tab)
  session1..5/      # the scripts above
```

## How these were validated

- `python -m py_compile` on all 22 scripts + 4 modules — pass.
- Import of every module and script against cflib 0.1.32 — pass.
- API-surface assertions (every `high_level_commander`, `Swarm`, `MotionCommander`,
  `param`, `commander`, `LogConfig` call the scripts use) — pass, including the
  `go_to(..., relative=...)` signature and `Swarm.parallel_safe(args_dict=...)`.
- TDoA2 specifics checked against the installed cflib/cfclient/lpslib: the
  `loco.mode` param and its values (2 = TDoA2), `LoPoAnchor.MODE_TDOA` /
  `set_mode` / `set_position`, and the `LocoMemory2` anchor-memory API
  (`update_id_list` / `update_active_id_list` / `update_data`) — pass.
- `check_anchors()` exercised against stubbed anchor memories: all 8 present,
  master missing, stray out-of-range anchor, invalid position, and old
  firmware with no anchor memory — all report correctly.
- Geometry check: all default formations and figures compute inside `BOX`.

Still unverified without your hardware: the flying, and whether your anchors
actually accept the mode switch from `setup_tdoa2.py`.

## Design notes / gotchas

- **TDoA2, not TDoA3.** Every script pins `loco.mode = 2` on the deck
  (`cf_utils.use_tdoa2_mode`, called from `prepare_for_flight`) instead of
  leaving it on auto, so one drone can never guess a different mode than the
  rest. TDoA2's rules are stricter than TDoA3's:
  - **Exactly 8 anchors, IDs 0–7.** No gaps, no ninth anchor. Anchors outside
    that range are simply ignored.
  - **Anchor 0 is the master.** If it loses power the whole system stops —
    the other seven cannot carry on without it. Check it first when nothing
    ranges. `s2_00` calls this out by name.
  - **Every anchor must hear every other anchor.** They share one fixed TDMA
    schedule, so anchor-to-anchor line of sight matters as much as
    anchor-to-drone. A missing anchor is a hole in the schedule, not something
    the system routes around.
  - In exchange the update rate is **fixed** — it doesn't sag as you add
    anchors, and drones are passive listeners, so adding drones costs the
    positioning system nothing. In `s4_03` everything you see degrade is
    Crazyradio bandwidth, not LPS.
- **Frames.** Sessions 2 & 4 use *relative* moves (work from any start). Sessions
  3 & 5 use *absolute* room coordinates (need `BOX`/`CENTER` set correctly). The
  high-level commander's absolute `go_to` uses the LPS global frame directly.
- **Estimator wait.** `reset_estimator()` blocks until the position variance is
  low. This prevents the classic take-off lurch; don't remove it.
- **No LED color.** The color-light effects would need the LED-ring deck (not in
  your kit), so nothing here drives ring LEDs. Add it later if you get the decks.
- **Batteries.** Scripts don't manage charge. Keep the battery marshal rotation
  from the lesson plan; ~5 usable minutes per pack.
- **Collision avoidance is by design.** Distinct homes + altitude lanes in
  `config.HOMES` are what keep drones apart. Tune these for your real placement
  before running the multi-drone scripts with more than a few aircraft.
```
