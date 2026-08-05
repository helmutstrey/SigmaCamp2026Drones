# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Teaching material for a drone camp: 17 standalone Crazyflie flight scripts (`cflib`) plus 3 shared
modules. Each script is a *lesson artifact* — it is meant to be read aloud, copied, and edited by
students, so it stays short and self-contained. The complexity lives in `cf_utils.py`; scripts are
thin. Preserve that split when adding code.

No package manifest, no test suite, no linter config. `README.md` is the student-facing counterpart
to this file and documents the session-by-session run order.

## Commands

```bash
pip install cflib                              # only dependency

# Always run from the drone_camp/ directory — scripts do sys.path.append('..')
# to import config/cf_utils/formations, and the TOC cache is './cache' (CWD-relative).
cd drone_camp
python setup_tdoa2.py                          # one-time: switch the anchors into TDoA2
python session2/s2_00_preflight_lps.py         # health check; run before any flight work
python session3/s3_02_figure.py spiral         # the one script taking an argument (a config.FIGURES key)

# Validation without hardware (this is what "tested" means here — no unit tests exist):
python -m py_compile session*/*.py *.py
python -c "import config, cf_utils, formations"
```

`lpslib` — used by `setup_tdoa2.py` and `cf_utils.set_anchor_modes_tdoa2` — ships inside the `cflib`
distribution as a separate top-level package, so there is nothing extra to install. It is still
imported lazily, at call time rather than module scope.

The `gui` extra (`uv sync --extra gui`) adds `cfclient` for the once-per-room GUI work: entering
anchor geometry and flashing firmware. Flying needs none of it and it pulls in PyQt6/vispy, so it is
not installed by default.

Everything except `setup_tdoa2`, `s1_01`, `s1_02`, `s2_00`, and `s2_03` commands motors. Those are
the only scripts that can be exercised without a calibrated flight space.

## Architecture

**`config.py` is the only file a user is expected to edit.** It holds radio URIs, the LPS mode and
anchor IDs, the flight `BOX`, per-drone `HOMES` (home x/y + altitude lane), and `FIGURES`. Never
hardcode a URI, height, or coordinate in a script — read it from `config`. `DRONE`/`PAIR`/`FEW` are
the slices scripts select their fleet from. `ANCHOR_POSITIONS` is deliberately empty by default:
cfclient is the source of truth for calibration, and filling it in creates a second one.

**`cf_utils.py`** wraps the safety-critical setup. `prepare_for_flight(scf)` (TDoA2 mode → Kalman
estimator → high-level commander → `reset_estimator`) must run before any takeoff, including inside
`Swarm.parallel_safe`. That order is load-bearing: a `loco.mode` change restarts the deck's ranging,
so it has to precede the estimator wait. `reset_estimator` blocks until position variance converges;
removing that wait causes the takeoff lurch. Every target coordinate passes through `safe_xyz()`,
which clamps to `config.BOX` and prints a warning — new movement code must call it too.

**Positioning is TDoA2** (`config.LPS_MODE = 2`, the `loco.mode` param on the deck), pinned
explicitly rather than left on auto. TDoA2 means exactly 8 anchors with IDs 0–7, anchor 0 as master
(no master, no positioning at all), and every anchor within range of every other. Diagnostics live
in `cf_utils`: `anchor_status()` reads the deck's `LocoMemory2` (`MemoryElement.TYPE_LOCO2`) for
known/active anchor IDs and positions; `check_anchors()` wraps it with the messages a student needs.
Both degrade gracefully — firmware without that memory returns `None` and the check passes rather
than blocking a flight. Call `scf.wait_for_params()` before touching `cf.mem` in a script that
doesn't otherwise go through `prepare_for_flight`.

**`formations.py`** returns `{uri: (x, y, z)}` dicts of absolute targets for the show. Drones have no
mutual sensing, so **altitude lanes are the entire collision-avoidance strategy**: `lane_z()` gives
each drone its `config.HOMES[uri]['z']` (or a staggered default) and formations must keep that z
per drone rather than flattening the fleet to one height.

**Two coordinate conventions, deliberately.** Sessions 2 & 4 use *relative* moves
(`go_to(..., relative=True)`, `MotionCommander`) so they work from any start position. Sessions 3 & 5
use *absolute* LPS-frame coordinates and therefore depend on `BOX`/`CENTER` being correct. Don't mix
them within a script.

## Conventions

- Filename encodes the lesson: `s<session>_<NN>_<slug>.py`, run in numeric order within a session.
- Every script opens with a docstring stating session/script number, `(FLIES)` or `(NO FLIGHT)`, a
  `Run:` line, `Prereq:`, and `Safety:`/`Note:` where relevant. Match this exactly for new scripts.
- Single-drone: `with cf_utils.make_scf(uri) as scf`. Multi-drone: `Swarm(uris,
  factory=CachedCfFactory(rw_cache='./cache'))` with per-drone work in module-level functions taking
  `scf`, dispatched via `swarm.parallel_safe(fn)` or `parallel_safe(fn, args_dict={uri: [arg]})`.
- After `land()`, call `high_level_commander.stop()`. `cf_utils.emergency_stop()` is the kill switch
  (`s5_03_abort_demo.py` rehearses it — an abort drops the drones, it is not a landing).
- Scale defaults are conservative on purpose (`config.FEW` = 4 drones). Don't raise a script's fleet
  size or shrink spacing/lane separation as an "improvement".
- No LED-ring code: the kit lacks the deck.
