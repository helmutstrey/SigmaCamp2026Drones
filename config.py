"""
config.py  —  ONE place to edit for your room and your fleet.

Every script imports from here. Change these values to match:
  * your Crazyradio channels + drone addresses (from the Installation Guide master sheet)
  * your measured anchor / flight-box geometry (from the LPS calibration)

Positioning: this camp runs the Loco Positioning System in **TDoA2** mode.
Coordinate frame: the Loco Positioning frame you set in cfclient.
Origin (0, 0, 0) is whatever floor corner you chose when entering anchor
geometry. +x, +y are along the floor; +z is up. All distances are METRES.
"""

# ---------------------------------------------------------------------------
# 1. RADIOS + DRONE ADDRESSES
# ---------------------------------------------------------------------------
# One channel per radio; unique address per drone. Keep this list in the same
# order as the labels on your drones. Edit to match your master sheet.
URIS = [
    'radio://0/80/2M/E7E7E7E7E1',   # drone 1
    'radio://0/80/2M/E7E7E7E7E2',   # drone 2
    'radio://0/80/2M/E7E7E7E7E3',   # drone 3
    'radio://0/80/2M/E7E7E7E7E4',   # drone 4
    'radio://0/80/2M/E7E7E7E7E5',   # drone 5
    'radio://0/80/2M/E7E7E7E7E6',   # drone 6
    'radio://0/80/2M/E7E7E7E7E7',   # drone 7
    'radio://0/80/2M/E7E7E7E7E8',  # drone 8
    'radio://0/80/2M/E7E7E7E7E9',  # drone 9
    'radio://0/80/2M/E7E7E7E7EA',  # drone 10
]

URIS_DIST = [
    'radio://0/80/2M/E7E7E7E7E1',   # drone 1
    'radio://0/80/2M/E7E7E7E7E2',   # drone 2
    'radio://0/80/2M/E7E7E7E7E3',   # drone 3
    'radio://0/80/2M/E7E7E7E7E4',   # drone 4
    'radio://1/80/2M/E7E7E7E7E5',   # drone 5
    'radio://1/80/2M/E7E7E7E7E6',   # drone 6
    'radio://1/80/2M/E7E7E7E7E7',   # drone 7
    'radio://2/80/2M/E7E7E7E7E8',  # drone 8
    'radio://2/80/2M/E7E7E7E7E9',  # drone 9
    'radio://2/80/2M/E7E7E7E7EA',  # drone 10
]

# Convenience handles used by the single-/few-drone scripts.
DRONE = URIS[0]        # default single drone for sessions 1-3
PAIR = URIS[0:2]       # two-drone scripts
FEW = URIS[0:4]        # small-swarm scripts (safe default while learning)

# ---------------------------------------------------------------------------
# 2. LOCO POSITIONING MODE  —  TDoA2
# ---------------------------------------------------------------------------
# The LPS deck param 'loco.mode': 0 = auto, 1 = TWR, 2 = TDoA2, 3 = TDoA3.
# We pin it to TDoA2 rather than leaving it on auto so a drone can never come
# up in a different mode than the anchors are broadcasting.
#
# TDoA2 rules you cannot bend:
#   * EXACTLY 8 anchors, with IDs 0..7 — no more, no fewer, no gaps.
#   * Anchor 0 is the MASTER. If it is off or out of range, the whole system
#     stops; the others cannot carry on without it (TDoA3 has no master).
#   * Every anchor must hear every other anchor — they share one fixed TDMA
#     schedule. Line-of-sight between anchors matters as much as to the drone.
# In exchange, the position update rate is fixed and does not sag as you add
# anchors or drones, which is what makes the swarm sessions predictable.
LPS_MODE = 2                       # 2 = TDoA2. Do not change for this camp.
ANCHOR_IDS = list(range(8))        # TDoA2 = anchor IDs 0..7

# OPTIONAL: anchor positions for setup_tdoa2.py to push over the air.
# cfclient's Loco Positioning tab is the SOURCE OF TRUTH for calibration —
# leave this empty ({}) and setup_tdoa2.py will only set the anchor mode.
# Fill it in (id -> (x, y, z) in metres) only if you want a scripted way to
# restore geometry after an anchor is reset or replaced.
ANCHOR_POSITIONS = {}

# ---------------------------------------------------------------------------
# 3. FLIGHT BOX  (must sit INSIDE the convex hull of your 8 anchors)
# ---------------------------------------------------------------------------
# TDoA positioning is only trustworthy inside the anchor volume. Make this box
# a bit SMALLER than the anchor box so drones never fly to the edge.
# Defaults assume a ~4 x 4 x 2.5 m anchor volume — REPLACE with your numbers.
BOX = {
    'x_min': 0.5, 'x_max': 3.5,
    'y_min': 0.5, 'y_max': 3.5,
    'z_min': 0.4, 'z_max': 1.8,
}

CENTER = ((BOX['x_min'] + BOX['x_max']) / 2.0,
          (BOX['y_min'] + BOX['y_max']) / 2.0)     # (x, y) middle of the floor

DEFAULT_HEIGHT = 0.7   # comfortable, above the noisy near-floor zone
TAKEOFF_TIME = 2.0     # seconds for take-off / landing ramps

# ---------------------------------------------------------------------------
# 4. PER-DRONE HOMES + ALTITUDE LANES (for swarm sessions 4-5)
# ---------------------------------------------------------------------------
# Each drone gets a distinct home (x, y) AND a distinct altitude 'z' lane so
# their paths never share the same airspace (our drones cannot sense each
# other). Edit to spread across your real box. Only URIs listed here fly in
# the choreography scripts — start with a few, add more once it looks clean.
HOMES = {
    'radio://0/80/2M/E7E7E7E701': {'home': (1.2, 1.2), 'z': 0.6},
    'radio://0/80/2M/E7E7E7E702': {'home': (2.8, 1.2), 'z': 0.9},
    'radio://1/80/2M/E7E7E7E704': {'home': (2.8, 2.8), 'z': 1.2},
    'radio://1/80/2M/E7E7E7E705': {'home': (1.2, 2.8), 'z': 1.5},
    # add more drones here as you scale the show ...
}

# ---------------------------------------------------------------------------
# 5. SOLO FIGURES (session 3) — lists of (x, y, z, duration_s) absolute points
# ---------------------------------------------------------------------------
_cx, _cy = CENTER
FIGURES = {
    'square': [
        (_cx - 0.6, _cy - 0.6, 0.7, 2.5),
        (_cx + 0.6, _cy - 0.6, 0.7, 2.5),
        (_cx + 0.6, _cy + 0.6, 0.7, 2.5),
        (_cx - 0.6, _cy + 0.6, 0.7, 2.5),
        (_cx - 0.6, _cy - 0.6, 0.7, 2.5),
    ],
    'triangle': [
        (_cx,       _cy + 0.7, 0.8, 2.5),
        (_cx + 0.6, _cy - 0.5, 0.8, 2.5),
        (_cx - 0.6, _cy - 0.5, 0.8, 2.5),
        (_cx,       _cy + 0.7, 0.8, 2.5),
    ],
    # a rising spiral: same (x,y) loop but climbing each lap
    'spiral': [
        (_cx + 0.5, _cy,       0.6, 2.0),
        (_cx,       _cy + 0.5, 0.8, 2.0),
        (_cx - 0.5, _cy,       1.0, 2.0),
        (_cx,       _cy - 0.5, 1.2, 2.0),
        (_cx + 0.5, _cy,       1.4, 2.0),
        (_cx,       _cy,       1.4, 2.0),
    ],
}
