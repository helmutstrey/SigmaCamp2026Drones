"""
formations.py  —  compute per-drone target points for the show (Session 5).

Each function returns a dict {uri: (x, y, z)} of ABSOLUTE targets. Drones are
assigned to slots by their order in the `uris` list, and each keeps its own
altitude lane (from config.HOMES, or a staggered default) so paths stay
vertically separated during transitions — remember, the drones can't see each
other.
"""

import math
import config


def lane_z(uri, i):
    """Altitude lane for a drone: its configured z, or a staggered default."""
    if uri in config.HOMES:
        return config.HOMES[uri]['z']
    return min(config.BOX['z_max'] - 0.1, 0.6 + 0.25 * i)


def line(uris, span=None):
    """Evenly spaced along x, at the room's mid-y, each on its own lane."""
    cx, cy = config.CENTER
    n = len(uris)
    if span is None:
        span = min(2.4, (config.BOX['x_max'] - config.BOX['x_min']) - 0.4)
    out = {}
    for i, uri in enumerate(uris):
        x = cx - span / 2 + (span * i / (n - 1) if n > 1 else 0)
        out[uri] = (x, cy, lane_z(uri, i))
    return out


def circle(uris, radius=None):
    """Evenly spaced around a circle centred on the room, each on its own lane."""
    cx, cy = config.CENTER
    n = len(uris)
    if radius is None:
        radius = min(1.0, (config.BOX['x_max'] - config.BOX['x_min']) / 2 - 0.4)
    out = {}
    for i, uri in enumerate(uris):
        a = 2 * math.pi * i / n
        out[uri] = (cx + radius * math.cos(a), cy + radius * math.sin(a), lane_z(uri, i))
    return out


def two_rows(uris, gap=0.9):
    """Split the fleet into two rows (front/back) — a simple 'split' look."""
    cx, cy = config.CENTER
    half = (len(uris) + 1) // 2
    out = {}
    for i, uri in enumerate(uris):
        row = 0 if i < half else 1
        idx = i if row == 0 else i - half
        count = half if row == 0 else len(uris) - half
        span = min(2.0, (config.BOX['x_max'] - config.BOX['x_min']) - 0.6)
        x = cx - span / 2 + (span * idx / (count - 1) if count > 1 else 0)
        y = cy - gap / 2 + row * gap
        out[uri] = (x, y, lane_z(uri, i))
    return out
