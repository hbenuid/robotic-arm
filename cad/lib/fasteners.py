"""The arm's own fasteners - a leaf (lib/params.py re-exports it; the lib/ packages that size a hole, a nut pocket or a
screw for them - lib/forearm/ - import it directly).

The standard sizes, the clearance holes printed parts take, and the plain-geometry screw and nut the purchased
pattern parts are built from (parts/joints/*_pulley_screws / *_nuts, like the drive's cycloidal_housing_bolts): no
thread; a screw's head carries its hex socket, a nut its bore at the thread's nominal diameter, so a screw runs
through its nut line-to-line. Volumes are those of that geometry (the masses in lib/params.py).

Tags as in lib/params.py. Units mm.
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from cadgen import build123d as bd

from lib.geom import cylinder, hex_prism, single_solid, through
from lib.units import NUDGE

# Close clearance holes for the arm's screws through printed parts.
M3_CLEAR = 3.4          # [DESIGN] M3
M4_CLEAR = 4.4          # [DESIGN] M4 (the elbow block's and the 90T pulleys' pulley bolts)
M5_CLEAR = 5.5          # [DESIGN] M5

M4_PITCH = 0.7          # [DATASHEET] M4 coarse thread


@dataclass(frozen=True)
class ShcsSize:
    """A socket head cap screw (ISO 4762)."""
    d: float              # thread (and shank) diameter
    head_dia: float       # dk max
    head_h: float         # k max
    socket_af: float      # hex socket, across flats (the key size)
    socket_depth: float   # t min


@dataclass(frozen=True)
class NutSize:
    """A hex nut (ISO 4032)."""
    d: float              # thread diameter: the modelled bore
    af: float             # s max
    h: float              # m max


M4_SHCS = ShcsSize(4.0, 7.0, 4.0, 3.0, 2.0)   # [DATASHEET] ISO 4762 M4
M4_NUT = NutSize(4.0, 7.0, 3.2)               # [DATASHEET] ISO 4032 M4


def hex_area(across_flats: float) -> float:
    return math.sqrt(3.0) / 2.0 * across_flats * across_flats


def shcs_volume(size: ShcsSize, length: float) -> float:
    """The modelled screw's volume: the head less its socket, and the shank `length` long (under the head)."""
    head = math.pi * (size.head_dia / 2.0) ** 2 * size.head_h - hex_area(size.socket_af) * size.socket_depth
    return head + math.pi * (size.d / 2.0) ** 2 * length


def nut_volume(size: NutSize) -> float:
    """The modelled nut's volume: the hex less its bore."""
    return (hex_area(size.af) - math.pi * (size.d / 2.0) ** 2) * size.h


def shcs(size: ShcsSize, length: float, xy=(0.0, 0.0)):
    """One screw: the head's bearing face on z=0, the head in -Z (its socket open at the top, z = -head_h), the shank in
    +Z to z = length."""
    head = cylinder(size.head_dia / 2.0, size.head_h, xy, z0=-size.head_h)
    socket = bd.Pos(xy[0], xy[1], -size.head_h - NUDGE) * hex_prism(size.socket_af, 0.0, size.socket_depth + NUDGE)
    return single_solid(head - socket + cylinder(size.d / 2.0, length, xy))


def hex_nut(size: NutSize, angle_rad: float, xy=(0.0, 0.0), z0: float = 0.0):
    """One nut standing on z0 (its bearing face), a corner at `angle_rad` from +X (lib.geom.hex_prism), the bore on
    its axis."""
    nut = bd.Pos(xy[0], xy[1], z0) * hex_prism(size.af, angle_rad, size.h)
    return single_solid(nut - through(size.d / 2.0, size.h, xy, z0))
