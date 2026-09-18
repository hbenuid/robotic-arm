"""Small build123d helpers shared by the cycloidal drive builders (algebra mode)."""
from __future__ import annotations

from build123d import Align, Cylinder, Pos, Shape, Solid

from lib.units import NUDGE   # not lib.params: that module imports lib.cycloidal.params (no import back up)

MIN = (Align.CENTER, Align.CENTER, Align.MIN)   # footprint centred, extends +Z from z0


def cylinder(radius: float, height: float, xy=(0.0, 0.0), z0: float = 0.0):
    """A cylinder standing on z0 (exact height - use for blind holes and bodies)."""
    return Pos(xy[0], xy[1], z0) * Cylinder(radius, height, align=MIN)


def through(radius: float, height: float, xy=(0.0, 0.0), z0: float = 0.0):
    """A through-hole cutter spanning z0..z0+height with NUDGE overshoot on both ends."""
    return cylinder(radius, height + 2 * NUDGE, xy, z0 - NUDGE)


def single_solid(shape: Shape) -> Solid:
    """The one Solid of an algebra-mode result (raises if a cutter leaked or the body split)."""
    solids = shape.solids()
    if len(solids) != 1:
        raise ValueError(f"expected exactly one solid, got {len(solids)}")
    return solids[0]
