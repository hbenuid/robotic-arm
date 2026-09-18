"""Small build123d helpers shared by the cycloidal drive builders (algebra mode)."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.units import NUDGE   # not lib.params: that module imports lib.cycloidal.params (no import back up)

def align_min():
    """`align=` for a body standing on z0: footprint centred, extends +Z. A function, not a constant:
    an Align at module level would import the kernel with this module."""
    return (bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)


def cylinder(radius: float, height: float, xy=(0.0, 0.0), z0: float = 0.0):
    """A cylinder standing on z0 (exact height - use for blind holes and bodies)."""
    return bd.Pos(xy[0], xy[1], z0) * bd.Cylinder(radius, height, align=align_min())


def through(radius: float, height: float, xy=(0.0, 0.0), z0: float = 0.0):
    """A through-hole cutter spanning z0..z0+height with NUDGE overshoot on both ends."""
    return cylinder(radius, height + 2 * NUDGE, xy, z0 - NUDGE)


def single_solid(shape: bd.Shape) -> bd.Solid:
    """The one Solid of an algebra-mode result (raises if a cutter leaked or the body split)."""
    solids = shape.solids()
    if len(solids) != 1:
        raise ValueError(f"expected exactly one solid, got {len(solids)}")
    return solids[0]
