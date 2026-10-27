"""Small build123d helpers shared by the arm's builders - the drive, the links, the purchased parts (algebra mode)."""
from __future__ import annotations

import math

import build123d as bd

from lib.units import NUDGE  # not lib.params: that module imports lib.cycloidal.params (no import back up)


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


def hex_circumdiameter(across_flats: float) -> float:
    """Across-corners diameter of a hexagon from its across-flats size."""
    return across_flats / math.cos(math.radians(30))


def hex_prism(across_flats: float, angle_rad: float, height: float):
    """A hexagonal prism standing on z=0, first vertex on +X rotated by ``angle_rad`` (the
    CadQuery ``polygon(6, d)`` + ``transformed(rotate=...)`` convention)."""
    r = hex_circumdiameter(across_flats) / 2.0
    return bd.extrude(bd.RegularPolygon(r, 6, rotation=math.degrees(angle_rad)), amount=height, dir=(0, 0, 1))


# --- the belt drives' frames and rotors (lib/forearm/roll.py, lib/wrist/roll.py): a drive's axis is its frame's Z ---

def box(x, y, z):
    """A plain box over the x / y / z ranges ((x0, x1), ...)."""
    return bd.Pos(x[0], y[0], z[0]) * bd.Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))


def stop_wedge(r_in: float, r_out: float, z0: float, height: float, deg_width: float, at_deg: float):
    """A hard-stop lug or post: a radial box from r_in to r_out, deg_width wide at r_out (its sides parallel), standing
    on z0, turned to at_deg about Z."""
    half = r_out * math.tan(math.radians(deg_width / 2.0))
    lug = bd.Pos(r_in, 0.0, z0) * bd.Box(r_out - r_in, 2 * half, height, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    return bd.Rot(0.0, 0.0, at_deg) * lug


def slot_y(x: float, y: float, width: float, length: float, z0: float, depth: float):
    """A slot `length` long along Y, `width` wide, through a plate from z0 (a motor plate's tension slots, its pilot
    slot)."""
    return bd.Pos(x, y, z0 - NUDGE) * bd.Rot(0.0, 0.0, 90.0) * bd.extrude(bd.SlotCenterToCenter(length, width), amount=depth + 2 * NUDGE)


def nut_slot(r_in: float, r_out: float, af: float, z0: float, z1: float, at_rad: float):
    """A nut's pocket: a radial slot from r_in to r_out, af wide (a flat either side of the nut), z0 .. z1, along at_rad."""
    slot = bd.Pos((r_in + r_out) / 2.0, 0.0, z0) * bd.Box(r_out - r_in, af, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    return bd.Rot(0.0, 0.0, math.degrees(at_rad)) * slot
