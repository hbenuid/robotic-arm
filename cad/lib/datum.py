"""The arm's datum frames: the SolidWorks capture frame W and the base_link frame B.

W is the frame of reference/placements.json: millimetres, **+Y up** (the base_yaw axis), the arm
extends toward -X. B is the REP-103 base frame of the robot description (Z up, X forward), at the
base_yaw axis foot on the base's mounting face: origin (0, BASE_BOTTOM_Y, 0), Z_B = +Y_W, X_B = BASE_FORWARD - -X_W
turned BASE_TURN_DEG about +Y_W with the base (lib/placements.py TURNS), so the arm stands square to it at zero -,
Y_B = Z_B x X_B (+Z_W turned as much).

Both the kinematic decomposition (robot/frames.py: every link / joint frame is built with frame())
and the arm assembly (assemblies/arm.py emits the arm in B, arm_from_w() = base_frame()^-1) need these,
so they live below both - assemblies/ never imports robot/.

Nothing here touches the CAD kernel at import (a model file must stay kernel-free until cadgen has
gated it - tests/test_lazy_kernel.py): a frame a module DECLARES is data, `(position mm,
rotation_xyz_deg)` like a placements.json record (IDENTITY, a part's LOCAL_FROM_REF / VENDOR_TO_REF),
turned into a Location by to_location() inside a body; base_frame() is computed on first use.
"""
from __future__ import annotations

import functools
import math

import build123d as bd

from lib.base.params import DEFAULT as _BASE

U = (0.0, 1.0, 0.0)                          # base_yaw axis: world up
CAPTURE_YAW_DEG = -86.305545                 # [REFERENCE] j1_coupler#1's capture rotation about U (placements.json; tests/test_placements.py)
# [DESIGN] the base turned about U so its octagon (lib/base/params.py ShellParams.sides) meets j1_coupler's faceted disc
# (lib/yaw_coupler/params.py DiscParams.sides) flat to flat at the capture pose, where the coupler sat a fraction of a
# facet off it: the capture's yaw less the nearest whole number of facets (lib/placements.py TURNS turns base#1, and so
# all base_link carries, by it); base_link's frame turns with the base, so the arm stands square to it at zero
_FACET_DEG = 360.0 / _BASE.shell.sides if _BASE.shell.sides else 0.0
BASE_TURN_DEG = round(CAPTURE_YAW_DEG - round(CAPTURE_YAW_DEG / _FACET_DEG) * _FACET_DEG, 6) if _FACET_DEG else 0.0
BASE_FORWARD = (-math.cos(math.radians(BASE_TURN_DEG)), 0.0, math.sin(math.radians(BASE_TURN_DEG)))   # the base's -X_W, turned with it:
#                                                                                                       the arm extends along it at zero
BASE_BOTTOM_Y = _BASE.shell.y0               # -112.0 [DESIGN] the base's bottom face (mounting face; base#1 sits at identity): the
#                                              SolidWorks -100.9 lowered under the base_yaw motor's board (lib/base/params.py BOARD_CLEAR)

IDENTITY = ((0.0, 0.0, 0.0), (0.0, 0.0, 0.0))   # a frame as data: (position mm, rotation_xyz_deg)


def to_location(frame_data=IDENTITY) -> bd.Location:
    """The Location of a frame given as data. IDENTITY gives a bare `Location()` - not a zero
    translation + rotation, which OCCT carries as a real (if trivial) transform."""
    position, rotation = frame_data
    if tuple(position) == IDENTITY[0] and tuple(rotation) == IDENTITY[1]:
        return bd.Location()
    return bd.Location(tuple(position), tuple(rotation))


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def frame(origin_w, z_w, x_hint_w) -> bd.Location:
    """World Location of a right-handed frame: Z along z_w, X along x_hint_w projected
    perpendicular to Z, origin at origin_w (mm)."""
    z = _unit(z_w)
    d = sum(a * b for a, b in zip(x_hint_w, z, strict=True))
    x = _unit(tuple(a - d * b for a, b in zip(x_hint_w, z, strict=True)))
    return bd.Location(bd.Plane(origin=bd.Vector(*origin_w), x_dir=bd.Vector(*x), z_dir=bd.Vector(*z)))


@functools.cache
def base_frame() -> bd.Location:
    """B in W: the base_link frame (Z up, X forward, origin on the base's mounting face)."""
    return frame((0.0, BASE_BOTTOM_Y, 0.0), U, BASE_FORWARD)
