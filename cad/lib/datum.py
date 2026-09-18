"""The arm's datum frames: the SolidWorks capture frame W and the base_link frame B.

W is the frame of reference/placements.json: millimetres, **+Y up** (the base_yaw axis), the arm
extends toward -X. B is the REP-103 base frame of the robot description (Z up, X forward), at the
base_yaw axis foot on the base's mounting face: origin (0, BASE_BOTTOM_Y, 0), X_B = -X_W,
Y_B = +Z_W, Z_B = +Y_W.

Both the kinematic decomposition (robot/frames.py: every link / joint frame is built with frame())
and the arm assembly (assemblies/arm.py emits the arm in B, ARM_FROM_W = BASE_FRAME^-1) need these,
so they live below both - assemblies/ never imports robot/.
"""
from __future__ import annotations

import math

from build123d import Location, Plane, Vector

U = (0.0, 1.0, 0.0)                          # base_yaw axis: world up
BASE_FORWARD = (-1.0, 0.0, 0.0)              # the arm extends toward -X_W
BASE_BOTTOM_Y = -100.9                       # [REFERENCE] base world bbox min Y (mounting face)


def _unit(v):
    n = math.sqrt(sum(x * x for x in v))
    return tuple(x / n for x in v)


def frame(origin_w, z_w, x_hint_w) -> Location:
    """World Location of a right-handed frame: Z along z_w, X along x_hint_w projected
    perpendicular to Z, origin at origin_w (mm)."""
    z = _unit(z_w)
    d = sum(a * b for a, b in zip(x_hint_w, z))
    x = _unit(tuple(a - d * b for a, b in zip(x_hint_w, z)))
    return Location(Plane(origin=Vector(*origin_w), x_dir=Vector(*x), z_dir=Vector(*z)))


BASE_FRAME: Location = frame((0.0, BASE_BOTTOM_Y, 0.0), U, BASE_FORWARD)
