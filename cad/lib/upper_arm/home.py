"""build123d builder of the elbow_pitch home sensor's clip (lib/upper_arm/params.py ElbowHomeParams;
parts/joints/elbow_home_clip): built in j1_link's part frame (x along the link, y = N, z across), then moved into its own
(lib/upper_arm/layout.py home_clip_frame()). Imported explicitly by the part, so importing lib.upper_arm never pulls in
OCCT."""
from __future__ import annotations

import build123d as bd

from lib.datum import to_location
from lib.geom import box, hull_solid, single_solid, teardrop
from lib.units import NUDGE
from lib.upper_arm.layout import (
    home_board_points,
    home_clip_frame,
    home_clip_z,
    home_end_x,
    home_plate_box,
    home_reliefs,
    home_sensor_frame,
    home_slab_y,
    home_to_link,
)
from lib.upper_arm.params import DEFAULT, UpperArmConfig


def build_home_clip(cfg: UpperArmConfig = DEFAULT):
    """The clip, one solid in its part frame: the spine outside the slab's end flat, `gap` off it, from under the board's
    plate up past the slab's top; the top arm on the slab's top face from top_x, the jaw under its underside from jaw_x;
    under the 90T's face the web - the convex hull of the board's plate and the spine's foot, the plate's depth - with the
    board's plate in it; the plate's relief slots under the header's pins and the chip's leads; the board's 2 M2 holes
    through the plate, teardrops peaked toward +z (up as it prints, on its -z face)."""
    h = cfg.home
    under, top = home_slab_y(cfg)
    z = home_clip_z(cfg)
    x0 = home_end_x(cfg) + h.gap
    x1 = x0 + h.spine_t
    (px, py, pz) = home_plate_box(cfg)
    foot_y = home_to_link(cfg, (0.0, 0.0, pz[0]))[1]         # the plate's underside: the web's and the spine's foot
    board_y = home_to_link(cfg, (0.0, 0.0, pz[1]))[1]        # the board's underside: the web's top
    top_y = top + h.gap + h.arm_t
    elbow_x = cfg.slab.elbow_x
    body = box((x0, x1), (foot_y, top_y), z)                                              # the spine
    body = body + box((elbow_x + h.top_x, x1), (top + h.gap, top_y), z)                    # the top arm
    body = body + box((elbow_x + h.jaw_x, x1), (under - h.gap - h.arm_t, under - h.gap), z)   # the jaw
    plate = [home_to_link(cfg, (x, y, 0.0)) for x in px for y in py]
    web = [(p[0], wy, p[2]) for p in plate for wy in (foot_y, board_y)]
    web += [(x, wy, wz) for x in (x0, x1) for wy in (foot_y, board_y) for wz in z]
    body = body + hull_solid(web)
    sensor = to_location(home_sensor_frame(cfg))
    for rx, ry, depth in home_reliefs(cfg):
        body = body - sensor * box(rx, ry, (-depth, NUDGE))
    for x, y in home_board_points(cfg):
        # the teardrop's peak (its +Y) along the KY-003's +X: this frame's +z, up as the clip prints
        body = body - sensor * bd.Pos(x, y, 0.0) * bd.Rot(0.0, 0.0, -90.0) * teardrop(h.board_hole_dia / 2.0, h.plate_t, z0=pz[0])
    return single_solid(to_location(home_clip_frame(cfg)).inverse() * body)
