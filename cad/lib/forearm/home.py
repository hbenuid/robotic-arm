"""build123d builder of the wrist_pitch home sensor's clip (lib/forearm/params.py WristHomeParams;
parts/joints/wrist_home_clip): built in j2_link's part frame (the wrist axis along +Z through x = web.wrist_x), then
moved into its own (lib/forearm/layout.py home_clip_frame()). Imported explicitly by the part, so importing lib.forearm
never pulls in OCCT."""
from __future__ import annotations

import build123d as bd

from lib.datum import to_location
from lib.forearm.layout import (
    home_board_points,
    home_boss_z,
    home_clip_frame,
    home_end_x,
    home_plate_box,
    home_reliefs,
    home_sensor_frame,
    home_to_link,
)
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.geom import box, cylinder, faceted_end, hull_solid, single_solid, teardrop
from lib.units import NUDGE

_FAR = 200.0   # past the part: the span of a cutter only its other sides limit


def _ring_prism(cfg: ForearmConfig, apothem: float, z0: float, z1: float):
    """The boss's faceted end grown to `apothem` (lib/geom.py faceted_end, about the wrist axis), from z0 up to z1."""
    return bd.Pos(0.0, 0.0, z0) * bd.extrude(faceted_end(apothem, (cfg.web.wrist_x, 0.0)), amount=z1 - z0)


def build_home_clip(cfg: ForearmConfig = DEFAULT):
    """The clip, one solid in its part frame: the cap - the ring `gap` .. `gap` + `wall` round the boss's faceted end past
    cap_x, from the jaw's underside to top_clear under the boss's top -, the jaw under the boss from jaw_r out, the spine
    on down the end flat to under the board's plate, the web (the convex hull of the plate and the spine's foot, the
    plate's depth) with the board's plate in it; the plate's relief slots under the header's pins and the chip's leads;
    the board's 2 M2 holes through the plate, teardrops peaked toward +x (up as it prints, on its end-flat face)."""
    h = cfg.home
    under, top = home_boss_z(cfg)
    a = cfg.boss.dia / 2.0
    end = (cfg.web.wrist_x - _FAR, cfg.web.wrist_x - h.cap_x)    # the half the cap and the jaw keep: past cap_x, the end's side
    jaw = (under - h.gap - h.jaw_t, under - h.gap)
    cap = _ring_prism(cfg, a + h.gap + h.wall, jaw[0], top - h.top_clear) - _ring_prism(cfg, a + h.gap, jaw[0] - 1.0, top + 1.0)
    cap = cap & box(end, (-_FAR, _FAR), (-_FAR, _FAR))
    lip = _ring_prism(cfg, a + h.gap + h.wall, *jaw) - cylinder(h.jaw_r, jaw[1] - jaw[0] + 2.0, (cfg.web.wrist_x, 0.0), jaw[0] - 1.0)
    lip = lip & box(end, (-_FAR, _FAR), (-_FAR, _FAR))
    x0, x1 = home_end_x(cfg) - h.gap - h.wall, home_end_x(cfg) - h.gap
    px, py, pz = home_plate_box(cfg)
    foot_z = home_to_link(cfg, (0.0, 0.0, pz[0]))[2]             # the plate's underside: the web's and the spine's foot
    board_z = home_to_link(cfg, (0.0, 0.0, pz[1]))[2]            # the board's underside: the web's top
    spine = box((x0, x1), (-h.spine_w / 2.0, h.spine_w / 2.0), (foot_z, jaw[0]))
    plate = [home_to_link(cfg, (x, y, 0.0)) for x in px for y in py]
    web = [(p[0], p[1], wz) for p in plate for wz in (foot_z, board_z)]
    web += [(x, y, wz) for x in (x0, x1) for y in (-h.spine_w / 2.0, h.spine_w / 2.0) for wz in (foot_z, board_z)]
    body = cap + lip + spine + hull_solid(web)
    sensor = to_location(home_sensor_frame(cfg))
    for rx, ry, depth in home_reliefs(cfg):
        body = body - sensor * box(rx, ry, (-depth, NUDGE))
    for x, y in home_board_points(cfg):
        # the teardrop's peak (its +Y) along this frame's +x: the KY-003's -Y, up as the clip prints
        body = body - sensor * bd.Pos(x, y, 0.0) * bd.Rot(0.0, 0.0, 180.0) * teardrop(h.board_hole_dia / 2.0, h.plate_t, z0=pz[0])
    return single_solid(to_location(home_clip_frame(cfg)).inverse() * body)
