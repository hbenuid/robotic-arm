"""build123d builder of the wrist_roll home sensor's fin (lib/wrist/params.py WristRollHomeParams;
parts/wrist/wrist_roll_home_fin), in wrist_roll_drive's module frame: the upright plate the KY-003 stands on beside the
pulley's face, its back flush with the far face, and the shoe under it on the front facet and the front face
(lib/wrist/layout.py home_*). Imported explicitly by the part, so importing lib.wrist never pulls in OCCT."""
from __future__ import annotations

import math

import build123d as bd

from lib.datum import to_location
from lib.geom import box, cylinder, hex_prism, single_solid
from lib.units import NUDGE
from lib.wrist.layout import (
    home_board_points,
    home_face_z,
    home_fin_back,
    home_fin_t,
    home_reliefs,
    home_sensor_frame,
    stack_positions,
)
from lib.wrist.params import DEFAULT, WristConfig

_FAR = 200.0   # past the part: the span of a cutter only its other sides limit


def _half_space(point, normal):
    """The half-space n . (p - point) >= 0 as a solid."""
    n = bd.Vector(normal).normalized()
    return bd.Plane(origin=point, z_dir=n) * bd.Box(4 * _FAR, 4 * _FAR, 2 * _FAR,
                                                    align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def frame_keepout(cfg: WristConfig = DEFAULT):
    """The frame round its far face's front corner, `gap` bigger all round: under the front face and under the front
    facet (its 45 deg plane from the far face to the front face) - what the fin's shoe is cut by, so it sits on them
    with the glue's gap."""
    d, h, S = cfg.roll, cfg.roll.home, stack_positions(cfg)
    facet = d.frame_r + S["z_front"] - d.front_facet                    # the facet's plane: x + z = facet
    body = box((-_FAR, _FAR), (-_FAR, _FAR), (-_FAR, S["z_front"] + h.gap))
    return body & _half_space((0.0, 0.0, facet + h.gap * math.sqrt(2.0)), (-1.0, 0.0, -1.0))


def build_home_fin(cfg: WristConfig = DEFAULT):
    """The fin, one solid in the module frame: the upright plate from the board's back to the far face (plate_x along the
    KY-003, plate_w across it), from the facet up; the shoe from shoe_x0 to the far face under the plate's lower edge, down
    onto the facet and the front face - both `gap` off; the board's 2 M2 holes
    through it, a nut's hex pocket in its back at each; its slots under the header's pins' tails and the chip's leads'."""
    d, h, S = cfg.roll, cfg.roll.home, stack_positions(cfg)
    sensor = to_location(home_sensor_frame(cfg))
    (bx, by, _bz), _rot = home_sensor_frame(cfg)
    back, t = home_fin_back(cfg), home_fin_t(cfg)
    zc = home_face_z(cfg)
    z0, z1 = zc - h.plate_w / 2.0, zc + h.plate_w / 2.0                 # the KY-003's y along -Z
    y0, y1 = by + h.plate_x[0], by + h.plate_x[1]                       # its x along +Y
    floor = S["z_front"] - d.front_facet - 1.0                          # under the facet: the keepout trims it
    body = box((bx, back), (y0, y1), (floor, z1)) + box((h.shoe_x0, back), (y0, y1), (floor, z0))
    body = body - frame_keepout(cfg)
    for x, y in home_board_points(cfg):
        body = body - sensor * cylinder(h.board_hole_dia / 2.0, t + 2.0, (x, y), z0=-t - 1.0)
        body = body - sensor * bd.Pos(x, y, -t - 1.0) * hex_prism(h.nut_pocket_af, math.radians(90.0), h.nut_pocket_depth + 1.0)
    for rx, ry, depth in home_reliefs(cfg):
        body = body - sensor * box(rx, ry, (-depth, NUDGE))
    return single_solid(max(body.solids(), key=lambda s: s.volume))
