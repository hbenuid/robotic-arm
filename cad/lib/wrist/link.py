"""build123d builder of wrist_link from WristConfig, in the wrist body's part frame.

Built in the part frame directly: the plate, the tower's wall and the seat's holes stand along Z, the slope is a prism
along Y (its outline in the XZ plane), the end face's holes run along X. The fillet on the slope's edge with the bore
is made on a tall core - the slope and the bore carried far above and below the tower - and the core trimmed to the
tower after: on the tower itself the edge ends where the slope touches the bore at the top (a point), which a fillet
cannot run through."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.geom import align_min, cylinder, hex_prism, single_solid, through
from lib.units import NUDGE
from lib.wrist.layout import end_face_holes, seat_bolt_points, slope_x
from lib.wrist.params import DEFAULT, WristConfig

_FAR = 200.0         # beyond the part: the span of a cutter or a core that only one side limits
_CORE_BELOW = 5.0    # the fillet's core, under the plate's underside ...
_CORE_ABOVE = 20.0   # ... and over the tower's top


def _box(x: tuple, y: tuple, z: tuple):
    """A box spanning the ranges x = (x0, x1), y = (y0, y1), z = (z0, z1)."""
    (x0, x1), (y0, y1), (z0, z1) = x, y, z
    return bd.Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0, (z0 + z1) / 2.0) * bd.Box(x1 - x0, y1 - y0, z1 - z0)


def _xcyl(radius: float, x0: float, x1: float, y: float, z: float):
    """A cylinder along X from x0 to x1, its axis through (y, z)."""
    return bd.Pos(x0, y, z) * bd.Rot(0.0, 90.0, 0.0) * bd.Cylinder(radius, x1 - x0, align=align_min())


def _behind_slope(cfg: WristConfig, z0: float, z1: float):
    """Everything behind the slope (x <= slope_x(z)) from z0 to z1: a prism along Y."""
    outline = [(-_FAR, z0), (slope_x(z0, cfg), z0), (slope_x(z1, cfg), z1), (-_FAR, z1)]
    return bd.extrude(bd.Plane.XZ * bd.Polygon(*outline, align=None), amount=_FAR, both=True)


def _bore(cfg: WristConfig, z0: float, z1: float):
    """The bore from z0 to z1, its seam turned to -X (off the slope's edge)."""
    return bd.Rot(0.0, 0.0, 180.0) * cylinder(cfg.tower.bore_r, z1 - z0, z0=z0)


def _plate(cfg: WristConfig):
    """The plate: a round end on each axis, clipped to the flats and at the end face (the round end's r wall_r reaches
    past a block shorter than it)."""
    p, t = cfg.plate, cfg.tower
    plate = cylinder(p.pitch_r, p.thickness, (p.pitch_x, 0.0)) + cylinder(t.wall_r, p.thickness)
    plate = plate + _box((p.pitch_x, 0.0), (-p.half_width, p.half_width), (0.0, p.thickness))
    return plate & _box((-_FAR, t.block_x1), (-p.half_width, p.half_width), (-NUDGE, p.thickness + NUDGE))


def _tower(cfg: WristConfig):
    """The wall and the block, less the bore behind the slope, the slope's edge with the bore filleted."""
    t = cfg.tower
    # the core: from _CORE_BELOW under the plate (lower, the slope meets the bore's back too steeply for the fillet)
    # to _CORE_ABOVE over the tower - the slope's edge runs out through its underside, over the top unbroken
    z0, z1 = -_CORE_BELOW, t.z1 + _CORE_ABOVE
    core = _box((t.x0 - _FAR / 4.0, t.block_x1 + 1.0), (-_FAR / 4.0, _FAR / 4.0), (z0, z1))
    core = core - (_bore(cfg, z0 - 1.0, z1 + 1.0) & _behind_slope(cfg, z0 - 1.0, z1 + 1.0))
    n = bd.Vector(-(t.z1 - cfg.plate.thickness), 0.0, t.slope_x1 - t.slope_x0).normalized()
    slope = max(core.faces().filter_by(bd.GeomType.PLANE), key=lambda f: f.normal_at().dot(n))
    edges = [e for e in slope.edges() if e.geom_type != bd.GeomType.LINE]
    core = core.fillet(t.fillet_r, edges)
    wall = cylinder(t.wall_r, t.z1) & _box((t.x0, t.block_x1), (-_FAR, _FAR), (-_FAR, _FAR))   # ends at the end face
    block = _box((t.x0, t.block_x1), (-t.block_half_width, t.block_half_width), t.block_z)
    return core & (wall + block)


def _cheeks(cfg: WristConfig):
    """The cheeks either side of the slot, inside the bore (and the block), from the plate's top up to cheek_z1."""
    t, z0 = cfg.tower, cfg.plate.thickness
    return [_box((t.cheek_x, min(t.bore_r, t.block_x1)), y, (z0, t.cheek_z1))
            for y in ((t.slot_half, t.bore_r), (-t.bore_r, -t.slot_half))]


def _back(cfg: WristConfig):
    """The back of the bore behind the cheeks' front faces, down to the slope: out to back_y on +Y, the bore on -Y."""
    t, z0 = cfg.tower, cfg.plate.thickness
    back = _bore(cfg, z0, t.z1 + NUDGE) + _box((t.x0 - 1.0, t.cheek_x), (0.0, t.back_y), (z0, t.z1 + NUDGE))
    return back & _box((-_FAR, t.cheek_x), (-_FAR, _FAR), (-_FAR, _FAR)) & _behind_slope(cfg, z0, t.z1 + NUDGE)


def _seat_cuts(cfg: WristConfig):
    """The through hole on the pitch axis, the flange's 4 bolt holes over their hex nut pockets."""
    p, s = cfg.plate, cfg.seat
    cuts = [through(s.hole_dia / 2.0, p.thickness, (p.pitch_x, 0.0))]
    for x, y, a in seat_bolt_points(cfg):
        cuts.append(cylinder(s.bolt_dia / 2.0, p.thickness - s.nut_depth + 2.0 * NUDGE, (x, y), s.nut_depth - NUDGE))
        # a flat toward the pitch axis: the corners at a +/- 30 degrees
        cuts.append(bd.Pos(x, y, -NUDGE) * hex_prism(s.nut_af, math.radians(a + 30.0), s.nut_depth + NUDGE))
    return cuts


def _end_face_cuts(cfg: WristConfig):
    """The end face's M3 and M4, along X through the whole part."""
    e, x1 = cfg.end_face, cfg.tower.block_x1 + NUDGE
    x0 = cfg.plate.pitch_x - cfg.plate.pitch_r - 1.0
    m3, m4 = end_face_holes(cfg)
    return ([_xcyl(e.m3_dia / 2.0, x0, x1, y, z) for y, z in m3]
            + [_xcyl(e.m4_dia / 2.0, x0, x1, y, z) for y, z in m4])


def build_wrist_link(cfg: WristConfig = DEFAULT):
    body = _plate(cfg) + _tower(cfg)
    for cheek in _cheeks(cfg):
        body = body + cheek
    body = body - _back(cfg)
    for cut in _seat_cuts(cfg) + _end_face_cuts(cfg):
        body = body - cut
    return single_solid(body)
