"""build123d builder of gripper_j3_connector (ConnectorParams) in its SolidWorks part frame: the plate in the x-z plane,
y 0 .. plate_t, its +Y face toward the wrist roll's output. LEGACY: the plate and the boss, two bodies as SolidWorks
modelled them, the boss's 20-tooth bore (and its recess into the plate) as the circle of its area. DEFAULT: one body -
the plate and the hub bolted to the wrist roll pulley's face (lib/wrist/roll.py), the cable bore through both."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.geom import align_min, single_solid
from lib.units import NUDGE
from lib.wrist.layout import connector_bolt_points
from lib.wrist.params import DEFAULT, WristConfig


def _y_cylinder(radius: float, y0: float, y1: float, xz=(0.0, 0.0)):
    """A cylinder along +Y from y0 to y1, its axis through (x, z) = xz."""
    return bd.Pos(xz[0], y0, xz[1]) * bd.Rot(-90.0, 0.0, 0.0) * bd.Cylinder(radius, y1 - y0, align=align_min())


def _plate(cfg: WristConfig):
    """The plate: the Ø disc_dia disc cut to its flats (x +/- half_width), the servo holder's and the cover's 4 holes."""
    c = cfg.connector
    disc = _y_cylinder(c.disc_dia / 2.0, 0.0, c.plate_t)
    flats = bd.Pos(0.0, c.plate_t / 2.0, 0.0) * bd.Box(2.0 * c.half_width, c.plate_t + 2 * NUDGE, c.disc_dia + 2.0)
    plate = disc & flats
    for x, z, dia in c.holes:
        plate = plate - _y_cylinder(dia / 2.0, -NUDGE, c.plate_t + NUDGE, (x, z))
    return plate


def build_connector(cfg: WristConfig = DEFAULT):
    c = cfg.connector
    if not c.hub:
        plate = _plate(cfg) - _y_cylinder(c.bore_r_eq, c.plate_t - c.recess_depth, c.plate_t + NUDGE)
        boss = _y_cylinder(c.boss_dia / 2.0, c.plate_t, c.boss_y1) - _y_cylinder(c.bore_r_eq, c.plate_t - NUDGE, c.boss_y1 + NUDGE)
        return bd.Compound([single_solid(plate), single_solid(boss)])
    y_flange = c.plate_t + c.hub_head_room
    y_end = c.plate_t + c.hub_len
    body = _plate(cfg) + _y_cylinder(c.hub_core_dia / 2.0, c.plate_t - NUDGE, y_flange + NUDGE)
    body = body + _y_cylinder(c.hub_flange_dia / 2.0, y_flange, y_end)
    body = body - _y_cylinder(c.hub_bore / 2.0, -NUDGE, y_end + NUDGE)
    # the 4x M3 through the flange (their heads between it and the plate); a notch in the flange's rim over each pair of
    # the plate's holes (a slot round both: one hole's notch alone would leave a sliver between them), so a key reaches
    # the servo holder's and the cover's screws past it
    for x, z in connector_bolt_points(cfg):
        body = body - _y_cylinder(cfg.roll.bolt_dia / 2.0, y_flange - NUDGE, y_end + NUDGE, (x, z))
    for z in sorted({hz for _, hz, _ in c.holes}):
        xs = [hx for hx, hz, _ in c.holes if hz == z]
        for x in xs:
            body = body - _y_cylinder(c.notch_r, y_flange - NUDGE, y_end + NUDGE, (x, z))
        body = body - bd.Pos((min(xs) + max(xs)) / 2.0, (y_flange + y_end) / 2.0, z) * bd.Box(
            max(xs) - min(xs), y_end - y_flange + 2 * NUDGE, 2.0 * c.notch_r)
    return single_solid(body)
