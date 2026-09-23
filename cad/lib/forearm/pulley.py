"""A printed GT2 pulley ring (the roll shaft's integral 90T), built from lib/belts.py's tooth constants."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.belts import GT2_GROOVE_R, GT2_TOOTH_DEPTH, pulley_od
from lib.cycloidal.geom import cylinder, single_solid
from lib.units import NUDGE


def groove_centre_radius(teeth: int) -> float:
    """Radius of the groove-bottom circle's centre: land radius - tooth depth + groove radius (90T: 28.2, the
    r0.555 arcs of the SolidWorks pulley sit at 28.2)."""
    return pulley_od(teeth) / 2.0 - GT2_TOOTH_DEPTH + GT2_GROOVE_R


def gt2_ring(teeth: int, width: float, flange_dia: float = 0.0, flange_t: float = 0.0, z0: float = 0.0,
             inner_dia: float = 0.0):
    """The toothed ring standing on z0: `teeth` grooves in a Ø pulley_od(teeth) land, `width` along Z, each groove
    a round bottom (GT2_GROOVE_R, GT2_TOOTH_DEPTH deep) with straight sides to the land; with `flange_dia` > 0 a
    flange of `flange_t` below (z0 - flange_t .. z0) and above (z0 + width ..) the teeth; with `inner_dia` > 0 an
    annulus (the ring fuses onto a core of that diameter - keep every later boolean away from the teeth)."""
    r_land = pulley_od(teeth) / 2.0
    r_c = groove_centre_radius(teeth)
    throat = r_land + 0.1 - r_c
    groove = bd.Pos(r_c, 0.0) * bd.Circle(GT2_GROOVE_R) + bd.Pos(r_c + throat / 2.0, 0.0) * bd.Rectangle(throat, 2.0 * GT2_GROOVE_R)
    profile = bd.Circle(r_land) - (bd.PolarLocations(0.0, teeth) * groove)
    if inner_dia > 0.0:
        profile = profile - bd.Circle(inner_dia / 2.0)
    ring = bd.Pos(0.0, 0.0, z0) * bd.extrude(profile, amount=width)
    if flange_dia > 0.0 and flange_t > 0.0:
        flange = cylinder(flange_dia / 2.0, flange_t, z0=z0 - flange_t) + cylinder(flange_dia / 2.0, flange_t, z0=z0 + width)
        if inner_dia > 0.0:
            flange = flange - cylinder(inner_dia / 2.0, width + 2 * flange_t + 2 * NUDGE, z0=z0 - flange_t - NUDGE)
        ring = ring + flange
    return single_solid(ring)
