"""The cycloidal disc - shared builder for parts/cycloidal/cycloidal_disc_1.py and cycloidal_disc_2.py.

Port of cycloidal_drive@2f1f67d src/cycloidal_disc.py. Disc 1 and disc 2 share the hole
pattern and chamfer but disc 2's epitrochoid is phase-rotated by ``cfg.gear.disc2_phase_deg``
(-9 deg) so its lobes mesh with the ring pins at its (-e, 0) orbit; the holes are NOT rotated.
Each disc's 6003 seat is z 0..thickness in its local frame; ``DiscParams.bearing_lip`` (DEFAULT_CONFIG) makes the disc
that much thicker on its OUTER face - below z 0 for disc 1 (the motor plate's side), above the seat for disc 2 (the
hub's) - with a lip there over the bearing's outer race (``bearing_lip_dia``), the seat blind from the inner face.
"""
from __future__ import annotations

import build123d as bd

from lib.cycloidal.layout import output_pin_points
from lib.cycloidal.params import DEFAULT_CONFIG, DriveConfig
from lib.cycloidal.profiles import profile_points
from lib.geom import cylinder, single_solid, through
from lib.units import NUDGE


def build_disc(cfg: DriveConfig = DEFAULT_CONFIG, phase_offset_deg: float = 0.0, lip_on_top: bool = False) -> bd.Solid:
    """One disc at its local origin (profile centred, its 6003's seat z 0..thickness). With a bearing lip the disc runs
    on past the seat by `bearing_lip` on its outer face: above it with `lip_on_top` (disc 2), below z 0 otherwise."""
    disc = cfg.disc
    lip = disc.bearing_lip
    z0 = -lip if lip > 0 and not lip_on_top else 0.0   # the disc's bottom face
    height = disc.thickness + lip
    points = profile_points(cfg, phase_offset_deg)

    # Periodic interpolating spline through every profile point (exact lobe peaks - never an
    # approximating spline), one closed edge -> face -> prism.
    edge = bd.Edge.make_spline([bd.Vector(x, y, z0) for x, y in points], periodic=True)
    solid = bd.Solid.extrude(bd.Face(bd.Wire([edge])), bd.Vector(0, 0, height))

    # Chamfer the outer lobe edges BEFORE the holes: the top/bottom faces then carry only the
    # epitrochoid boundary, so "all edges of that face" is exactly the outer edge.
    top = max(solid.faces().filter_by(bd.GeomType.PLANE), key=lambda f: f.center().Z)
    solid = solid.chamfer(disc.lobe_chamfer, None, top.edges())
    bottom = min(solid.faces().filter_by(bd.GeomType.PLANE), key=lambda f: f.center().Z)
    solid = solid.chamfer(disc.lobe_chamfer, None, bottom.edges())

    # Centre bore for the 6003 bearing - through (the port), or its seat blind from the inner face and the lip's hole
    # through the lip -, then the 4 output-pin clearance holes on the 60 mm circle.
    if lip > 0:
        seat_z0 = -NUDGE if lip_on_top else 0.0
        result = solid - cylinder(disc.center_bore_dia / 2.0, disc.thickness + NUDGE, z0=seat_z0)
        result = result - through(disc.bearing_lip_dia / 2.0, height, z0=z0)
    else:
        result = solid - through(disc.center_bore_dia / 2.0, disc.thickness)
    for xy in output_pin_points(cfg):
        result = result - through(disc.output_pin_hole_dia / 2.0, height, xy, z0)
    return single_solid(result)
