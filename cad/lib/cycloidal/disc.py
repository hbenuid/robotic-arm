"""The cycloidal disc - shared builder for parts/cycloidal_disc_1.py and cycloidal_disc_2.py.

Port of cycloidal_drive@2f1f67d src/cycloidal_disc.py. Disc 1 and disc 2 share the hole
pattern and chamfer but disc 2's epitrochoid is phase-rotated by ``cfg.gear.disc2_phase_deg``
(-9 deg) so its lobes mesh with the ring pins at its (-e, 0) orbit; the holes are NOT rotated.
"""
from __future__ import annotations

from cadgen import build123d as bd

from lib.cycloidal.geom import single_solid, through
from lib.cycloidal.layout import output_pin_points
from lib.cycloidal.params import DEFAULT_CONFIG, DriveConfig
from lib.cycloidal.profiles import profile_points


def build_disc(cfg: DriveConfig = DEFAULT_CONFIG, phase_offset_deg: float = 0.0) -> bd.Solid:
    """One disc at its local origin (profile centred, z 0..thickness)."""
    disc = cfg.disc
    points = profile_points(cfg, phase_offset_deg)

    # Periodic interpolating spline through every profile point (exact lobe peaks - never an
    # approximating spline), one closed edge -> face -> prism.
    edge = bd.Edge.make_spline([bd.Vector(x, y, 0.0) for x, y in points], periodic=True)
    solid = bd.Solid.extrude(bd.Face(bd.Wire([edge])), bd.Vector(0, 0, disc.thickness))

    # Chamfer the outer lobe edges BEFORE the holes: the top/bottom faces then carry only the
    # epitrochoid boundary, so "all edges of that face" is exactly the outer edge.
    top = max(solid.faces().filter_by(bd.GeomType.PLANE), key=lambda f: f.center().Z)
    solid = solid.chamfer(disc.lobe_chamfer, None, top.edges())
    bottom = min(solid.faces().filter_by(bd.GeomType.PLANE), key=lambda f: f.center().Z)
    solid = solid.chamfer(disc.lobe_chamfer, None, bottom.edges())

    # Centre bore for the 6003 bearing, then the 4 output-pin clearance holes on the 60 mm circle.
    result = solid - through(disc.center_bore_dia / 2.0, disc.thickness)
    for xy in output_pin_points(cfg):
        result = result - through(disc.output_pin_hole_dia / 2.0, disc.thickness, xy)
    return single_solid(result)
