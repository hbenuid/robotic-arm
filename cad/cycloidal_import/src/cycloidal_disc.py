"""Cycloidal disc model — the core part of the drive.

Disc 1 and disc 2 share the same hole pattern and chamfered profile shape
but use different ``phase_offset_deg`` values so the profiles mesh correctly
with the ring pins at their respective orbit positions (+e, 0) and (-e, 0).
Disc 1 uses 0°; disc 2 uses ``cfg.gear.disc2_phase_deg = -180°/N_lobes``.
"""

import math
import sys
import os
import cadquery as cq

# Allow running directly or as a module
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DriveConfig, DEFAULT_CONFIG
from src.profiles import compute_epitrochoid


def build_cycloidal_disc(
    cfg: DriveConfig = DEFAULT_CONFIG,
    phase_offset_deg: float = 0.0,
) -> cq.Workplane:
    """Build a single cycloidal disc with center bore and output pin holes.

    ``phase_offset_deg`` rotates the epitrochoid profile around the disc
    center without rotating the output-pin holes. Disc 1 uses 0°; disc 2
    uses ``cfg.gear.disc2_phase_deg`` so its lobes mesh correctly with the
    ring pins when placed at orbit (-e, 0).
    """
    gear = cfg.gear
    disc = cfg.disc
    profile_cfg = cfg.profile

    # Generate epitrochoid profile points
    points = compute_epitrochoid(
        R=gear.ring_pin_circle_radius,
        r=gear.ring_pin_radius,
        N=gear.num_ring_pins,
        e=gear.eccentricity,
        num_points=profile_cfg.num_points,
    )

    if phase_offset_deg != 0.0:
        a = math.radians(phase_offset_deg)
        cos_a, sin_a = math.cos(a), math.sin(a)
        points = [
            (cos_a * x - sin_a * y, sin_a * x + cos_a * y)
            for (x, y) in points
        ]

    # Build disc from spline profile using OCC-level periodic spline
    # for exact interpolation of lobe peaks
    pts_3d = [cq.Vector(x, y, 0) for x, y in points]
    edge = cq.Edge.makeSpline(pts_3d, periodic=True)
    wire = cq.Wire.assembleEdges([edge])
    face = cq.Face.makeFromWires(wire)
    solid = cq.Solid.extrudeLinear(face, cq.Vector(0, 0, disc.thickness))
    result = cq.Workplane("XY").add(solid)

    # Chamfer outer lobe edges before holes — top+bottom faces only carry the
    # epitrochoid boundary at this point, so the selector is unambiguous.
    result = result.faces(">Z").edges().chamfer(disc.lobe_chamfer)
    result = result.faces("<Z").edges().chamfer(disc.lobe_chamfer)

    # Center bore for 6003 bearing
    result = result.faces(">Z").workplane().hole(disc.center_bore_dia)

    # Output pin holes: 4x equally spaced on 60mm circle
    pin_hole_r = disc.output_pin_circle_dia / 2.0
    pin_positions = [
        (
            pin_hole_r * math.cos(math.radians(i * 360 / disc.output_pin_count)),
            pin_hole_r * math.sin(math.radians(i * 360 / disc.output_pin_count)),
        )
        for i in range(disc.output_pin_count)
    ]
    result = (
        result.faces(">Z")
        .workplane()
        .pushPoints(pin_positions)
        .hole(disc.output_pin_hole_dia)
    )

    return result


if __name__ == "__main__":
    from ocp_vscode import show_object

    show_object(build_cycloidal_disc(), name="disc_1")
    show_object(
        build_cycloidal_disc(phase_offset_deg=DEFAULT_CONFIG.gear.disc2_phase_deg),
        name="disc_2",
    )
