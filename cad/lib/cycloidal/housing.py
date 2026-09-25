"""Shared outer profile of the two housing parts (motor plate + ring gear body).

Port of cycloidal_drive@2f1f67d src/helpers/housing_profile.py. Both parts present the same
silhouette: 8 trapezoidal pillars (one per M4 housing bolt) joined only at the inner annulus,
with reveal windows between them. ``reveal_window_cutter`` is subtracted from an annular housing
solid to imprint that profile; ``chamfer_outer_silhouette`` bevels the result's outer edges.

Pillar (radially flipped trapezoid): ``pillar_inner_w`` (18) tangential at the bore,
``pillar_outer_w`` (10) at the OD; the radial bounds overshoot the bore (-1) and the OD (+1) so
the bore / base-cylinder cuts trim every pillar flush.
"""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.cycloidal.params import DEFAULT_CONFIG, DriveConfig, compute_housing_bolt_angles
from lib.geom import align_min, hex_prism, single_solid

PILLAR_OVERSHOOT = 1.0    # [DESIGN] pillars past the bore (-) and OD (+); trimmed flush by the bore/OD cuts
CUTTER_OVERSHOOT = 0.1    # [DESIGN] cutter annulus past the OD so the base cylinder trims it flush
BARREL_EDGE_MARGIN = 1.0  # [DESIGN] barrel corners are the vertical edges at r >= od/2 - this (skips r<=66 bolt artefacts)


def reveal_window_cutter(cfg: DriveConfig = DEFAULT_CONFIG, height: float = 0.0, z_offset: float = 0.0):
    """Wall-removal volume leaving 8 trapezoidal pillars at the bolt angles: 8 disjoint solids
    spanning z_offset..z_offset+height. The annulus inside the bore is untouched."""
    h = cfg.housing
    housing_r, bore_r = h.od / 2.0, h.bore_dia / 2.0
    inner_r, outer_r = bore_r - PILLAR_OVERSHOOT, housing_r + PILLAR_OVERSHOOT
    cutter = bd.Cylinder(housing_r + CUTTER_OVERSHOOT, height, align=align_min()) - bd.Cylinder(bore_r, height, align=align_min())
    for a in compute_housing_bolt_angles(cfg):
        c, s = math.cos(a), math.sin(a)
        local = [
            (inner_r, -h.pillar_inner_w / 2.0),
            (outer_r, -h.pillar_outer_w / 2.0),
            (outer_r, +h.pillar_outer_w / 2.0),
            (inner_r, +h.pillar_inner_w / 2.0),
        ]
        pts = [(lx * c - ly * s, lx * s + ly * c) for lx, ly in local]
        pillar = bd.extrude(bd.make_face(bd.Polyline(*pts, close=True)), amount=height, dir=(0, 0, 1))
        cutter = cutter - pillar
    return bd.Pos(0, 0, z_offset) * cutter


def end_face(solid: bd.Shape, z: float, tol: float = 1e-4) -> bd.Face:
    """The largest planar face lying in the plane z (normal along +/-Z)."""
    faces = [
        f for f in solid.faces().filter_by(bd.GeomType.PLANE)
        if abs(f.center().Z - z) <= tol and abs(abs(f.normal_at().Z) - 1.0) <= 1e-6
    ]
    if not faces:
        raise ValueError(f"no planar face at z={z}")
    return max(faces, key=lambda f: f.area)


def chamfer_outer_silhouette(shape: bd.Shape, cfg: DriveConfig = DEFAULT_CONFIG, external_z: float | None = None) -> bd.Solid:
    """Bevel the outer silhouette of a finished housing part (call AFTER the windows are cut).

    Always chamfers the 8 pillar outer vertical corners (the barrel edges). When ``external_z``
    is the height of the part's externally facing end face, that face's whole outer perimeter
    (outer arcs, window arcs, pillar sides - the outer wire, so internal holes are excluded) is
    chamfered too. Mating faces stay sharp. Unchanged when ``edge_chamfer <= 0``."""
    solid = single_solid(shape)
    ch = cfg.housing.edge_chamfer
    if ch <= 0:
        return solid
    rmin = cfg.housing.od / 2.0 - BARREL_EDGE_MARGIN
    sel = []
    for e in solid.edges():
        if e.geom_type != bd.GeomType.LINE:
            continue
        p0, p1 = e.position_at(0.0), e.position_at(1.0)
        vertical = abs(p1.Z - p0.Z) > 1e-6 and math.hypot(p0.X - p1.X, p0.Y - p1.Y) < 1e-6
        if vertical and math.hypot((p0.X + p1.X) / 2.0, (p0.Y + p1.Y) / 2.0) >= rmin:
            sel.append(e)
    if external_z is not None:
        sel += list(end_face(solid, external_z).outer_wire().edges())
    return solid.chamfer(ch, None, sel) if sel else solid


def hex_pocket(cfg: DriveConfig, xy, angle_rad: float, depth: float, z0: float = 0.0):
    """Captive M4 nut pocket (pocket AF) at xy, from z0 up by ``depth``, hex keyed along ``angle_rad``."""
    return bd.Pos(xy[0], xy[1], z0) * hex_prism(cfg.housing.bolt_nut_pocket_af, angle_rad, depth)
