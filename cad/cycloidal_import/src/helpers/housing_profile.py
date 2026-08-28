"""Shared outer profile for housing parts (motor plate + ring gear body).

Both parts present the same outer silhouette: 8 trapezoidal pillars (one per
M4 housing bolt) connected only at the inner annulus, with reveal windows
between them.  This module builds the wall-removal cutter — subtract it from
any annular housing solid to imprint the shared profile.

Pillar geometry (radially flipped trapezoid):
  - Inner edge (at bore radius):    18mm tangential width
  - Outer edge (at OD):             10mm tangential width
  - Radial bounds overshoot bore (-1mm) and OD (+1mm) so the bore and base
    cylinder cuts trim each pillar flush with the housing walls.
"""

import math
import sys
import os

import cadquery as cq

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DriveConfig, compute_housing_bolt_angles


def build_reveal_window_cutter(
    cfg: DriveConfig,
    height: float,
    z_offset: float = 0.0,
) -> cq.Workplane:
    """Wall-removal cutter that leaves 8 trapezoidal pillars at the bolt angles.

    Subtract this from any annular housing part (motor plate, ring gear body)
    to produce the shared outer profile.  The annulus inside ``pillar_inner_r``
    (57mm) is untouched — bearing seats and retention shoulders are preserved.

    Args:
        cfg:       Drive configuration (used for housing OD, bore, bolt angles).
        height:    Axial extent of the cutter (along Z).
        z_offset:  Z position of the cutter's bottom face.  Default 0.

    Returns:
        A CadQuery Workplane representing the volume to subtract.
    """
    h = cfg.housing
    housing_r = h.od / 2.0  # 70mm (140mm OD)
    bore_r = h.bore_dia / 2.0  # 58mm

    pillar_inner_r = bore_r - 1.0  # 57mm
    pillar_outer_r = housing_r + 1.0  # 71mm
    pillar_inner_w = 18.0  # mm, tangential width at bore
    pillar_outer_w = 10.0  # mm, tangential width at housing OD

    # Full annular wall section to remove (extends 0.1mm past OD so the base
    # cylinder trims the cutter flush at the housing OD).
    cutter = (
        cq.Workplane("XY")
        .workplane(offset=z_offset)
        .circle(housing_r + 0.1)
        .circle(bore_r)
        .extrude(height)
    )

    # Subtract trapezoidal pillars (preserved material) from the cutter.
    for a in compute_housing_bolt_angles(cfg):
        cos_a = math.cos(a)
        sin_a = math.sin(a)
        local_pts = [
            (pillar_inner_r, -pillar_inner_w / 2.0),
            (pillar_outer_r, -pillar_outer_w / 2.0),
            (pillar_outer_r, +pillar_outer_w / 2.0),
            (pillar_inner_r, +pillar_inner_w / 2.0),
        ]
        world_pts = [
            (lx * cos_a - ly * sin_a, lx * sin_a + ly * cos_a)
            for lx, ly in local_pts
        ]
        pillar = (
            cq.Workplane("XY")
            .workplane(offset=z_offset)
            .polyline(world_pts)
            .close()
            .extrude(height)
        )
        cutter = cutter.cut(pillar)

    return cutter


def chamfer_outer_silhouette(result, cfg, external_face=None):
    """Bevel the outer silhouette of a finished housing part.

    Always chamfers the 8 pillar outer vertical corners (the full-height "barrel"
    edges, present on every part).  When ``external_face`` is given (a CadQuery face
    selector for the part's externally-facing end face, e.g. ``"<Z"`` or ``">Z"``),
    the *entire outer perimeter* of that face is also chamfered — the complete
    8-pillar / 8-window silhouette (outer arcs, the inner arcs between pillars, and
    the pillar side edges), taken from the face's outer wire so internal holes are
    excluded.

    Mating end faces, internal holes, and (on parts whose ends both mate, e.g. the
    ring gear body) the window cut-outs are left sharp.  Call AFTER the reveal
    windows are cut — the pillar corners and perimeter only exist once the windows
    and central features are present.

    Args:
        result:         CadQuery Workplane holding the finished single solid.
        cfg:            Drive configuration (housing OD + ``edge_chamfer``).
        external_face:  Face selector (``"<Z"`` / ``">Z"``) for the external end
                        face whose full perimeter should be beveled, or None if both
                        end faces are mating (only the barrel verticals are beveled).

    Returns:
        The Workplane with the outer silhouette chamfered (unchanged if
        ``edge_chamfer`` <= 0).
    """
    ch = cfg.housing.edge_chamfer
    if ch <= 0:
        return result

    rmin = cfg.housing.od / 2.0 - 1.0  # ~69mm: pillar outer corners, skips r≤66 bolt/counterbore artifacts
    sel = []
    # Pillar outer vertical corners (every part) — the barrel silhouette.
    for e in result.val().Edges():
        p0, p1 = e.startPoint(), e.endPoint()
        rm = math.hypot((p0.x + p1.x) / 2.0, (p0.y + p1.y) / 2.0)
        if (
            rm >= rmin
            and e.geomType() == "LINE"
            and abs(p1.z - p0.z) > 1e-6
            and math.hypot(p0.x - p1.x, p0.y - p1.y) < 1e-6
        ):
            sel.append(e)  # pillar outer vertical corner

    # Full outer perimeter of the external end face (outer wire excludes holes).
    if external_face is not None:
        sel += result.faces(external_face).val().outerWire().Edges()

    return result.newObject(sel).chamfer(ch) if sel else result
