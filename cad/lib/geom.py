"""Small build123d helpers shared by the arm's builders - the drive, the links, the purchased parts (algebra mode)."""
from __future__ import annotations

import math

import build123d as bd

from lib import style
from lib.units import NUDGE  # not lib.params: that module imports lib.cycloidal.params (no import back up)


def align_min():
    """`align=` for a body standing on z0: footprint centred, extends +Z. A function, not a constant:
    an Align at module level would import the kernel with this module."""
    return (bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN)


def cylinder(radius: float, height: float, xy=(0.0, 0.0), z0: float = 0.0):
    """A cylinder standing on z0 (exact height - use for blind holes and bodies)."""
    return bd.Pos(xy[0], xy[1], z0) * bd.Cylinder(radius, height, align=align_min())


def through(radius: float, height: float, xy=(0.0, 0.0), z0: float = 0.0):
    """A through-hole cutter spanning z0..z0+height with NUDGE overshoot on both ends."""
    return cylinder(radius, height + 2 * NUDGE, xy, z0 - NUDGE)


def single_solid(shape: bd.Shape) -> bd.Solid:
    """The one Solid of an algebra-mode result (raises if a cutter leaked or the body split)."""
    solids = shape.solids()
    if len(solids) != 1:
        raise ValueError(f"expected exactly one solid, got {len(solids)}")
    return solids[0]


def faceted_end(apothem: float, centre=(0.0, 0.0), sides: int = style.SIDES):
    """The design language's faceted end (lib/style.py SIDES): a regular polygon about `centre` in the XY plane, a flat
    on each axis `apothem` off it - its corners apothem / cos(180 deg / sides) out."""
    return bd.Pos(centre[0], centre[1]) * bd.RegularPolygon(apothem, sides, major_radius=False, rotation=180.0 / sides)


def chamfer_outline(prism, axis: tuple, length: float = style.EDGE_CHAMFER) -> bd.Solid:
    """`prism` - one solid, its two end faces the extremes along `axis` - with the outline edges of both end faces
    chamfered `length` at 45 deg (lib/style.py EDGE_CHAMFER): the design language's edge, cut on a part's main body
    before its holes and pockets so that they stay sharp."""
    solid = single_solid(prism)
    faces = solid.faces().sort_by(bd.Axis((0.0, 0.0, 0.0), axis))
    return solid.chamfer(length, None, list(faces[0].outer_wire().edges()) + list(faces[-1].outer_wire().edges()))


def hull_faces(points, tol: float = 1e-6) -> list[list[tuple[float, float, float]]]:
    """The faces of the convex hull of a few 3D points (pure math, no kernel): each face the list of its corners, in
    order counter-clockwise seen from outside. Coplanar points merge into one face and a point inside a face or on an
    edge is no corner, so neighbouring faces share their edges exactly. O(n^4): for the dozen-odd corners of a faceted
    transition (lib/yaw_coupler/body.py), not for meshes."""
    pts = [tuple(float(c) for c in p) for p in points]

    def sub(a, b):
        return (a[0] - b[0], a[1] - b[1], a[2] - b[2])

    def cross(a, b):
        return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])

    def dot(a, b):
        return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]

    planes: list[tuple[tuple[float, float, float], float]] = []
    n_pts = len(pts)
    for i in range(n_pts):
        for j in range(i + 1, n_pts):
            for k in range(j + 1, n_pts):
                n = cross(sub(pts[j], pts[i]), sub(pts[k], pts[i]))
                length = math.sqrt(dot(n, n))
                if length < tol:
                    continue
                n = (n[0] / length, n[1] / length, n[2] / length)
                d = dot(n, pts[i])
                side = [dot(n, p) - d for p in pts]
                if max(side) > tol:
                    if min(side) < -tol:
                        continue                         # points on both sides: no face
                    n, d = (-n[0], -n[1], -n[2]), -d     # every point behind -n: the outward normal is -n
                if not any(dot(n, m) > 1.0 - tol and abs(d - e) < tol for m, e in planes):
                    planes.append((n, d))
    faces = []
    for n, d in planes:
        on = [p for p in pts if abs(dot(n, p) - d) < tol]
        # the face's 2D convex hull (monotone chain, collinear points dropped) in a frame (u, v) with u x v = n
        u = sub(on[1], on[0]) if dot(sub(on[1], on[0]), sub(on[1], on[0])) > tol else sub(on[2], on[0])
        u_len = math.sqrt(dot(u, u))
        u = (u[0] / u_len, u[1] / u_len, u[2] / u_len)
        v = cross(n, u)
        flat = sorted({(round(dot(p, u), 9), round(dot(p, v), 9)): p for p in on}.items())

        def turn(o, a, b):
            return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

        lower, upper = [], []
        for chain, items in ((lower, flat), (upper, flat[::-1])):
            for uv, p in items:
                while len(chain) >= 2 and turn(chain[-2][0], chain[-1][0], uv) <= tol:
                    chain.pop()
                chain.append((uv, p))
        ring = [p for _, p in lower[:-1] + upper[:-1]]
        if len(ring) >= 3:
            faces.append(ring)
    return faces


def hull_solid(points) -> bd.Solid:
    """The convex hull of a few 3D points as a solid of planar faces (hull_faces)."""
    faces = [bd.Face(bd.Wire.make_polygon(ring, close=True)) for ring in hull_faces(points)]
    solid = bd.Solid(bd.Shell(faces))
    if not solid.is_valid:
        raise ValueError("the hull's faces did not close into a valid solid")
    return solid


def hex_circumdiameter(across_flats: float) -> float:
    """Across-corners diameter of a hexagon from its across-flats size."""
    return across_flats / math.cos(math.radians(30))


def hex_prism(across_flats: float, angle_rad: float, height: float):
    """A hexagonal prism standing on z=0, first vertex on +X rotated by ``angle_rad`` (the
    CadQuery ``polygon(6, d)`` + ``transformed(rotate=...)`` convention)."""
    r = hex_circumdiameter(across_flats) / 2.0
    return bd.extrude(bd.RegularPolygon(r, 6, rotation=math.degrees(angle_rad)), amount=height, dir=(0, 0, 1))


# --- the belt drives' frames and rotors (lib/forearm/roll.py, lib/wrist/roll.py): a drive's axis is its frame's Z ---

def box(x, y, z):
    """A plain box over the x / y / z ranges ((x0, x1), ...)."""
    return bd.Pos(x[0], y[0], z[0]) * bd.Box(x[1] - x[0], y[1] - y[0], z[1] - z[0], align=(bd.Align.MIN, bd.Align.MIN, bd.Align.MIN))


def stop_wedge(r_in: float, r_out: float, z0: float, height: float, deg_width: float, at_deg: float):
    """A hard-stop lug or post: a radial box from r_in to r_out, deg_width wide at r_out (its sides parallel), standing
    on z0, turned to at_deg about Z."""
    half = r_out * math.tan(math.radians(deg_width / 2.0))
    lug = bd.Pos(r_in, 0.0, z0) * bd.Box(r_out - r_in, 2 * half, height, align=(bd.Align.MIN, bd.Align.CENTER, bd.Align.MIN))
    return bd.Rot(0.0, 0.0, at_deg) * lug


def slot_y(x: float, y: float, width: float, length: float, z0: float, depth: float):
    """A slot `length` long along Y, `width` wide, through a plate from z0 (a motor plate's tension slots, its pilot
    slot)."""
    return bd.Pos(x, y, z0 - NUDGE) * bd.Rot(0.0, 0.0, 90.0) * bd.extrude(bd.SlotCenterToCenter(length, width), amount=depth + 2 * NUDGE)


def nut_slot(r_in: float, r_out: float, af: float, z0: float, z1: float, at_rad: float):
    """A nut's pocket: a radial slot from r_in to r_out, af wide (a flat either side of the nut), z0 .. z1, along at_rad."""
    slot = bd.Pos((r_in + r_out) / 2.0, 0.0, z0) * bd.Box(r_out - r_in, af, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))
    return bd.Rot(0.0, 0.0, math.degrees(at_rad)) * slot
