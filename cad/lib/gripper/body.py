"""build123d builders of the gripper's measured parts (lib/gripper/params.py), each in its export's frame: build_base(),
build_rack(), build_jaw(), build_pinion(). Algebra mode; every cutter overshoots its faces by NUDGE."""
from __future__ import annotations

import math

import build123d as bd

from lib.geom import box, cylinder, single_solid, through
from lib.gripper.params import DEFAULT, BaseParams, JawParams, PinionParams, RackParams
from lib.units import NUDGE


def _yz_prism(points, x0: float, x1: float):
    """A prism of the (y, z) polygon `points`, from x0 to x1."""
    return bd.extrude(bd.Plane.YZ.offset(x0) * bd.Polygon(*points, align=None), amount=x1 - x0)


def _xz_prism(points, y0: float, y1: float):
    """A prism of the (x, z) polygon `points`, from y0 to y1."""
    # Plane.XZ's normal is -Y: its local (u, v) = (x, z); offset(-y1) puts it at y1, extruding toward -y down to y0
    return bd.extrude(bd.Plane.XZ.offset(-y1) * bd.Polygon(*points, align=None), amount=y1 - y0)


def _face(points):
    """A plane Face of the closed (x, y) polygon `points` in XY."""
    return bd.Face(bd.Wire.make_polygon([(x, y, 0.0) for x, y in points], close=True))


def _arc(centre, radius: float, p0, p1):
    """The short arc of the circle (centre, radius) from p0 to p1 (both on it)."""
    a0 = math.atan2(p0[1] - centre[1], p0[0] - centre[0])
    a1 = math.atan2(p1[1] - centre[1], p1[0] - centre[0])
    d = (a1 - a0 + math.pi) % (2 * math.pi) - math.pi
    am = a0 + d / 2
    return bd.ThreePointArc(p0, (centre[0] + radius * math.cos(am), centre[1] + radius * math.sin(am)), p1)


# --- gripper_base -----------------------------------------------------------------------------------------------------

def build_base(cfg: BaseParams = DEFAULT.base):
    b = cfg
    body = box((-b.half_x, b.half_x), (-b.half_y, b.half_y), (0.0, b.plate_t))
    for s in (1.0, -1.0):
        body += box((-b.half_x, b.half_x), sorted((s * b.wall_y, s * b.half_y)), (0.0, b.wall_top))
        for x in ((-b.half_x, -b.rib_x), (b.rib_x, b.half_x)):
            body += box(x, sorted((s * b.rib_y[0], s * b.rib_y[1])), (b.plate_t, b.wall_top))
    flare_z = b.post_mid_z + (b.post_top_y - b.post_y) / math.tan(math.radians(b.flare_deg))
    upper = [(-b.post_y, b.post_mid_z), (b.post_y, b.post_mid_z), (b.post_top_y, flare_z), (b.post_top_y, b.post_top_z),
             (-b.post_top_y, b.post_top_z), (-b.post_top_y, flare_z)]
    for x0, x1 in b.posts_x:
        body += box((x0, x1), (-b.post_y, b.post_y), (b.plate_t, b.post_mid_z))
        body += _yz_prism(upper, x0, x1)
    for s in (1.0, -1.0):   # the slots, through the plate
        y0, y1 = sorted((s * b.slot_y[0], s * b.slot_y[1]))
        body -= box((-b.slot_x, b.slot_x), (y0, y1), (-NUDGE, b.plate_t + NUDGE))
        for x in (-b.slot_x, b.slot_x):
            body -= through(b.slot_end_r, b.plate_t, (x, s * b.slot_end_y))
    for (x0, x1), hole_x, nut_x, rib_x in zip(b.posts_x, b.holes_x, b.nut_x, b.rib_slot_x, strict=True):
        low = x1 < 0.0   # the -x post: its outer face is x0 (the nut trap opens there), its inner face x1
        for s in (1.0, -1.0):
            body -= cylinder(b.hole_r, b.post_top_z - b.hole_z + NUDGE, (hole_x, s * b.hole_y), b.hole_z)
            nx = (x0 - NUDGE, nut_x) if low else (nut_x, x1 + NUDGE)
            body -= box(nx, sorted((s * b.nut_y[0], s * b.nut_y[1])), b.nut_z)
        rx = (rib_x, x1 + NUDGE) if low else (x0 - NUDGE, rib_x)
        body -= box(rx, (-b.rib_slot_y, b.rib_slot_y), (b.rib_slot_z, b.post_top_z + NUDGE))
    return single_solid(body)


# --- gripper_rack -----------------------------------------------------------------------------------------------------

def _tooth(cfg: RackParams, c: float):
    """One tooth's face, centred on x = c, from the bar's edge (y = body_y[0]) to its tip."""
    r = cfg
    root = r.body_y[0]
    # the two arcs are tangent, one inside the other: arc2's centre sits on the line from arc1's through its measured
    # point, arc2_r - arc1_r from it (7e-5 off the export's 4-decimal value - a short arc's fit would bend by 0.01),
    # and they meet on that line
    (c1x, c1y) = (r.arc1_x, r.flank_y)
    d = math.hypot(r.arc2_c[0] - c1x, r.arc2_c[1] - c1y)
    ux, uy = (r.arc2_c[0] - c1x) / d, (r.arc2_c[1] - c1y) / d
    c2x, c2y = c1x + (r.arc2_r - r.arc1_r) * ux, c1y + (r.arc2_r - r.arc1_r) * uy
    jx, jy = c1x - r.arc1_r * ux, c1y - r.arc1_r * uy
    a2x = c2x - math.sqrt(r.arc2_r**2 - (root - c2y) ** 2)                          # the root's half-width
    tip_cy = r.tip_y + r.tip_r
    half = [(a2x, root), (jx, jy), (r.flank_x, r.flank_y), (r.flank_x, tip_cy), (r.tip_x, r.tip_y)]
    left = [(c - x, y) for x, y in half]
    right = [(c + x, y) for x, y in reversed(half)]
    edges = [
        _arc((c - c2x, c2y), r.arc2_r, left[0], left[1]),
        _arc((c - r.arc1_x, r.flank_y), r.arc1_r, left[1], left[2]),
        bd.Line(left[2], left[3]),
        _arc((c - r.tip_x, tip_cy), r.tip_r, left[3], left[4]),
        bd.Line(left[4], right[0]),
        _arc((c + r.tip_x, tip_cy), r.tip_r, right[0], right[1]),
        bd.Line(right[1], right[2]),
        _arc((c + r.arc1_x, r.flank_y), r.arc1_r, right[2], right[3]),
        _arc((c + c2x, c2y), r.arc2_r, right[3], right[4]),
        bd.Line(right[4], left[0]),
    ]
    return bd.extrude(bd.Face(bd.Wire(edges)), amount=r.t)


def build_rack(cfg: RackParams = DEFAULT.rack):
    r = cfg
    body = box((-r.half_x, r.half_x), r.body_y, (0.0, r.t))
    n = r.teeth // 2
    for k in range(-n, r.teeth - n):
        body += _tooth(r, k * r.pitch)
    y_mid = sum(r.post_y) / 2.0
    w = r.post_y[1] - r.post_y[0]
    for (x0, x1), slit, hole_x in zip(r.posts_x, r.slits_x, r.holes_x, strict=True):
        post = bd.Pos((x0 + x1) / 2.0, y_mid, r.t) * bd.extrude(bd.SlotCenterToCenter((x1 - x0) - w, w), amount=r.post_top - r.t)
        body += post
        body -= box(slit, (r.post_y[0] - NUDGE, r.post_y[1] + NUDGE), (r.slit_z, r.post_top + NUDGE))
        body -= cylinder(r.hole_r, r.post_top - r.head_z + NUDGE, (hole_x, y_mid), r.head_z)
        body -= cylinder(r.head_r, r.head_z + NUDGE, (hole_x, y_mid), -NUDGE)
    return single_solid(body)


# --- gripper_jaw ------------------------------------------------------------------------------------------------------

def build_jaw(cfg: JawParams = DEFAULT.jaw):
    j = cfg
    fx0, fx1 = j.foot_x
    # the plate: the arm and the foot as one outline, its five outer corners rounded (the arm meets the face square)
    outline = _face([(j.arm_x[0], j.arm_y[0]), (fx0, j.arm_y[0]), (fx0, j.foot_y[0]), (fx1, j.foot_y[0]),
                     (fx1, j.arm_y[1]), (j.arm_x[0], j.arm_y[1])])
    corners = [v for v in outline.vertices() if abs(v.X - fx0) > 1e-6 or abs(v.Y - j.arm_y[0]) > 1e-6]
    outline = outline.fillet_2d(j.corner_r, corners)
    plate = bd.extrude(outline, amount=j.t)
    footprint = bd.extrude(outline, amount=j.riser_top + NUDGE)
    # the riser: 45 deg on its -y, +y and +x sides (planes: two prisms crossed), inside the plate's rounded footprint
    rise, ry = j.riser_top - j.t, j.riser_y
    across = _yz_prism([(-ry, j.t), (ry, j.t), (ry - rise, j.riser_top), (-ry + rise, j.riser_top)], fx0 - 1.0, fx1 + 1.0)
    along = _xz_prism([(fx0 - 1.0, j.t), (fx1, j.t), (fx1 - rise, j.riser_top), (fx0 - 1.0, j.riser_top)], -ry - 1.0, ry + 1.0)
    riser = across & along & footprint
    # the finger: its section (|y| <= finger_w) along x from the face, cut by its tilted round back, then the top's
    # edges - all but the face's - rounded top_r (a rolling ball: round the back it leaves the export's blend)
    w = j.finger_w
    sec = _face([(-w, j.riser_top), (w, j.riser_top), (w, j.finger_top), (-w, j.finger_top)])   # (y, z)
    section = bd.extrude(bd.Plane.YZ.offset(j.finger_x) * sec, amount=fx1 - j.finger_x)
    dx, dz = j.back_dir
    px, _, pz = j.back_point
    back = bd.Pos(px, 0.0, pz) * bd.Rot(0.0, math.degrees(math.atan2(dx, dz)), 0.0) * bd.Cylinder(w, 4 * j.finger_top)
    lo, hi = -10.0, j.finger_top + 10.0
    back += _xz_prism([(j.finger_x - 1.0, lo), (px + (lo - pz) * dx / dz, lo), (px + (hi - pz) * dx / dz, hi),
                       (j.finger_x - 1.0, hi)], -w - 1.0, w + 1.0)
    finger = single_solid(section & back)
    top = [e for e in finger.edges() if abs(e.center().Z - j.finger_top) < 1e-6 and e.center().X > j.finger_x + 1e-6]
    finger = finger.fillet(j.top_r, top)
    body = plate + riser + finger
    for bx, by in j.bosses:
        body += bd.Pos(bx, by, j.t) * bd.extrude(bd.RectangleRounded(*j.boss_size, j.boss_r), amount=j.riser_top - j.t)
        body -= bd.Pos(bx, by, j.t) * bd.extrude(bd.RegularPolygon(j.nut_af / math.sqrt(3.0), 6), amount=j.riser_top - j.t + NUDGE)
        body -= through(j.hole_r, j.t, (bx, by))
    # the ramp under the foot's end: 0 deep at notch_x[0], notch_depth at the end
    (nx0, nx1), slope = j.notch_x, j.notch_depth / (j.notch_x[1] - j.notch_x[0])
    body -= _xz_prism([(nx0 - 1.0, -slope), (nx1 + 1.0, -slope), (nx1 + 1.0, j.notch_depth + slope)], *j.notch_y)
    return single_solid(body)


# --- gripper_pinion ---------------------------------------------------------------------------------------------------

def _inv(a: float) -> float:
    return math.tan(a) - a


def _polar(r: float, a: float):
    return (r * math.cos(a), r * math.sin(a))


def pinion_outline(cfg: PinionParams = DEFAULT.pinion):
    """The gear's outline as a closed Wire in its plane: per tooth (centred on k * 360/teeth deg from +x) the root's
    rounds, the radial flank under the base circle, the involute to the tip circle, the tip arc."""
    p = cfg
    rp = p.pitch_r
    alpha = math.radians(p.pressure_deg)
    rb = rp * math.cos(alpha)
    psi_b = p.thickness / (2 * rp) + _inv(alpha)            # the tooth's half-angle on (and under) the base circle

    def psi(r):
        return psi_b - _inv(math.acos(rb / r))

    rc = p.root_r + p.fillet_r                               # the root rounds' centres, tangent to the root circle ...
    phi_c = psi_b + math.asin(p.fillet_r / rc)               # ... and to the radial flank
    r_t = math.sqrt(rc**2 - p.fillet_r**2)                   # where a round meets the flank
    pitch = 2 * math.pi / p.teeth
    inv_r = [rb + (p.tip_r - rb) * (i / 24) ** 1.5 for i in range(25)]
    edges = []
    for k in range(p.teeth):
        c = k * pitch
        for side in (-1.0, 1.0):
            root_pt = _polar(p.root_r, c + side * phi_c)
            flank_pt = _polar(r_t, c + side * psi_b)
            centre = _polar(rc, c + side * phi_c)
            invo = [_polar(r, c + side * psi(r)) for r in inv_r]
            if side < 0:     # up the -side flank
                edges += [_arc(centre, p.fillet_r, root_pt, flank_pt), bd.Line(flank_pt, invo[0]), bd.Spline(*invo)]
                tip0 = invo[-1]
            else:            # the tip, then down the +side flank
                tip_mid = _polar(p.tip_r, c)
                edges += [bd.ThreePointArc(tip0, tip_mid, invo[-1]), bd.Spline(*reversed(invo)),
                          bd.Line(invo[0], flank_pt), _arc(centre, p.fillet_r, flank_pt, root_pt)]
                nxt = _polar(p.root_r, c + pitch - phi_c)
                edges.append(bd.ThreePointArc(root_pt, _polar(p.root_r, c + pitch / 2), nxt))
    return bd.Wire(edges)


def build_pinion(cfg: PinionParams = DEFAULT.pinion):
    p = cfg
    body = bd.extrude(bd.Face(pinion_outline(p)), amount=p.width)
    body -= cylinder(p.head_r, p.head_z + NUDGE, z0=-NUDGE)
    body -= through(p.hole_r, p.width)
    body -= cylinder(p.spline_r, p.width - p.spline_z + NUDGE, z0=p.spline_z)
    return single_solid(body)
