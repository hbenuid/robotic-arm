"""The GT2 groove and the toothed ring of a printed pulley, from lib/belts.py's tooth form.

A groove is arcs only, each tangent to the next: from the land a fillet, the flank, a blend, the bottom, then the same
back up. groove_centres() / groove_arcs() are the pure math (the tests check them without the kernel); gt2_profile()
builds the toothed outline as ONE closed wire of those arcs and the land arcs between them, never as a boolean - the
fillets touch the land tangentially, where a subtraction would have to split a tangency."""
from __future__ import annotations

import math

from cadgen import build123d as bd

from lib.belts import GT2_BLEND_R, GT2_FLANK_R, GT2_GROOVE_R, GT2_TIP_R, GT2_TOOTH_DEPTH, flank_offset, pulley_od
from lib.geom import align_min, cylinder, single_solid
from lib.units import NUDGE


def _circles_meet(p, d1: float, q, d2: float, pick):
    """The one of the two points at d1 from p and d2 from q that `pick` chooses from the pair."""
    dx, dy = q[0] - p[0], q[1] - p[1]
    d = math.hypot(dx, dy)
    a = (d1 * d1 - d2 * d2 + d * d) / (2.0 * d)
    h = math.sqrt(d1 * d1 - a * a)
    mx, my = p[0] + a * dx / d, p[1] + a * dy / d
    return pick([(mx + h * dy / d, my - h * dx / d), (mx - h * dy / d, my + h * dx / d)])


def _toward(p, q, r: float):
    """The point r from p toward q."""
    d = math.dist(p, q)
    return (p[0] + r * (q[0] - p[0]) / d, p[1] + r * (q[1] - p[1]) / d)


def groove_centres(teeth: int, blend_r: float = GT2_BLEND_R) -> dict[str, tuple[float, float]]:
    """The centres of the arcs of the +y half of a groove centred on +X (the -y half mirrors them): the bottom's on the
    centreline, the flank's on the land circle across it (at -y, flank_offset()), the blend's (radius `blend_r`) tangent
    inside both (at -y), the fillet's tangent to the land and outside the flank (at +y)."""
    r_land = pulley_od(teeth) / 2.0
    offset = flank_offset(teeth)
    bottom = (r_land - GT2_TOOTH_DEPTH + GT2_GROOVE_R, 0.0)
    flank = (math.sqrt(r_land * r_land - offset * offset), -offset)
    blend = _circles_meet(bottom, blend_r - GT2_GROOVE_R, flank, GT2_FLANK_R - blend_r, lambda c: max(c, key=lambda p: p[0]))
    tip = _circles_meet((0.0, 0.0), r_land - GT2_TIP_R, flank, GT2_FLANK_R + GT2_TIP_R, lambda c: max(c, key=lambda p: p[1]))
    return {"bottom": bottom, "blend": blend, "flank": flank, "tip": tip}


def _mid(centre, radius: float, p, q):
    """The midpoint of the short arc about `centre` from p to q."""
    ux, uy = p[0] - centre[0], p[1] - centre[1]
    vx, vy = q[0] - centre[0], q[1] - centre[1]
    nu, nv = math.hypot(ux, uy), math.hypot(vx, vy)
    return _toward(centre, (centre[0] + ux / nu + vx / nv, centre[1] + uy / nu + vy / nv), radius)


def groove_arcs(teeth: int, blend_r: float = GT2_BLEND_R) -> list[tuple[tuple[float, float], tuple[float, float], tuple[float, float]]]:
    """The 7 arcs of a groove centred on +X as (start, mid, end), in the order of increasing angle - from the land at -y
    (fillet, flank, blend) over the bottom and back up to the land at +y (blend, flank, fillet)."""
    c = groove_centres(teeth, blend_r)
    r_land = pulley_od(teeth) / 2.0
    land = _toward((0.0, 0.0), c["tip"], r_land)                             # fillet | land
    tip_flank = _toward(c["tip"], c["flank"], GT2_TIP_R)                     # fillet | flank (outside each other)
    flank_blend = _toward(c["flank"], c["blend"], GT2_FLANK_R)               # flank | blend (the blend inside the flank)
    blend_bottom = _toward(c["bottom"], c["blend"], -GT2_GROOVE_R)           # blend | bottom (the bottom inside the blend)
    upper = [(c["blend"], blend_r, blend_bottom, flank_blend), (c["flank"], GT2_FLANK_R, flank_blend, tip_flank),
             (c["tip"], GT2_TIP_R, tip_flank, land)]

    def mirror(p):
        return (p[0], -p[1])

    arcs = [(mirror(e), mirror(_mid(o, r, s, e)), mirror(s)) for o, r, s, e in reversed(upper)]
    arcs.append((mirror(blend_bottom), (c["bottom"][0] - GT2_GROOVE_R, 0.0), blend_bottom))
    arcs += [(s, _mid(o, r, s, e), e) for o, r, s, e in upper]
    return arcs


def gt2_profile(teeth: int, inner_dia: float = 0.0, blend_r: float = GT2_BLEND_R):
    """The toothed disc in the XY plane: `teeth` grooves (groove_arcs(teeth, blend_r)) in a Ø pulley_od(teeth) land, one
    centred on +X; with `inner_dia` > 0 an annulus."""
    r_land = pulley_od(teeth) / 2.0
    arcs = groove_arcs(teeth, blend_r)
    pitch = 2.0 * math.pi / teeth

    def turned(p, k):
        a = (k % teeth) * pitch
        return (p[0] * math.cos(a) - p[1] * math.sin(a), p[0] * math.sin(a) + p[1] * math.cos(a), 0.0)

    edges = []
    for k in range(teeth):
        edges += [bd.ThreePointArc(turned(s, k), turned(m, k), turned(e, k)) for s, m, e in arcs]
        mid = (k + 0.5) * pitch
        edges.append(bd.ThreePointArc(turned(arcs[-1][2], k), (r_land * math.cos(mid), r_land * math.sin(mid), 0.0),
                                      turned(arcs[0][0], k + 1)))               # the land on to the next groove
    profile = bd.Face(bd.Wire(edges))
    if inner_dia > 0.0:
        profile = profile - bd.Circle(inner_dia / 2.0)
    return profile


def _flange(radius: float, t: float, chamfer: float, z0: float, teeth_above: bool):
    """A flange disc standing on z0, `t` thick, its outer edge on the teeth's side chamfered 45 deg by `chamfer`."""
    if chamfer <= 0.0:
        return cylinder(radius, t, z0=z0)
    if teeth_above:
        return cylinder(radius, t - chamfer, z0=z0) + bd.Pos(0.0, 0.0, z0 + t - chamfer) * bd.Cone(
            radius, radius - chamfer, chamfer, align=align_min())
    return bd.Pos(0.0, 0.0, z0) * bd.Cone(radius - chamfer, radius, chamfer, align=align_min()) + cylinder(
        radius, t - chamfer, z0=z0 + chamfer)


def gt2_ring(teeth: int, width: float, flange_dia: float = 0.0, flange_t: float = 0.0, z0: float = 0.0,
             inner_dia: float = 0.0, chamfer: float = 0.0, blend_r: float = GT2_BLEND_R):
    """The toothed ring standing on z0: gt2_profile(teeth, blend_r=blend_r) over `width` along Z; with `flange_dia` > 0 a flange of
    `flange_t` below (z0 - flange_t .. z0) and above (z0 + width ..) the teeth, each with a 45 deg `chamfer` on its
    outer edge on the teeth's side; with `inner_dia` > 0 an annulus (the ring fuses onto a core of that diameter - keep
    every later boolean away from the teeth)."""
    ring = bd.Pos(0.0, 0.0, z0) * bd.extrude(gt2_profile(teeth, inner_dia, blend_r), amount=width)
    if flange_dia > 0.0 and flange_t > 0.0:
        r = flange_dia / 2.0
        flange = _flange(r, flange_t, chamfer, z0 - flange_t, True) + _flange(r, flange_t, chamfer, z0 + width, False)
        if inner_dia > 0.0:
            flange = flange - cylinder(inner_dia / 2.0, width + 2 * flange_t + 2 * NUDGE, z0=z0 - flange_t - NUDGE)
        ring = ring + flange
    return single_solid(ring)
