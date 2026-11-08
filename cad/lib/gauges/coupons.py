"""The fit gauges' coupons (lib/gauges/fits.py says which): build123d bodies, each standing on z = 0 with its footprint's
low corner at the origin (the bed's frame).

A coupon prints one fit at a sweep of sizes (sweep()): each step's printed size debossed beside it, an arm value with a
bar under its number. Strips run their steps along X, the labels in front (-Y) on a shelf, the title under them; the big
rings are one piece per step, the size on a tab. Upright: the feature's axis along print Z; `side_strip`: along Y, in a
wall standing on a foot. Units mm.

The kernel stays lazy (cad/AGENTS.md "Lazy kernel"): `bd.` inside function bodies only.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from pathlib import Path

import build123d as bd

from lib.units import NUDGE

FONT = Path(__file__).parent / "font" / "liberation_sans_bold.ttf"   # SIL OFL 1.1 (font/license.txt): one font everywhere
STEP = 0.05          # [DESIGN] the sweep's step on a diameter / across flats / gap ...
REACH = 2            # [DESIGN] ... this many steps either side of each arm value
TXT_DEPTH = 0.4      # [DESIGN] the labels' deboss: two 0.2 layers
TXT = 3.0            # [DESIGN] a size's label (cap height about 2.1)
TITLE = 3.4          # [DESIGN] a strip's title
SHELF = 1.6          # [DESIGN] the label shelf's height (and the pegs' / clips' base)
MAX_LEN = 236.0      # [DESIGN] the longest strip: a 256 bed less its margins - a longer sweep splits
TAB = (30.0, 13.0)   # [DESIGN] a ring's tab (width, reach past the ring) carrying its size and title
BAR = (0.6, 1.2)     # [DESIGN] an arm value's bar: from this far under the label's half-height to this far


def sweep(values, step: float = STEP, reach: int = REACH) -> list[float]:
    """Every size a gauge prints: each arm value and `reach` steps either side of it, merged into one ascending run (a
    single missing step between two runs filled in), rounded to 0.001."""
    sizes = sorted({round(v + k * step, 3) for v in values for k in range(-reach, reach + 1)})
    out = [sizes[0]]
    for s in sizes[1:]:
        if abs(s - out[-1] - 2.0 * step) < 1e-6:
            out.append(round(out[-1] + step, 3))
        out.append(s)
    return out


def fmt(size: float) -> str:
    return f"{size:.2f}"


def is_arm(size: float, values) -> bool:
    return any(abs(size - v) < 1e-6 for v in values)


@dataclass
class Coupon:
    """One printed piece: `shape` on the bed (z 0 up, its footprint from the origin), the sizes it prints in order, the
    arm values among them; a ring's clear `opening` round `centre` (lib/gauges/layout.py nests small strips there)."""
    key: str
    title: str
    shape: object
    sizes: list = field(default_factory=list)
    marks: tuple = ()
    orient: str = "upright"
    kind: str = ""
    opening: float = 0.0
    centre: tuple = (0.0, 0.0)
    gauge: str = ""      # the key of the gauge it prints (build() sets it)

    @property
    def size(self) -> tuple[float, float, float]:
        bb = self.shape.bounding_box()
        return (bb.size.X, bb.size.Y, bb.size.Z)


# --- primitives -------------------------------------------------------------------------------------------------

def _text(s: str, size: float):
    return bd.Text(s, font_size=size, font_path=str(FONT), align=(bd.Align.CENTER, bd.Align.CENTER))


def text_width(s: str, size: float) -> float:
    bb = _text(s, size).bounding_box()
    return bb.max.X - bb.min.X


def deboss(s: str, size: float, x: float, y: float, z_top: float):
    """The cutter of `s` centred on (x, y), TXT_DEPTH down from the face at z_top."""
    return bd.Pos(x, y, z_top + NUDGE) * bd.extrude(_text(s, size), amount=-(TXT_DEPTH + NUDGE))


def box(x0, x1, y0, y1, z0, z1):
    return bd.Pos((x0 + x1) / 2.0, (y0 + y1) / 2.0, z0) * bd.Box(
        x1 - x0, y1 - y0, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def cyl(d, z0, z1, x=0.0, y=0.0):
    return bd.Pos(x, y, z0) * bd.Cylinder(d / 2.0, z1 - z0, align=(bd.Align.CENTER, bd.Align.CENTER, bd.Align.MIN))


def hexagon(af, z0, z1, x=0.0, y=0.0, corner_deg=0.0):
    """A hex prism `af` across flats, a corner at corner_deg from +X."""
    return bd.Pos(x, y, z0) * bd.extrude(bd.RegularPolygon(af / 2.0, 6, major_radius=False, rotation=corner_deg),
                                         amount=z1 - z0)


def d_shape(d, flat, z0, z1, x=0.0, y=0.0):
    """A D: a circle `d`, its flat `flat` off the axis on +Y."""
    face = bd.Circle(d / 2.0) - bd.Pos(0.0, flat) * bd.Rectangle(2.0 * d, d, align=(bd.Align.CENTER, bd.Align.MIN))
    return bd.Pos(x, y, z0) * bd.extrude(face, amount=z1 - z0)


def _label(size, x, y, z_top, marks, font=TXT):
    cut = deboss(fmt(size), font, x, y, z_top)
    if is_arm(size, marks):
        w = text_width(fmt(size), font)
        cut = cut + box(x - w / 2.0, x + w / 2.0, y - font / 2.0 - BAR[1], y - font / 2.0 - BAR[0], z_top - TXT_DEPTH,
                        z_top + NUDGE)
    return cut


def _labels(sizes, xs, y, z_top, marks):
    cut = None
    for s, x in zip(sizes, xs, strict=True):
        c = _label(s, x, y, z_top, marks)
        cut = c if cut is None else cut + c
    return cut


def _pitch(feature_w: float, sizes) -> float:
    return max(feature_w, max(text_width(fmt(s), TXT) for s in sizes) + 2.2)


def _xs(n: int, pitch: float) -> list[float]:
    return [(i - (n - 1) / 2.0) * pitch for i in range(n)]


def _on_bed(shape):
    bb = shape.bounding_box()
    return bd.Pos(-bb.min.X, -bb.min.Y, -bb.min.Z) * shape


def _strips(key, title, sizes, marks, pitch, build, orient, kind) -> list[Coupon]:
    """`build(part, title)` for each strip of the sweep, as few as keep each under MAX_LEN, as even as possible."""
    per = max(1, int(MAX_LEN // pitch))
    n = math.ceil(len(sizes) / per)
    k = math.ceil(len(sizes) / n)
    parts = [sizes[i:i + k] for i in range(0, len(sizes), k)]
    out = []
    for i, part in enumerate(parts, 1):
        t, kk = (title, key) if len(parts) == 1 else (f"{title} {i}/{len(parts)}", f"{key}_{i}")
        out.append(Coupon(kk, t, _on_bed(build(part, t)), list(part), tuple(marks), orient, kind))
    return out


# --- the coupons ------------------------------------------------------------------------------------------------

def hole_strip(key, title, values, *, depth, width, shape="round", floor=1.2, wall=2.4, through=False, push=None,
               under=None, d_flat=None, channel=None, step=STEP, reach=REACH):
    """A bar `floor + depth` tall, one hole per size from its top: round, hex (a corner along Y), a D (its flat
    `d_flat` off the axis at the arm value, moving out half a step a step) or a channel (`channel` long along Y, the
    size across X). `through`: the hole through the floor too; `push`: a hole that diameter through the floor (a
    bearing pushed back out); `under`: a smaller hole through the floor (a screw under its head or nut). `width`: the
    feature's widest extent (the bar's depth is it + 2 walls)."""
    sizes = sweep(values, step, reach)
    pitch = _pitch(width + 2.0 * wall, sizes)
    top = floor + depth
    half = width / 2.0 + wall

    def cutter(s, x):
        z0 = -NUDGE if through else floor
        if shape == "round":
            c = cyl(s, z0, top + NUDGE, x)
        elif shape == "hex":
            c = hexagon(s, z0, top + NUDGE, x, corner_deg=90.0)
        elif shape == "d":
            c = d_shape(s, d_flat + (s - values[0]) / 2.0, z0, top + NUDGE, x)
        else:
            c = box(x - s / 2.0, x + s / 2.0, -channel / 2.0, channel / 2.0, z0, top + NUDGE)
        for d in (push, under):
            if d is not None:
                c = c + cyl(d, -NUDGE, floor + NUDGE, x)
        return c

    def build(part, t):
        xs = _xs(len(part), pitch)
        shelf, lab_y = min(top, SHELF), -half - 2.9
        tit_y = lab_y - 5.4
        body = box(xs[0] - pitch / 2, xs[-1] + pitch / 2, -half, half, 0.0, top)
        body = body + box(xs[0] - pitch / 2, xs[-1] + pitch / 2, tit_y - 3.2, -half + NUDGE, 0.0, shelf)
        for s, x in zip(part, xs, strict=True):
            body = body - cutter(s, x)
        return body - _labels(part, xs, lab_y, shelf, values) - deboss(t, TITLE, 0.0, tit_y, shelf)

    return _strips(key, title, sizes, values, pitch, build, "upright", shape)


def peg_strip(key, title, values, *, height, spacing, bore=None, step=STEP, reach=REACH):
    """Pegs (rings round a `bore`) standing on the bed, `spacing` apart at least - a bought part's bore goes down over
    one at a time -, a label shelf touching their fronts."""
    sizes = sweep(values, step, reach)
    pitch = _pitch(spacing, sizes)

    def build(part, t):
        xs = _xs(len(part), pitch)
        lab_y = -max(sizes) / 2.0 - 3.9
        tit_y = lab_y - 5.4
        body = box(xs[0] - pitch / 2, xs[-1] + pitch / 2, tit_y - 3.2, -min(sizes) / 2.0 + 2.0, 0.0, SHELF)
        for s, x in zip(part, xs, strict=True):
            peg = cyl(s, 0.0, SHELF + height, x)
            if bore:
                peg = peg - cyl(bore, -NUDGE, SHELF + height + NUDGE, x)
            body = body + peg
            if bore:
                body = body - cyl(bore, -NUDGE, SHELF + NUDGE, x)
        return body - _labels(part, xs, lab_y, SHELF, values) - deboss(t, TITLE, 0.0, tit_y, SHELF)

    return _strips(key, title, sizes, values, pitch, build, "upright", "peg")


def seat_strip(key, title, values, *, band, wall=3.0, lip=1.0, ledge=1.5, step=STEP, reach=REACH):
    """Seat rings side by side, joined where they meet: a bore `band` deep over a lip `lip` thick and `ledge` wide (the
    bought part stops on it and is pushed back out from below), a label shelf along the front."""
    sizes = sweep(values, step, reach)
    od = max(sizes) + 2.0 * wall
    pitch = _pitch(od - wall, sizes)

    def build(part, t):
        xs = _xs(len(part), pitch)
        top = lip + band
        body = None
        for x in xs:
            body = cyl(od, 0.0, top, x) if body is None else body + cyl(od, 0.0, top, x)
        lab_y = -od / 2.0 - 2.9
        tit_y = lab_y - 5.4
        body = body + box(xs[0] - od / 2 + 4.0, xs[-1] + od / 2 - 4.0, tit_y - 3.2, -od / 2.0 + wall, 0.0, SHELF)
        for s, x in zip(part, xs, strict=True):
            body = body - cyl(s, lip, top + NUDGE, x) - cyl(s - 2.0 * ledge, -NUDGE, lip + NUDGE, x)
        return body - _labels(part, xs, lab_y, SHELF, values) - deboss(t, TITLE, 0.0, tit_y, SHELF)

    return _strips(key, title, sizes, values, pitch, build, "upright", "seat")


def _tab(r_out, z_top, size, title, values, inner_d):
    tab = box(-TAB[0] / 2, TAB[0] / 2, -r_out - TAB[1], -r_out + 2.0, 0.0, z_top) - cyl(inner_d, -NUDGE, z_top + NUDGE)
    return tab - _label(size, 0.0, -r_out - 3.6, z_top, values, TXT + 0.6) - deboss(title, 2.6, 0.0, -r_out - 9.0,
                                                                                    z_top)


def ring_seats(key, title, values, *, band, wall=3.0, lip=1.0, ledge=1.5, step=STEP, reach=REACH):
    """One ring per size, its seat bore `band` deep over a lip (seat_strip's), its size on a tab."""
    out = []
    for s in sweep(values, step, reach):
        r = s / 2.0 + wall
        body = cyl(2.0 * r, 0.0, lip + band) - cyl(s, lip, lip + band + NUDGE) - cyl(s - 2.0 * ledge, -NUDGE, lip + NUDGE)
        body = body + _tab(r, SHELF, s, title, values, s)
        out.append(Coupon(f"{key}_{fmt(s)}", f"{title} {fmt(s)}", _on_bed(body), [s], tuple(values), "upright",
                          "seat ring", s - 2.0 * ledge, (r, r + TAB[1])))
    return out


def ring_bosses(key, title, values, *, band, wall=3.0, step=STEP, reach=REACH):
    """One ring per size, tested on its OUTSIDE (a bought part's bore goes down over it to the tab), its size on a
    tab."""
    out = []
    for s in sweep(values, step, reach):
        inner = s - 2.0 * wall
        body = cyl(s, 0.0, SHELF + band) - cyl(inner, -NUDGE, SHELF + band + NUDGE)
        body = body + _tab(s / 2.0, SHELF, s, title, values, inner)
        out.append(Coupon(f"{key}_{fmt(s)}", f"{title} {fmt(s)}", _on_bed(body), [s], tuple(values), "upright",
                          "boss ring", inner, (s / 2.0, s / 2.0 + TAB[1])))
    return out


def slot_strip(key, title, values, *, af, height, screw, wall=2.4, floor=1.6, roof=1.6, past=0.2, step=STEP,
               reach=REACH):
    """Nut slots in from the bar's front face (the nut's axis along Z, its flats on the slot's walls, the size the
    slot's width), on past the nut's far corner by `past`, `height` tall; a `screw` hole down through each on the nut's
    axis."""
    sizes = sweep(values, step, reach)
    corner = af / math.sqrt(3.0)
    pitch = _pitch(max(sizes) + 2.0 * wall, sizes)

    def build(part, t):
        xs = _xs(len(part), pitch)
        front, back, top = -af / 2.0 - 3.0, corner + past + wall, floor + height + roof
        lab_y = front - 3.2
        tit_y = lab_y - 5.4
        body = box(xs[0] - pitch / 2, xs[-1] + pitch / 2, front, back, 0.0, top)
        body = body + box(xs[0] - pitch / 2, xs[-1] + pitch / 2, tit_y - 3.2, front + NUDGE, 0.0, floor)
        for s, x in zip(part, xs, strict=True):
            body = body - box(x - s / 2.0, x + s / 2.0, front - NUDGE, corner + past, floor, floor + height)
            body = body - cyl(screw, -NUDGE, top + NUDGE, x)
        return body - _labels(part, xs, lab_y, floor, values) - deboss(t, TITLE, 0.0, tit_y, floor)

    return _strips(key, title, sizes, values, pitch, build, "upright", "slot")


def side_strip(key, title, values, *, depth, shape="round", wall=2.6, blind=False, screw=None, step=STEP,
               reach=REACH):
    """A wall standing on a foot, each hole running into it along +Y from its front face: round, or hex with a corner
    up (its roof an arch: no bridge); `blind` stops it `depth` in, `screw` a hole on through the wall. The labels on
    the foot in front."""
    sizes = sweep(values, step, reach)
    width = max(sizes) * (2.0 / math.sqrt(3.0) if shape == "hex" else 1.0)
    pitch = _pitch(width + 2.0 * wall, sizes)

    def build(part, t):
        xs = _xs(len(part), pitch)
        zc = SHELF + wall + width / 2.0
        top, thick = zc + width / 2.0 + wall, depth + (2.0 if blind else 0.0)
        lab_y = -3.2
        tit_y = lab_y - 5.4
        x0, x1 = xs[0] - pitch / 2, xs[-1] + pitch / 2
        body = box(x0, x1, 0.0, thick, 0.0, top) + box(x0, x1, tit_y - 3.2, NUDGE, 0.0, SHELF)
        body = body + box(x0, x1, thick - NUDGE, thick + 6.0, 0.0, SHELF)   # a heel behind: it stands
        for s, x in zip(part, xs, strict=True):
            prof = bd.Circle(s / 2.0) if shape == "round" else bd.RegularPolygon(s / 2.0, 6, major_radius=False,
                                                                                 rotation=90.0)
            cut = bd.Pos(x, -NUDGE, zc) * bd.Rot(-90.0, 0.0, 0.0) * bd.extrude(prof, amount=depth + NUDGE)
            if screw is not None:
                cut = cut + bd.Pos(x, -NUDGE, zc) * bd.Rot(-90.0, 0.0, 0.0) * bd.extrude(bd.Circle(screw / 2.0),
                                                                                         amount=thick + 2.0 * NUDGE)
            body = body - cut
        return body - _labels(part, xs, lab_y, SHELF, values) - deboss(t, TITLE, 0.0, tit_y, SHELF)

    return _strips(key, title, sizes, values, pitch, build, "side", f"side {shape}")


def gap_clips(key, title, values, *, height, wall=3.0, depth=14.0, step=STEP, reach=REACH):
    """One U-clip per size: two walls that far apart (a bought body - a servo's case - presses between them), the size
    and the title on the floor between them; separate pieces, so a body never meets two sizes at once."""
    out = []
    for s in sweep(values, step, reach):
        x0, x1 = -s / 2.0 - wall, s / 2.0 + wall
        body = box(x0, x1, -depth / 2, depth / 2, 0.0, SHELF)
        body = body + box(x0, -s / 2.0, -depth / 2, depth / 2, SHELF - NUDGE, SHELF + height)
        body = body + box(s / 2.0, x1, -depth / 2, depth / 2, SHELF - NUDGE, SHELF + height)
        body = body - _label(s, 0.0, 2.6, SHELF, values) - deboss(title, 2.4, 0.0, -3.6, SHELF)
        out.append(Coupon(f"{key}_{fmt(s)}", f"{title} {fmt(s)}", _on_bed(body), [s], tuple(values), "upright",
                          "gap clip"))
    return out


BUILDERS = {f.__name__: f for f in (hole_strip, peg_strip, seat_strip, ring_seats, ring_bosses, slot_strip, side_strip,
                                    gap_clips)}


def build(gauge) -> list[Coupon]:
    """A gauge's coupons (lib/gauges/fits.py Gauge): its builder with its options, its arm values marked."""
    out = BUILDERS[gauge.coupon](gauge.key, gauge.title, gauge.values, step=gauge.step, reach=gauge.reach,
                                 **dict(gauge.options))
    for c in out:
        c.gauge = gauge.key
    return out
