"""build123d builder of a closed GT2 belt round its loop (lib/belts.py: circles with signed pitch radii, counter-clockwise
in the XY plane): the band between the belt's land - its tooth side's flat, GT2_PLD inside the pitch line, on a pulley's
tooth tips - and its back - GT2_BELT_BACK outside it, on a back-side idler's seat -, GT2_BELT_W wide and centred on
z=0. No teeth: they would have to mesh with every pulley's grooves; the band is the envelope the clearance checks need,
and a purchased belt's geometry (parts/*/<joint>_belt.py)."""
from __future__ import annotations

import math

import build123d as bd

from lib.belts import GT2_BELT_BACK, GT2_BELT_W, GT2_PLD, belt_runs, belt_wraps
from lib.geom import single_solid


def _loop_face(circles, offset: float):
    """The face inside the loop's line `offset` outside its pitch line: its arcs and runs in order."""
    edges = []
    for (x, y, r), (a0, sweep), (start, end, _) in zip(circles, belt_wraps(circles, offset), belt_runs(circles, offset), strict=True):
        rr = r + offset

        def at(a: float, x=x, y=y, rr=rr) -> tuple[float, float]:
            return (x + rr * math.cos(a), y + rr * math.sin(a))

        edges.append(bd.ThreePointArc(at(a0), at(a0 + sweep / 2.0), at(a0 + sweep)))
        edges.append(bd.Line(start, end))
    return bd.Face(bd.Wire(edges))


def closed_belt(circles, width: float = GT2_BELT_W):
    """The belt's band round `circles`, z from -width / 2 to +width / 2."""
    band = _loop_face(circles, GT2_BELT_BACK) - _loop_face(circles, -GT2_PLD)
    return single_solid(bd.Pos(0.0, 0.0, -width / 2.0) * bd.extrude(band, amount=width))
