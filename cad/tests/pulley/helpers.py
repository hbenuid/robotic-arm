"""The pulley tests' own helpers (tests/pulley/test_*.py; the shared geometry helpers are tests/helpers.py): probe
points about a pulley's axis and a pulley's surface census - every printed pulley is modelled with its axis on Y."""
from __future__ import annotations

import math


def at(r: float, y: float, deg: float = 45.0) -> tuple[float, float, float]:
    """A point r off the axis at `deg` about it (atan2(z, x)), y along it."""
    return r * math.cos(math.radians(deg)), y, r * math.sin(math.radians(deg))


def surfaces(shape) -> set[tuple]:
    """Every face's surface: a cylinder by (radius, its axis's offset from the pulley's), a cone by its half-angle (either
    sign: its axis may run either way) and its span along Y, a plane by (the normal's sign along Y, its Y) - every axis
    and normal along Y."""
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cone, GeomAbs_Cylinder, GeomAbs_Plane

    found = set()
    for f in shape.faces():
        s = BRepAdaptor_Surface(f.wrapped)
        kind = s.GetType()
        if kind == GeomAbs_Cylinder:
            axis = s.Cylinder().Axis()
            assert abs(abs(axis.Direction().Y()) - 1.0) < 1e-9
            found.add(("cylinder", round(s.Cylinder().Radius(), 5), round(math.hypot(axis.Location().X(), axis.Location().Z()), 5)))
        elif kind == GeomAbs_Cone:
            bb = f.bounding_box()
            found.add(("cone", round(abs(math.degrees(s.Cone().SemiAngle())), 5), round(bb.min.Y, 3), round(bb.max.Y, 3)))
        else:
            assert kind == GeomAbs_Plane and abs(abs(f.normal_at().Y) - 1.0) < 1e-9
            found.add(("plane", round(f.normal_at().Y), round(f.center().Y, 5)))
    return found
