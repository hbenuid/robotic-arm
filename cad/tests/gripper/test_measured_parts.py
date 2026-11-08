"""The gripper's four measured conversions (lib/gripper/, lib/reference.py MEASURED): PapaBravo's rack & pinion gripper
(Printables, CC BY 4.0). No reference file is committed, so the numbers measured 2026-11-07 on the exports live HERE -
each export's solids fused: its volume, its bounding box, its cylinders' radii - and feature probes at points read off
the exports.

How close each build is (measured against the export when this was written, not re-run: the exports are not in the
repo): the base's symmetric difference 0.0000 mm^3 and every plane / cylinder exact; the rack's 0.013 mm^3, every
surface within 0.0005 mm; the jaw's every plane and cylinder within 0.001 mm, 1.4 mm^3 short in the finger's top
rounds (a rolling-ball fillet here, the export's blend a little fuller); the pinion's flanks a fitted 14.5 deg involute
(its outline's symmetric difference 0.018 mm^2, the radial roots within 0.007 mm, +0.43 mm^3)."""
import math

import pytest

from tests import built
from tests.helpers import is_inside

# name -> (volume of the export's solids fused, its bbox (min, max), the build's tolerance on that volume)
EXPORT = {
    "gripper_base": (13669.4152, ((-40.0, -18.0, 0.0), (40.0, 18.0, 22.0)), 0.01),
    "gripper_rack": (2737.6017, ((-28.0, 6.2955, 0.0), (28.0, 16.0, 10.3)), 0.05),
    "gripper_jaw": (5878.9384, ((-28.0, -8.0, 0.0), (28.0, 16.0, 45.0)), 1.6),
    "gripper_pinion": (833.9734, ((-8.16, -8.0297, 0.0), (8.16, 8.0297, 6.5)), 0.6),
}
# the radii of the export's cylinders (every one, either way: none missing, none extra)
RADII = {
    "gripper_base": (1.6, 2.1),
    "gripper_rack": (0.4855, 1.65, 1.6807, 2.0, 3.1, 4.5222),
    "gripper_jaw": (1.0, 1.6, 2.0, 4.0, 5.0),
    "gripper_pinion": (0.6, 1.6, 2.85, 3.1, 5.8652, 8.16),
}


def _radii(shape) -> list[float]:
    from OCP.BRepAdaptor import BRepAdaptor_Surface
    from OCP.GeomAbs import GeomAbs_Cylinder

    out = []
    for f in shape.faces():
        s = BRepAdaptor_Surface(f.wrapped)
        if s.GetType() == GeomAbs_Cylinder:
            out.append(s.Cylinder().Radius())
    return out


@pytest.mark.slow
@pytest.mark.parametrize("name", sorted(EXPORT))
def test_matches_the_export(name):
    part = built.part(name)
    volume, (lo, hi), tol = EXPORT[name]
    assert part.is_valid and len(part.solids()) == 1
    assert part.volume == pytest.approx(volume, abs=tol)
    bb = part.bounding_box()
    assert (*bb.min, *bb.max) == pytest.approx((*lo, *hi), abs=3e-3)
    found = _radii(part)
    for r in RADII[name]:
        assert any(abs(f - r) < 1e-3 for f in found), (name, "missing a cylinder of radius", r)
    for f in found:
        assert any(abs(f - r) < 1e-3 for r in RADII[name]), (name, "a cylinder the export has not", f)


@pytest.mark.slow
def test_base_features():
    b = built.part("gripper_base")
    assert is_inside(b, 0.0, 0.0, 2.0) and not is_inside(b, 0.0, 0.0, 4.2)                  # the plate, 4 thick
    for s in (1.0, -1.0):
        assert not is_inside(b, 0.0, s * 12.375, 2.0)                                       # the slots ...
        assert not is_inside(b, 35.8, s * 12.375, 2.0) and is_inside(b, 36.2, s * 12.375, 2.0)   # ... ending at 36
        assert is_inside(b, 0.0, s * 17.1, 8.8) and not is_inside(b, 0.0, s * 17.1, 9.2)       # the walls, 9 high
        assert not is_inside(b, 0.0, s * 11.0, 6.0)                                         # each rack's channel
        assert is_inside(b, -20.0, s * 5.35, 6.0) and not is_inside(b, 0.0, s * 5.35, 6.0)    # the ribs, the pinion between
    for x, hole_x in ((-12.4, -13.0), (34.4, 35.0)):                                        # the posts
        assert is_inside(b, x, 0.0, 10.0) and is_inside(b, x, 9.3, 21.0)                    # the block, flared to the top
        assert not is_inside(b, x, 0.0, 21.5) and not is_inside(b, x, 0.0, 22.2)            # the tab rib's slot; the top at 22
        for s in (1.0, -1.0):
            assert not is_inside(b, hole_x, s * 5.0, 15.0) and not is_inside(b, hole_x, s * 5.0, 9.5)   # the screw holes
            assert is_inside(b, hole_x, s * 5.0, 8.5)                                       # ... down to 9
            assert not is_inside(b, hole_x, s * 6.5, 19.0)                                  # the nut traps


@pytest.mark.slow
def test_rack_features():
    r = built.part("gripper_rack")
    for k in range(-8, 9):                                                                  # 17 teeth, 3.31 apart
        assert is_inside(r, k * 3.31, 6.4, 3.0) and not is_inside(r, k * 3.31 + 1.655, 7.0, 3.0)
    assert not is_inside(r, 9 * 3.31, 6.4, 3.0)
    assert is_inside(r, 0.0, 12.0, 3.0) and not is_inside(r, 0.0, 12.0, 6.2)               # the bar, 6 thick
    for x in (-20.39, 20.41):                                                               # the posts: split, bored, the head sunk
        assert is_inside(r, x - 3.0, 12.375, 10.0) and is_inside(r, x + 3.0, 12.375, 10.0)
        assert not is_inside(r, x, 11.0, 8.0) and not is_inside(r, x, 12.375, 4.0)
        assert not is_inside(r, x + 2.5, 12.375, 1.0) and is_inside(r, x + 2.5, 12.375, 3.5)


@pytest.mark.slow
def test_jaw_features():
    j = built.part("gripper_jaw")
    assert is_inside(j, 11.6, 0.0, 20.0) and not is_inside(j, 11.4, 0.0, 20.0)             # the gripping face at 11.5
    back = 20.1132 - (20.0 - 4.1251) * 0.2009 / 0.9796 + 5.0 / 0.9796                     # the back's round at z 20
    assert is_inside(j, back - 0.15, 0.0, 20.0) and not is_inside(j, back + 0.15, 0.0, 20.0)
    assert is_inside(j, 12.0, 0.0, 44.9) and not is_inside(j, 12.0, 0.0, 45.1)             # the top at 45
    assert not is_inside(j, 12.0, 4.9, 44.9)                                                # ... rounded across
    for bx, by in ((-20.3901, 12.3735), (20.4076, 12.375)):                                 # the nut bosses
        assert not is_inside(j, bx, by + 2.8, 4.5) and is_inside(j, bx, by + 3.0, 4.5)      # a hex 5.8 across flats
        assert not is_inside(j, bx, by, 1.5) and is_inside(j, bx + 1.8, by, 1.5)            # the M3 hole under it
    assert not is_inside(j, 27.5, 2.0, 1.0) and is_inside(j, 25.5, 2.0, 1.0)                # the ramp under the foot's end


@pytest.mark.slow
def test_pinion_features():
    p = built.part("gripper_pinion")
    half = math.pi / 14
    for k in range(14):                                                                     # 14 teeth, one on +x
        a = 2 * half * k
        assert is_inside(p, 8.1 * math.cos(a), 8.1 * math.sin(a), 3.0)
        assert not is_inside(p, 6.0 * math.cos(a + half), 6.0 * math.sin(a + half), 3.0)    # the gaps to the root at 5.865
        assert is_inside(p, 5.7 * math.cos(a + half), 5.7 * math.sin(a + half), 3.0)
    assert not is_inside(p, 3.0, 0.0, 1.0) and not is_inside(p, 1.5, 0.0, 2.8)             # the head's recess, the hole
    assert is_inside(p, 2.0, 0.0, 2.8) and not is_inside(p, 2.8, 0.0, 5.0) and is_inside(p, 2.95, 0.0, 5.0)   # the spline bore
