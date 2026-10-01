"""Shell ring (parts/cycloidal/cycloidal_shell_ring): the turning shell's motor end, designed here with no reference - its
numbers are here (volume, box), and its features probed: the ring round the held motor plate, the ring-pin holes, the
housing bolts' holes and counterbores, the skirt's 6814 seat and lip."""
import math

import pytest

import parts
from lib import reference as R
from lib.cycloidal import housing_bolt_points, ring_pin_hole_dia, ring_pin_points
from tests.cycloidal.helpers import CFG
from tests.helpers import is_inside

cycloidal_shell_ring = parts.load("cycloidal_shell_ring")
T = CFG.stack_up.z_motor_plate_inner                           # 9: the port plate's place
SKIRT_END = -(CFG.bearings.out_width + CFG.shell.skirt_lip)    # -12


@pytest.fixture(scope="module")
def ring():
    return cycloidal_shell_ring.build()


@pytest.mark.slow
def test_numbers(ring):
    """One solid; the housing's od and pillars across, the skirt's end to the body's face along the axis."""
    assert len(ring.solids()) == 1 and ring.is_valid
    assert R.solid_volume(ring) == pytest.approx(45127.781, abs=0.5)
    bb = ring.bounding_box()
    assert (bb.min.Z, bb.max.Z) == pytest.approx((SKIRT_END, T), abs=1e-6)
    assert (bb.size.X, bb.size.Y) == pytest.approx((CFG.housing.od, 116.023), abs=1e-3)   # as the body's (pillars on X)


@pytest.mark.slow
def test_ring(ring):
    """The bore round the held plate; the ring pins through it; the housing bolts' holes with their heads' counterbores
    from the outer face (z=0)."""
    sh, h = CFG.shell, CFG.housing
    z = T / 2.0
    assert not is_inside(ring, sh.ring_bore_dia / 2.0 - 0.3, 0, z) and is_inside(ring, sh.ring_bore_dia / 2.0 + 0.3, 0, z)
    for x, y in ring_pin_points(CFG):
        assert not is_inside(ring, x, y, z)                                        # the pin's hole, through ...
        r = math.hypot(x, y) + ring_pin_hole_dia(CFG) / 2.0 + 0.3
        a = math.atan2(y, x)
        assert is_inside(ring, r * math.cos(a), r * math.sin(a), z)                # ... in the ring
    for x, y in housing_bolt_points(CFG):
        assert not is_inside(ring, x, y, T - 0.5)                                  # the bolt's hole ...
        off = (h.bolt_dia + 0.4) / 2.0 + 0.6                                       # ... past it, in the counterbore
        a = math.atan2(y, x) + math.pi / 2.0
        assert not is_inside(ring, x + off * math.cos(a), y + off * math.sin(a), 1.0)
        assert is_inside(ring, x + off * math.cos(a), y + off * math.sin(a), h.bolt_counterbore_depth + 1.0)


@pytest.mark.slow
def test_skirt(ring):
    """The 6814's seat out_width deep from the ring, the lip past it (the outer race's stop), the skirt's od."""
    sh, h, b = CFG.shell, CFG.housing, CFG.bearings
    mid = -b.out_width / 2.0
    assert not is_inside(ring, h.output_bearing_seat_dia / 2.0 - 0.1, 0, mid)
    assert is_inside(ring, h.output_bearing_seat_dia / 2.0 + 0.3, 0, mid)
    assert is_inside(ring, sh.skirt_od / 2.0 - 0.3, 0, mid) and not is_inside(ring, sh.skirt_od / 2.0 + 0.3, 0, mid)
    lip = SKIRT_END + sh.skirt_lip / 2.0
    assert is_inside(ring, h.lip_bore_dia / 2.0 + 0.3, 0, lip) and not is_inside(ring, h.lip_bore_dia / 2.0 - 0.3, 0, lip)
    assert not is_inside(ring, 0, 0, mid)                                          # open for the sleeve
