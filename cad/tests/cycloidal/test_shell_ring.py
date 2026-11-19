"""Shell ring (parts/cycloidal/cycloidal_shell_ring): the turning shell's motor end, designed here with no reference - its
numbers are here (volume, box), and its features probed: the lip, the motor-end 6814's seat, the pin ring round the held
motor plate with the ring pins' blind holes, the housing bolts' holes and counterbores, the pillars and windows from end
to end. The body's mirror: tests/cycloidal/test_shell_body.py."""
import math

import pytest
from build123d import Axis

import parts
from lib import reference as R
from lib.cycloidal import (
    compute_housing_bolt_angles,
    housing_bolt_points,
    ring_pin_hole_dia,
    ring_pin_points,
    shell_ends,
    stack_positions,
)
from tests.cycloidal.helpers import CFG
from tests.helpers import is_inside

cycloidal_shell_ring = parts.load("cycloidal_shell_ring")
T = CFG.stack_up.z_motor_plate_inner                           # 9: the motor plate's inner face, the body's
END = shell_ends(CFG)[0]                                       # -13


@pytest.fixture(scope="module")
def ring():
    return cycloidal_shell_ring.build()


def _radial(solid, x, y, d, z):
    """Is the point d out from (x, y) along its radius from the drive's axis (d < 0: in), at z, inside the solid?"""
    a = math.atan2(y, x)
    return is_inside(solid, x + d * math.cos(a), y + d * math.sin(a), z)


@pytest.mark.slow
def test_numbers(ring):
    """One solid; the housing's od and pillars across, the shell's motor end to the body's face along the axis."""
    assert len(ring.solids()) == 1 and ring.is_valid
    assert R.solid_volume(ring) == pytest.approx(75782.663, abs=0.5)
    bb = ring.bounding_box()
    assert (bb.min.Z, bb.max.Z) == pytest.approx((END, T), abs=1e-6)
    bb = ring.rotate(Axis.Z, -CFG.housing.bolt_start_deg).bounding_box()                 # its first pillar on X
    assert (bb.size.X, bb.size.Y) == pytest.approx((CFG.housing.od, 116.023), abs=1e-3)   # as the body's


@pytest.mark.slow
def test_bore(ring):
    """From the end in: the lip (the 6814's outer race's stop), the seat, the pin ring round the held plate - each
    open inside its bore, solid just outside it; open on the axis for the sleeve."""
    sh, h, b = CFG.shell, CFG.housing, CFG.bearings
    for r, z in ((h.lip_bore_dia / 2.0, END + sh.end_lip / 2.0), (h.output_bearing_seat_dia / 2.0, -b.out_width / 2.0),
                 (sh.ring_bore_dia / 2.0, T / 2.0)):
        assert not is_inside(ring, r - 0.1, 0, z) and is_inside(ring, r + 0.3, 0, z), (r, z)
    assert not is_inside(ring, 0, 0, -b.out_width / 2.0)


@pytest.mark.slow
def test_ring_pins_and_housing_bolts(ring):
    """The ring pins' blind holes from the face on the body down past the pins' ends (open at the face, solid under
    their bottoms), each one's lead-in at its mouth - open to its rim on both sides (toward the bore and toward the
    window), solid just past it; the housing bolts' holes through, their heads' counterbores in the end face."""
    h = CFG.housing
    pins_z = stack_positions(CFG)["z_ring_pins"]                  # 4
    hole_r = ring_pin_hole_dia(CFG) / 2.0
    mouth_r, depth = hole_r + h.ring_pin_entry_chamfer_add / 2.0, h.ring_pin_entry_chamfer_depth
    for x, y in ring_pin_points(CFG):
        assert not is_inside(ring, x, y, T - 0.5) and not is_inside(ring, x, y, pins_z + 0.1)
        assert is_inside(ring, x, y, pins_z - 1.0)                 # the hole's bottom
        assert _radial(ring, x, y, hole_r + 0.3, T - depth - 0.5)  # the hole's wall below its lead-in
        for side in (1.0, -1.0):
            assert not _radial(ring, x, y, side * (mouth_r - 0.05), T - 0.02) and _radial(ring, x, y, side * (mouth_r + 0.15), T - 0.02)
    for x, y in housing_bolt_points(CFG):
        assert not is_inside(ring, x, y, T - 0.5) and not is_inside(ring, x, y, END + 6.0)
        off, a = (h.bolt_dia + 0.4) / 2.0 + 0.6, math.atan2(y, x) + math.pi / 2.0
        assert not is_inside(ring, x + off * math.cos(a), y + off * math.sin(a), END + 1.0)                          # counterbore
        assert is_inside(ring, x + off * math.cos(a), y + off * math.sin(a), END + h.bolt_counterbore_depth + 1.0)


@pytest.mark.slow
def test_windows_end_to_end(ring):
    """Midway between the pillars, past the bore's radius: void at the pin ring, the seat and the lip alike."""
    h = CFG.housing
    step, r = 2 * math.pi / h.bolt_count, (h.bore_dia + h.od) / 4.0
    for a in compute_housing_bolt_angles(CFG):
        for z in (END + 1.5, -5.0, 4.5):
            assert not is_inside(ring, r * math.cos(a + step / 2.0), r * math.sin(a + step / 2.0), z), (math.degrees(a), z)
