"""Motor plate (ported from cycloidal_drive tests/test_motor_plate.py).

  1. Dimensional checks - feature clearances, bolt patterns (config only)
  2. The built solid - the port's housing half (LEGACY_CONFIG, its REFERENCE_BUILD): topology, bounding box, volume sanity
  3. The turning shell's carrier plate (DEFAULT_CONFIG, ShellParams): its numbers (the port's export does not hold
     them) and probes - the plate inside the shell ring, the relief over the 6814's outer race, the sleeve over the motor
"""
import math

import pytest

import parts
from lib import reference as R
from lib.cycloidal import LEGACY_CONFIG, motor_bolt_counterbore_depth, ring_pin_hole_dia, sleeve_end
from tests.cycloidal.helpers import CFG
from tests.helpers import is_inside

cycloidal_motor_plate = parts.load("cycloidal_motor_plate")
PLATE_T = CFG.stack_up.motor_plate_wall + CFG.stack_up.motor_plate_inner_wall    # 9


class TestMotorPlateDimensions:

    def test_motor_bolts_inside_housing(self):
        """The M3 bolt holes (31 mm square) sit well inside the housing OD."""
        m, h, tol = CFG.motor, CFG.housing, CFG.tolerances
        bolt_corner_r = math.sqrt(2) * m.bolt_pattern_square / 2.0
        outermost = bolt_corner_r + (m.bolt_dia + tol.bolt_clearance_add) / 2.0
        assert outermost < h.od / 2.0

    def test_housing_bolts_inside_housing(self):
        h, tol = CFG.housing, CFG.tolerances
        assert h.bolt_circle_dia / 2.0 + (h.bolt_dia + tol.bolt_clearance_add) / 2.0 < h.od / 2.0

    def test_housing_bolts_have_wall_to_od(self):
        h, tol = CFG.housing, CFG.tolerances
        wall = h.od / 2.0 - (h.bolt_circle_dia / 2.0 + (h.bolt_dia + tol.bolt_clearance_add) / 2.0)
        assert wall >= 2.0, f"Wall from bolt edge to OD = {wall:.1f}mm, need >= 2mm"

    def test_counterbore_wall_to_od(self):
        h = CFG.housing
        wall = h.od / 2.0 - (h.bolt_circle_dia / 2.0 + h.bolt_counterbore_dia / 2.0)
        assert wall >= 2.0, f"Counterbore wall to OD = {wall:.2f}mm, need >= 2mm"

    def test_counterbore_fits_in_plate(self):
        assert CFG.housing.bolt_counterbore_depth < PLATE_T

    def test_counterbore_clears_ring_pins(self):
        """The counterbores do not overlap the ring-pin holes radially."""
        h, g = CFG.housing, CFG.gear
        inner_edge = h.bolt_circle_dia / 2.0 - h.bolt_counterbore_dia / 2.0
        pin_outer_r = g.ring_pin_circle_dia / 2.0 + g.ring_pin_dia / 2.0
        assert inner_edge > pin_outer_r

    def test_pilot_recess_smaller_than_bore(self):
        m, h, tol = CFG.motor, CFG.housing, CFG.tolerances
        assert m.pilot_dia + tol.mating_surface_add * 2 < h.bore_dia

    def test_shaft_bore_clears_motor_shaft(self):
        assert CFG.housing.motor_plate_shaft_bore > CFG.motor.shaft_dia

    def test_motor_bolts_clear_of_pilot(self):
        m, tol = CFG.motor, CFG.tolerances
        pilot_r = (m.pilot_dia + tol.mating_surface_add * 2) / 2.0
        clearance = m.bolt_pattern_square / 2.0 - pilot_r - (m.bolt_dia + tol.bolt_clearance_add) / 2.0
        assert clearance > 0, f"Motor bolt overlaps pilot recess, clearance = {clearance:.2f}mm"

    def test_plate_thickness_matches_stackup(self):
        assert PLATE_T == 9.0

    def test_motor_bolt_counterbore_fits_in_plate(self):
        m = CFG.motor
        cb_depth = PLATE_T - (m.motor_bolt_thread_length - (m.bolt_hole_depth - m.motor_bolt_thread_margin))
        assert 0 < cb_depth < PLATE_T
        assert cb_depth == motor_bolt_counterbore_depth(CFG)

    def test_motor_bolt_counterbore_recesses_head(self):
        assert motor_bolt_counterbore_depth(CFG) >= CFG.motor.motor_bolt_head_height

    def test_motor_bolt_engagement(self):
        """The M3 bolts engage 3..bolt_hole_depth mm into the motor body."""
        m = CFG.motor
        engagement = m.motor_bolt_thread_length - (PLATE_T - motor_bolt_counterbore_depth(CFG))
        assert engagement >= 3.0, f"Motor engagement {engagement:.1f}mm < 3mm minimum"
        assert engagement <= m.bolt_hole_depth

    def test_motor_bolt_counterbore_clears_pilot(self):
        m, tol = CFG.motor, CFG.tolerances
        pilot_r = (m.pilot_dia + tol.mating_surface_add * 2) / 2.0
        clearance = m.bolt_pattern_square / 2.0 - pilot_r - (m.motor_bolt_head_dia + tol.bolt_clearance_add) / 2.0
        assert clearance > 0, f"M3 counterbore overlaps pilot recess, clearance = {clearance:.2f}mm"

    def test_ring_pin_holes_are_clearance_fit(self):
        """Through-holes with clearance enable one-at-a-time pin insertion."""
        assert ring_pin_hole_dia(CFG) > CFG.gear.ring_pin_dia


@pytest.fixture(scope="module")
def plate_solid():
    return cycloidal_motor_plate.REFERENCE_BUILD()


@pytest.mark.slow
class TestSolid:

    def test_solid_is_valid(self, plate_solid):
        assert len(plate_solid.solids()) == 1
        assert plate_solid.is_valid

    def test_bounding_box_xy(self, plate_solid):
        """The port's od (8 pillars, one on each axis)."""
        size = plate_solid.bounding_box().size
        assert abs(size.X - LEGACY_CONFIG.housing.od) < 0.2, f"X extent {size.X:.2f}mm, expected {LEGACY_CONFIG.housing.od}mm"
        assert abs(size.Y - LEGACY_CONFIG.housing.od) < 0.2

    def test_bounding_box_z(self, plate_solid):
        assert abs(plate_solid.bounding_box().size.Z - PLATE_T) < 0.1

    def test_volume_sanity(self, plate_solid):
        """Between 85 % of the inner solid disc (bore radius) and the full uncut disc."""
        h = LEGACY_CONFIG.housing
        full_disc_vol = math.pi * (h.od / 2.0) ** 2 * PLATE_T
        floor = math.pi * (h.bore_dia / 2.0) ** 2 * PLATE_T * 0.85
        vol = R.solid_volume(plate_solid)
        assert floor < vol < full_disc_vol, f"Volume {vol:.0f}mm^3 outside ({floor:.0f}, {full_disc_vol:.0f})"

    def test_reveal_windows_present(self, plate_solid):
        """The windows cut most of the outer ring: volume < 85 % of the solid-disc baseline."""
        full_disc_vol = math.pi * (LEGACY_CONFIG.housing.od / 2.0) ** 2 * PLATE_T
        assert R.solid_volume(plate_solid) < full_disc_vol * 0.85, "reveal windows missing?"


@pytest.fixture(scope="module")
def carrier():
    return cycloidal_motor_plate.build()


@pytest.mark.slow
class TestCarrier:
    """DEFAULT's plate: held, inside the turning shell ring, its sleeve back over the motor through the yoke's
    motor-side leg (ShellParams)."""

    def test_numbers(self, carrier):
        sh, b = CFG.shell, CFG.bearings
        assert len(carrier.solids()) == 1 and carrier.is_valid
        assert R.solid_volume(carrier) == pytest.approx(72414.182, abs=0.5)
        bb = carrier.bounding_box()
        assert (bb.min.X, bb.min.Y, bb.min.Z) == pytest.approx((-sh.plate_dia / 2.0, -sh.plate_dia / 2.0, sleeve_end(CFG)), abs=1e-6)
        assert (bb.size.X, bb.size.Y, bb.size.Z) == pytest.approx((sh.plate_dia, sh.plate_dia, PLATE_T - sleeve_end(CFG)), abs=1e-6)
        assert sleeve_end(CFG) == -22.0 and b.out_width < -sleeve_end(CFG)

    def test_relief_seat_and_sleeve(self, carrier):
        """The face cut back over the outer race, the inner race's face inside it; the seat at the hub's grip over the
        last out_width, the sleeve's od behind it; the bore round the motor."""
        sh, b = CFG.shell, CFG.bearings
        r_relief = (sh.plate_relief_dia + sh.plate_dia) / 4.0
        assert not is_inside(carrier, r_relief, 0, sh.plate_relief_depth / 2.0) and is_inside(carrier, r_relief, 0, sh.plate_relief_depth + 0.2)
        assert is_inside(carrier, sh.plate_relief_dia / 2.0 - 0.5, 0, 0.2)
        seat_r, od_r = CFG.output_hub.od / 2.0, sh.sleeve_od / 2.0
        assert is_inside(carrier, (seat_r + od_r) / 2.0, 0, -b.out_width / 2.0)              # the seat (70.3) ...
        assert not is_inside(carrier, (seat_r + od_r) / 2.0, 0, -b.out_width - 5.0)          # ... the sleeve (70) behind it
        assert is_inside(carrier, od_r - 0.3, 0, -b.out_width - 5.0)
        assert not is_inside(carrier, sh.sleeve_bore_dia / 2.0 - 0.3, 0, -20.0) and is_inside(carrier, sh.sleeve_bore_dia / 2.0 + 0.3, 0, -20.0)
        assert not is_inside(carrier, 0, 0, -20.0)                                           # the motor's room
