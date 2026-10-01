"""Eccentric shaft (ported from cycloidal_drive tests/test_eccentric_shaft.py).

  1. Dimensional checks - thin wall, lobe offsets, bearing fits, D-bore (config only)
  2. The built solid - topology, bounding box, volume sanity
  3. The support-pin hole
"""
import math

import pytest

import parts
from lib import reference as R
from lib.cycloidal import stack_positions
from tests.cycloidal.helpers import CFG

cycloidal_eccentric_shaft = parts.load("cycloidal_eccentric_shaft")


class TestShaftDimensions:

    def test_thin_wall_minimum(self):
        """Thin-side wall between the spine corridor and the lobe surface >= 4 mm (spec 4.5)."""
        shaft = CFG.shaft
        thin_wall = shaft.bearing_seat_od / 2.0 - shaft.spine_od / 2.0 - shaft.eccentricity
        assert thin_wall >= 4.0, f"Thin-side wall = {thin_wall:.2f}mm, need >= 4mm"

    def test_lobes_180_degrees_apart(self):
        assert CFG.shaft.eccentricity == CFG.gear.eccentricity

    def test_lobe_fits_in_6003_bearing(self):
        """Lobe OD (17.10) >= 6003 bore (17) for a light press, by at most 0.2."""
        assert CFG.shaft.bearing_seat_od >= CFG.bearings.ecc_bore
        assert CFG.shaft.bearing_seat_od - CFG.bearings.ecc_bore <= 0.2

    def test_shaft_spans_both_discs(self):
        stack, disc_t = CFG.stack_up, CFG.disc.thickness
        z_start, z_end = stack.z_motor_plate_inner, stack.z_disc2 + disc_t      # 9, 35
        assert z_start < stack.z_disc1
        assert z_end >= stack.z_disc2 + disc_t

    def test_lobe_engulfs_spine(self):
        """Lobe radius > eccentricity + spine radius for a watertight union."""
        shaft = CFG.shaft
        assert shaft.bearing_seat_od / 2.0 > shaft.eccentricity + shaft.spine_od / 2.0

    def test_retention_bridge_clears_disc_bore(self):
        """The 23.10 bridge flange, offset by e, keeps >= 1 mm to the 35.10 disc bore."""
        shaft = CFG.shaft
        clearance = CFG.disc.center_bore_dia / 2.0 - (shaft.bridge_flange_od / 2.0 + shaft.eccentricity)
        assert clearance >= 1.0, f"Bridge flange clearance to disc bore = {clearance:.2f}mm, need >= 1mm"

    def test_retention_bridge_wider_than_bearing_bore(self):
        assert CFG.shaft.bridge_flange_od > CFG.bearings.ecc_bore


class TestDBore:

    def test_collar_contains_bore(self):
        shaft, tol = CFG.shaft, CFG.tolerances
        bore_r = (shaft.d_bore_dia + tol.d_bore_clearance_add * 2) / 2.0
        wall = shaft.input_collar_od / 2.0 - bore_r
        assert wall >= 1.5, f"Collar wall around D-bore = {wall:.2f}mm, need >= 1.5mm"

    def test_bore_does_not_break_lobe_thin_side(self):
        shaft, tol = CFG.shaft, CFG.tolerances
        bore_r = (shaft.d_bore_dia + tol.d_bore_clearance_add * 2) / 2.0
        thin_wall = shaft.bearing_seat_od / 2.0 - shaft.eccentricity - bore_r
        assert thin_wall >= 2.0, f"Lobe thin-side wall around D-bore = {thin_wall:.2f}mm, need >= 2mm"

    def test_bore_matches_motor_shaft(self):
        shaft, tol = CFG.shaft, CFG.tolerances
        assert shaft.d_bore_dia + tol.d_bore_clearance_add * 2 > CFG.motor.shaft_dia

    def test_bore_depth_gives_adequate_engagement(self):
        assert CFG.shaft.d_bore_depth >= 8.0


@pytest.fixture(scope="module")
def shaft_solid():
    return cycloidal_eccentric_shaft.build()


@pytest.mark.slow
class TestSolid:

    def test_solid_is_valid(self, shaft_solid):
        assert len(shaft_solid.solids()) == 1
        assert shaft_solid.is_valid

    def test_bounding_box_z(self, shaft_solid):
        """Built at its stack position: z 9..35 (26 long)."""
        stack = CFG.stack_up
        z_start, z_end = stack.z_motor_plate_inner, stack.z_disc2 + CFG.disc.thickness
        bb = shaft_solid.bounding_box()
        assert abs(bb.size.Z - (z_end - z_start)) < 0.1
        assert abs(bb.min.Z - z_start) < 0.1

    def test_bounding_box_xy(self, shaft_solid):
        """The bridge flange is the widest feature: X span = 2 (flange_r + e), Y span = 2 flange_r."""
        flange_r, e = CFG.shaft.bridge_flange_od / 2.0, CFG.shaft.eccentricity
        size = shaft_solid.bounding_box().size
        assert abs(size.X - 2 * (flange_r + e)) < 0.1, f"X extent {size.X:.2f}mm, expected {2 * (flange_r + e):.2f}mm"
        assert abs(size.Y - 2 * flange_r) < 0.1, f"Y extent {size.Y:.2f}mm, expected {2 * flange_r:.2f}mm"

    def test_volume_sanity(self, shaft_solid):
        """Between the bare spine and a full lobe-OD cylinder over the whole length."""
        shaft, stack = CFG.shaft, CFG.stack_up
        total_length = stack.z_disc2 + CFG.disc.thickness - stack.z_motor_plate_inner
        vol = R.solid_volume(shaft_solid)
        lower = math.pi * (shaft.spine_od / 2.0) ** 2 * total_length
        upper = math.pi * (shaft.bearing_seat_od / 2.0) ** 2 * total_length
        assert lower < vol < upper, f"Volume {vol:.0f}mm^3 outside [{lower:.0f}, {upper:.0f}]"


class TestOutputPinHole:

    def test_pin_hole_within_lobe(self):
        """The pin hole (on the shaft axis) stays >= 4 mm inside lobe 2 (centre at -e)."""
        shaft, tol = CFG.shaft, CFG.tolerances
        pin_bore_r = (shaft.support_pin_dia + tol.dowel_bore_clearance_add * 2) / 2.0
        wall = shaft.bearing_seat_od / 2.0 - shaft.eccentricity - pin_bore_r
        assert wall >= 4.0, f"Wall around pin hole = {wall:.2f}mm, need >= 4mm"

    def test_pin_hole_clears_d_bore(self):
        """At least 1 mm of material between the pin hole bottom (z 24) and the D-bore end (z 23)."""
        stack = CFG.stack_up
        z_pin_hole_bottom = stack.z_disc2 + CFG.disc.thickness - CFG.shaft.support_pin_hole_depth
        z_d_bore_end = stack.z_motor_plate_inner + CFG.shaft.d_bore_depth
        assert z_pin_hole_bottom - z_d_bore_end >= 1.0

    def test_pin_reaches_625_bearing(self):
        """The pin's protrusion (9) spans the gap to the 625 (the turning shell's 4: the hub's flange) plus the 5 mm
        625 bearing."""
        shaft, stack = CFG.shaft, CFG.stack_up
        protrusion = shaft.support_pin_length - shaft.support_pin_hole_depth
        gap = stack_positions(CFG)["z_625"] - (stack.z_disc2 + CFG.disc.thickness)
        assert protrusion >= gap + CFG.bearings.inp_width

    def test_pin_fits_625_bore(self):
        assert CFG.shaft.support_pin_dia <= CFG.bearings.inp_bore
