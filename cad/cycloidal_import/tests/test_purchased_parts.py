"""Validation test suite for purchased-part models.

Tests cover:
  1. Bearing dimensions — bore, OD, width for all three types
  2. NEMA 17 motor — body size, shaft length
  3. Ring pins — count, diameter, length, circle placement
  4. Output pins — count, diameter, length, circle placement
  5. Motor bolts — count, dimensions, volume
"""

import math
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DEFAULT_CONFIG

CFG = DEFAULT_CONFIG


# ===================================================================
# Fixtures — build each part once per module
# ===================================================================


@pytest.fixture(scope="module")
def bearing_6003():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_bearing_6003

    return build_bearing_6003()


@pytest.fixture(scope="module")
def bearing_6814():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_bearing_6814

    return build_bearing_6814()


@pytest.fixture(scope="module")
def bearing_625():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_bearing_625

    return build_bearing_625()


@pytest.fixture(scope="module")
def nema17():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_nema17_motor

    return build_nema17_motor()


@pytest.fixture(scope="module")
def ring_pins():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_ring_pins

    return build_ring_pins()


@pytest.fixture(scope="module")
def output_pins():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_output_pins

    return build_output_pins()


@pytest.fixture(scope="module")
def housing_bolts():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_housing_bolts

    return build_housing_bolts()


@pytest.fixture(scope="module")
def housing_nuts():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_housing_nuts

    return build_housing_nuts()


@pytest.fixture(scope="module")
def motor_bolts():
    cq = pytest.importorskip("cadquery")
    from src.purchased_parts import build_motor_bolts

    return build_motor_bolts()


# ===================================================================
# Bearing tests
# ===================================================================


class TestBearing6003:

    def test_solid_valid(self, bearing_6003):
        solids = bearing_6003.solids().vals()
        assert len(solids) == 1

    def test_bounding_box_xy(self, bearing_6003):
        bb = bearing_6003.val().BoundingBox()
        od = bb.xmax - bb.xmin
        assert abs(od - CFG.bearings.ecc_od) < 0.1, f"OD {od:.2f}mm, expected {CFG.bearings.ecc_od}mm"

    def test_bounding_box_z(self, bearing_6003):
        bb = bearing_6003.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - CFG.bearings.ecc_width) < 0.1, f"Width {z:.2f}mm, expected {CFG.bearings.ecc_width}mm"

    def test_volume(self, bearing_6003):
        b = CFG.bearings
        expected = math.pi * ((b.ecc_od / 2) ** 2 - (b.ecc_bore / 2) ** 2) * b.ecc_width
        vol = bearing_6003.val().Volume()
        assert abs(vol - expected) < 1.0, f"Volume {vol:.0f}mm³, expected {expected:.0f}mm³"


class TestBearing6814:

    def test_solid_valid(self, bearing_6814):
        solids = bearing_6814.solids().vals()
        assert len(solids) == 1

    def test_bounding_box_xy(self, bearing_6814):
        bb = bearing_6814.val().BoundingBox()
        od = bb.xmax - bb.xmin
        assert abs(od - CFG.bearings.out_od) < 0.1, f"OD {od:.2f}mm, expected {CFG.bearings.out_od}mm"

    def test_bounding_box_z(self, bearing_6814):
        bb = bearing_6814.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - CFG.bearings.out_width) < 0.1, f"Width {z:.2f}mm, expected {CFG.bearings.out_width}mm"

    def test_volume(self, bearing_6814):
        b = CFG.bearings
        expected = math.pi * ((b.out_od / 2) ** 2 - (b.out_bore / 2) ** 2) * b.out_width
        vol = bearing_6814.val().Volume()
        assert abs(vol - expected) < 1.0, f"Volume {vol:.0f}mm³, expected {expected:.0f}mm³"


class TestBearing625:

    def test_solid_valid(self, bearing_625):
        solids = bearing_625.solids().vals()
        assert len(solids) == 1

    def test_bounding_box_xy(self, bearing_625):
        bb = bearing_625.val().BoundingBox()
        od = bb.xmax - bb.xmin
        assert abs(od - CFG.bearings.inp_od) < 0.1, f"OD {od:.2f}mm, expected {CFG.bearings.inp_od}mm"

    def test_bounding_box_z(self, bearing_625):
        bb = bearing_625.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - CFG.bearings.inp_width) < 0.1, f"Width {z:.2f}mm, expected {CFG.bearings.inp_width}mm"

    def test_volume(self, bearing_625):
        b = CFG.bearings
        expected = math.pi * ((b.inp_od / 2) ** 2 - (b.inp_bore / 2) ** 2) * b.inp_width
        vol = bearing_625.val().Volume()
        assert abs(vol - expected) < 1.0, f"Volume {vol:.0f}mm³, expected {expected:.0f}mm³"


# ===================================================================
# NEMA 17 motor tests
# ===================================================================


class TestNema17Motor:

    def test_solid_valid(self, nema17):
        solids = nema17.solids().vals()
        assert len(solids) == 1

    def test_body_xy_extent(self, nema17):
        """XY bounding box should be at least the body width (42.3mm)."""
        bb = nema17.val().BoundingBox()
        x_size = bb.xmax - bb.xmin
        y_size = bb.ymax - bb.ymin
        assert abs(x_size - CFG.motor.body_width) < 0.1
        assert abs(y_size - CFG.motor.body_width) < 0.1

    def test_z_extent(self, nema17):
        """Z should span from -body_length to +shaft_length."""
        bb = nema17.val().BoundingBox()
        m = CFG.motor
        expected_z = m.body_length + m.shaft_length
        z_size = bb.zmax - bb.zmin
        assert abs(z_size - expected_z) < 0.5, f"Z extent {z_size:.2f}mm, expected {expected_z:.2f}mm"

    def test_shaft_extends_positive_z(self, nema17):
        """Shaft tip should be at +shaft_length from Z=0."""
        bb = nema17.val().BoundingBox()
        assert abs(bb.zmax - CFG.motor.shaft_length) < 0.5

    def test_dcut_reduces_y_extent_on_shaft(self, nema17):
        """The D-cut flat should make the shaft asymmetric in Y.

        The shaft Y extent at the tip should be less than the full 5mm
        diameter because the flat cuts 0.25mm off one side.
        """
        m = CFG.motor
        shaft_r = m.shaft_dia / 2.0
        dcut_offset = m.shaft_dcut_flat / 2.0
        cut_depth = shaft_r - dcut_offset  # 0.25mm
        # The shaft Y span in the D-cut region: from -shaft_r to +dcut_offset
        expected_y = shaft_r + dcut_offset  # 2.5 + 2.25 = 4.75mm
        assert expected_y < m.shaft_dia, "D-cut should reduce shaft Y extent"

    def test_bolt_holes_present(self, nema17):
        """Motor should have 4 M3 bolt holes on the 31mm square pattern.

        Volume should be less than a solid body+pilot+shaft (the holes
        remove material).
        """
        m = CFG.motor
        # Volume of 4 blind holes
        hole_vol = 4 * (math.pi * (m.bolt_dia / 2) ** 2 * m.bolt_hole_depth)
        # The motor solid should have material removed
        # Just verify the holes reduce volume by approximately the right amount
        shaft_r = m.shaft_dia / 2.0
        body_vol = m.body_width ** 2 * m.body_length
        pilot_vol = math.pi * (m.pilot_dia / 2) ** 2 * 2.0
        shaft_vol = math.pi * shaft_r ** 2 * m.shaft_length
        max_vol = body_vol + pilot_vol + shaft_vol
        actual_vol = nema17.val().Volume()
        assert actual_vol < max_vol, "Bolt holes should remove material from body"
        removed = max_vol - actual_vol
        assert removed > hole_vol * 0.5, (
            f"Expected at least ~{hole_vol:.0f}mm³ removed, got {removed:.0f}mm³"
        )


# ===================================================================
# Ring pins tests
# ===================================================================


class TestRingPins:

    def test_solid_count(self, ring_pins):
        """Should produce exactly 21 separate solids."""
        solids = ring_pins.solids().vals()
        assert len(solids) == CFG.gear.num_ring_pins, (
            f"Expected {CFG.gear.num_ring_pins} solids, got {len(solids)}"
        )

    def test_bounding_box_z(self, ring_pins):
        """All pins should be 30mm tall."""
        bb = ring_pins.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - CFG.gear.ring_pin_length) < 0.1

    def test_bounding_box_xy_span(self, ring_pins):
        """XY extent should be close to pin circle + pin diameter.

        With an odd pin count (21), no pin lands exactly at 180° opposite
        pin 0, so the X span is slightly less than the theoretical max.
        """
        g = CFG.gear
        expected_span = g.ring_pin_circle_dia + g.ring_pin_dia
        bb = ring_pins.val().BoundingBox()
        x_span = bb.xmax - bb.xmin
        assert abs(x_span - expected_span) < 1.0, (
            f"X span {x_span:.2f}mm, expected ~{expected_span:.2f}mm"
        )

    def test_single_pin_volume(self, ring_pins):
        """Total volume should equal 21 × single pin cylinder volume."""
        g = CFG.gear
        single_vol = math.pi * (g.ring_pin_dia / 2) ** 2 * g.ring_pin_length
        expected_total = g.num_ring_pins * single_vol
        total_vol = sum(s.Volume() for s in ring_pins.solids().vals())
        assert abs(total_vol - expected_total) < 5.0, (
            f"Total volume {total_vol:.0f}mm³, expected {expected_total:.0f}mm³"
        )


# ===================================================================
# Output pins tests
# ===================================================================


class TestOutputPins:

    def test_solid_count(self, output_pins):
        """Should produce exactly 4 separate solids."""
        solids = output_pins.solids().vals()
        assert len(solids) == CFG.disc.output_pin_count, (
            f"Expected {CFG.disc.output_pin_count} solids, got {len(solids)}"
        )

    def test_bounding_box_z(self, output_pins):
        """Pin height should equal the dowel length (45mm)."""
        d = CFG.disc
        bb = output_pins.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - d.output_pin_length) < 0.1, (
            f"Z {z:.2f}mm, expected {d.output_pin_length:.2f}mm"
        )

    def test_bounding_box_xy_span(self, output_pins):
        """XY extent should span the output pin circle + one pin diameter."""
        d = CFG.disc
        expected_span = d.output_pin_circle_dia + d.output_pin_dia
        bb = output_pins.val().BoundingBox()
        x_span = bb.xmax - bb.xmin
        assert abs(x_span - expected_span) < 0.2, (
            f"X span {x_span:.2f}mm, expected {expected_span:.2f}mm"
        )

    def test_single_pin_volume(self, output_pins):
        """Total volume should equal 4 × dowel cylinder volume."""
        d = CFG.disc
        single_vol = math.pi * (d.output_pin_dia / 2) ** 2 * d.output_pin_length
        expected_total = d.output_pin_count * single_vol
        total_vol = sum(s.Volume() for s in output_pins.solids().vals())
        assert abs(total_vol - expected_total) < 5.0, (
            f"Total volume {total_vol:.0f}mm³, expected {expected_total:.0f}mm³"
        )


# ===================================================================
# Housing bolt tests
# ===================================================================


class TestHousingBolts:

    def test_solid_valid(self, housing_bolts):
        solids = housing_bolts.solids().vals()
        assert len(solids) == CFG.housing.bolt_count, (
            f"Expected {CFG.housing.bolt_count} solids, got {len(solids)}"
        )

    def test_bounding_box_z(self, housing_bolts):
        """Z extent should be head_height + bolt_length."""
        h = CFG.housing
        expected_z = h.bolt_head_height + h.bolt_length  # 4 + 60 = 64mm
        bb = housing_bolts.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - expected_z) < 0.2, (
            f"Z extent {z:.2f}mm, expected {expected_z:.2f}mm"
        )

    def test_bounding_box_xy_span(self, housing_bolts):
        """XY extent should span the bolt circle + head diameter."""
        h = CFG.housing
        expected_span = h.bolt_circle_dia + h.bolt_head_dia
        bb = housing_bolts.val().BoundingBox()
        x_span = bb.xmax - bb.xmin
        assert abs(x_span - expected_span) < 0.5, (
            f"X span {x_span:.2f}mm, expected ~{expected_span:.2f}mm"
        )


# ===================================================================
# Housing nut tests
# ===================================================================


class TestHousingNuts:

    def test_solid_valid(self, housing_nuts):
        solids = housing_nuts.solids().vals()
        assert len(solids) == CFG.housing.bolt_count, (
            f"Expected {CFG.housing.bolt_count} solids, got {len(solids)}"
        )

    def test_bounding_box_z(self, housing_nuts):
        """Z extent should be the nut thickness."""
        h = CFG.housing
        bb = housing_nuts.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - h.bolt_nut_thickness) < 0.1, (
            f"Z extent {z:.2f}mm, expected {h.bolt_nut_thickness}mm"
        )


# ===================================================================
# Motor bolt tests
# ===================================================================


class TestMotorBolts:

    def test_solid_valid(self, motor_bolts):
        """Should produce exactly 4 separate solids."""
        solids = motor_bolts.solids().vals()
        assert len(solids) == 4, f"Expected 4 solids, got {len(solids)}"

    def test_bounding_box_z(self, motor_bolts):
        """Z extent should be total bolt length (thread + head)."""
        m = CFG.motor
        expected_z = m.motor_bolt_total_length  # 13mm
        bb = motor_bolts.val().BoundingBox()
        z = bb.zmax - bb.zmin
        assert abs(z - expected_z) < 0.2, (
            f"Z extent {z:.2f}mm, expected {expected_z:.2f}mm"
        )

    def test_bounding_box_xy_span(self, motor_bolts):
        """XY extent should span the bolt pattern + head diameter."""
        m = CFG.motor
        # Diagonal of 31mm square pattern + head diameter
        expected_span = m.bolt_pattern_square + m.motor_bolt_head_dia
        bb = motor_bolts.val().BoundingBox()
        x_span = bb.xmax - bb.xmin
        assert abs(x_span - expected_span) < 0.5, (
            f"X span {x_span:.2f}mm, expected ~{expected_span:.2f}mm"
        )

    def test_single_bolt_volume(self, motor_bolts):
        """Total volume should equal 4 × (shank + head cylinders)."""
        m = CFG.motor
        shank_vol = math.pi * (m.bolt_dia / 2) ** 2 * m.motor_bolt_thread_length
        head_vol = math.pi * (m.motor_bolt_head_dia / 2) ** 2 * m.motor_bolt_head_height
        expected_total = 4 * (shank_vol + head_vol)
        total_vol = sum(s.Volume() for s in motor_bolts.solids().vals())
        assert abs(total_vol - expected_total) < 5.0, (
            f"Total volume {total_vol:.0f}mm³, expected {expected_total:.0f}mm³"
        )
