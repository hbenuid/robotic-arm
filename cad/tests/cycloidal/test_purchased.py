"""Purchased-part envelopes of the cycloidal drive (ported from cycloidal_drive
tests/test_purchased_parts.py): bearings, NEMA 17, ring / output pins, bolts, nuts.

The envelopes are the drive repo's simplified models and the reference STEPs; the house
test_cots_* tests compare any step.parts vendor model against them."""
import math

import pytest

import parts
from lib import reference as R
from tests.cycloidal.helpers import CFG

bearing_625 = parts.load("bearing_625")
bearing_6003 = parts.load("bearing_6003")
bearing_6814 = parts.load("bearing_6814")
cycloidal_housing_bolts = parts.load("cycloidal_housing_bolts")
cycloidal_housing_nuts = parts.load("cycloidal_housing_nuts")
cycloidal_motor_bolts = parts.load("cycloidal_motor_bolts")
cycloidal_output_pins = parts.load("cycloidal_output_pins")
cycloidal_ring_pins = parts.load("cycloidal_ring_pins")
nema17_48mm = parts.load("nema17_48mm")
pytestmark = pytest.mark.slow


@pytest.fixture(scope="module")
def b6003():
    return bearing_6003._envelope()


@pytest.fixture(scope="module")
def b6814():
    return bearing_6814._envelope()


@pytest.fixture(scope="module")
def b625():
    return bearing_625._envelope()


@pytest.fixture(scope="module")
def nema17():
    return nema17_48mm._envelope()


@pytest.fixture(scope="module")
def ring_pins():
    return cycloidal_ring_pins._envelope()


@pytest.fixture(scope="module")
def output_pins():
    return cycloidal_output_pins._envelope()


@pytest.fixture(scope="module")
def housing_bolts():
    return cycloidal_housing_bolts._envelope()


@pytest.fixture(scope="module")
def housing_nuts():
    return cycloidal_housing_nuts._envelope()


@pytest.fixture(scope="module")
def motor_bolts():
    return cycloidal_motor_bolts._envelope()


def _annulus_checks(part, od, bore, width):
    assert len(part.solids()) == 1
    size = part.bounding_box().size
    assert abs(size.X - od) < 0.1, f"OD {size.X:.2f}mm, expected {od}mm"
    assert abs(size.Z - width) < 0.1, f"Width {size.Z:.2f}mm, expected {width}mm"
    expected = math.pi * ((od / 2) ** 2 - (bore / 2) ** 2) * width
    assert abs(R.solid_volume(part) - expected) < 1.0


class TestBearing6003:
    def test_solid_valid(self, b6003):
        assert len(b6003.solids()) == 1

    def test_bounding_box_xy(self, b6003):
        assert abs(b6003.bounding_box().size.X - CFG.bearings.ecc_od) < 0.1

    def test_bounding_box_z(self, b6003):
        assert abs(b6003.bounding_box().size.Z - CFG.bearings.ecc_width) < 0.1

    def test_volume(self, b6003):
        _annulus_checks(b6003, CFG.bearings.ecc_od, CFG.bearings.ecc_bore, CFG.bearings.ecc_width)


class TestBearing6814:
    def test_solid_valid(self, b6814):
        assert len(b6814.solids()) == 1

    def test_bounding_box_xy(self, b6814):
        assert abs(b6814.bounding_box().size.X - CFG.bearings.out_od) < 0.1

    def test_bounding_box_z(self, b6814):
        assert abs(b6814.bounding_box().size.Z - CFG.bearings.out_width) < 0.1

    def test_volume(self, b6814):
        _annulus_checks(b6814, CFG.bearings.out_od, CFG.bearings.out_bore, CFG.bearings.out_width)


class TestBearing625:
    def test_solid_valid(self, b625):
        assert len(b625.solids()) == 1

    def test_bounding_box_xy(self, b625):
        assert abs(b625.bounding_box().size.X - CFG.bearings.inp_od) < 0.1

    def test_bounding_box_z(self, b625):
        assert abs(b625.bounding_box().size.Z - CFG.bearings.inp_width) < 0.1

    def test_volume(self, b625):
        _annulus_checks(b625, CFG.bearings.inp_od, CFG.bearings.inp_bore, CFG.bearings.inp_width)


class TestNema17Motor:
    def test_solid_valid(self, nema17):
        assert len(nema17.solids()) == 1

    def test_body_xy_extent(self, nema17):
        size = nema17.bounding_box().size
        assert abs(size.X - CFG.motor.body_width) < 0.1
        assert abs(size.Y - CFG.motor.body_width) < 0.1

    def test_z_extent(self, nema17):
        """From -body_length to +shaft_length."""
        m = CFG.motor
        assert abs(nema17.bounding_box().size.Z - (m.body_length + m.shaft_length)) < 0.5

    def test_shaft_extends_positive_z(self, nema17):
        assert abs(nema17.bounding_box().max.Z - CFG.motor.shaft_length) < 0.5

    def test_dcut_reduces_y_extent_on_shaft(self, nema17):
        """The D-flat on +Y: the shaft's Y span is shaft_r + flat offset (4.75) < the 5 mm dia."""
        m = CFG.motor
        assert m.shaft_dia / 2.0 + m.shaft_dcut_flat / 2.0 < m.shaft_dia
        assert nema17.bounding_box().max.Y > m.body_width / 2.0 - 0.01     # the body, not the shaft, sets Y

    def test_bolt_holes_present(self, nema17):
        """The 4 blind M3 holes remove at least half their nominal volume from body+pilot+shaft."""
        m = CFG.motor
        hole_vol = 4 * math.pi * (m.bolt_dia / 2) ** 2 * m.bolt_hole_depth
        max_vol = m.body_width ** 2 * m.body_length + math.pi * (m.pilot_dia / 2) ** 2 * m.pilot_height + math.pi * (m.shaft_dia / 2) ** 2 * m.shaft_length
        removed = max_vol - R.solid_volume(nema17)
        assert removed > hole_vol * 0.5, f"Expected at least ~{hole_vol:.0f}mm^3 removed, got {removed:.0f}mm^3"


class TestRingPins:
    def test_solid_count(self, ring_pins):
        assert len(ring_pins.solids()) == CFG.gear.num_ring_pins

    def test_bounding_box_z(self, ring_pins):
        assert abs(ring_pins.bounding_box().size.Z - CFG.gear.ring_pin_length) < 0.1

    def test_bounding_box_xy_span(self, ring_pins):
        """~ pin circle + pin dia (21 is odd: no pin exactly opposite pin 0, so slightly less)."""
        g = CFG.gear
        assert abs(ring_pins.bounding_box().size.X - (g.ring_pin_circle_dia + g.ring_pin_dia)) < 1.0

    def test_single_pin_volume(self, ring_pins):
        g = CFG.gear
        expected = g.num_ring_pins * math.pi * (g.ring_pin_dia / 2) ** 2 * g.ring_pin_length
        assert abs(R.solid_volume(ring_pins) - expected) < 5.0


class TestOutputPins:
    def test_solid_count(self, output_pins):
        assert len(output_pins.solids()) == CFG.disc.output_pin_count

    def test_bounding_box_z(self, output_pins):
        assert abs(output_pins.bounding_box().size.Z - CFG.disc.output_pin_length) < 0.1

    def test_bounding_box_xy_span(self, output_pins):
        d = CFG.disc
        assert abs(output_pins.bounding_box().size.X - (d.output_pin_circle_dia + d.output_pin_dia)) < 0.2

    def test_single_pin_volume(self, output_pins):
        d = CFG.disc
        expected = d.output_pin_count * math.pi * (d.output_pin_dia / 2) ** 2 * d.output_pin_length
        assert abs(R.solid_volume(output_pins) - expected) < 5.0


class TestHousingBolts:
    def test_solid_valid(self, housing_bolts):
        assert len(housing_bolts.solids()) == CFG.housing.bolt_count

    def test_bounding_box_z(self, housing_bolts):
        """head_height + bolt_length = 4 + 55 = 59."""
        h = CFG.housing
        assert abs(housing_bolts.bounding_box().size.Z - (h.bolt_head_height + h.bolt_length)) < 0.2

    def test_bounding_box_xy_span(self, housing_bolts):
        h = CFG.housing
        assert abs(housing_bolts.bounding_box().size.X - (h.bolt_circle_dia + h.bolt_head_dia)) < 0.5


class TestHousingNuts:
    def test_solid_valid(self, housing_nuts):
        assert len(housing_nuts.solids()) == CFG.housing.bolt_count

    def test_bounding_box_z(self, housing_nuts):
        assert abs(housing_nuts.bounding_box().size.Z - CFG.housing.bolt_nut_thickness) < 0.1


class TestMotorBolts:
    def test_solid_valid(self, motor_bolts):
        assert len(motor_bolts.solids()) == 4

    def test_bounding_box_z(self, motor_bolts):
        assert abs(motor_bolts.bounding_box().size.Z - CFG.motor.motor_bolt_total_length) < 0.2

    def test_bounding_box_xy_span(self, motor_bolts):
        m = CFG.motor
        assert abs(motor_bolts.bounding_box().size.X - (m.bolt_pattern_square + m.motor_bolt_head_dia)) < 0.5

    def test_single_bolt_volume(self, motor_bolts):
        m = CFG.motor
        shank = math.pi * (m.bolt_dia / 2) ** 2 * m.motor_bolt_thread_length
        head = math.pi * (m.motor_bolt_head_dia / 2) ** 2 * m.motor_bolt_head_height
        assert abs(R.solid_volume(motor_bolts) - 4 * (shank + head)) < 5.0
