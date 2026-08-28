"""Eccentric lobes in the 6003 bearings in the discs (ported from cycloidal_drive
tests/test_lobe_bearing_fitment.py).

  1. Parametric fitment - lobe OD vs bearing bore, bearing OD vs disc bore, axial alignment
  2. Eccentric orbit - the lobe centre stays at radius e at every input angle
  3. Boolean interference - shaft lobe vs bearing annulus (designed light press), shaft/bearing vs disc
"""
import math

import pytest
from build123d import Pos

from tests.cycloidal.helpers import interference
from lib.cycloidal import DEFAULT_CONFIG, stack_positions
from lib.cycloidal.profiles import compute_epitrochoid, compute_profile_radii
from parts import bearing_6003, cycloidal_disc_1, cycloidal_eccentric_shaft

CFG = DEFAULT_CONFIG


class TestLobeBearingFitment:

    def test_lobe_od_fits_bearing_bore(self):
        """Lobe OD (17.10) >= 6003 bore (17) for a seat fit, by at most 0.2."""
        assert CFG.shaft.bearing_seat_od >= CFG.bearings.ecc_bore
        assert CFG.shaft.bearing_seat_od - CFG.bearings.ecc_bore <= 0.2

    def test_bearing_od_fits_disc_bore(self):
        clearance = CFG.disc.center_bore_dia - CFG.bearings.ecc_od
        assert 0 < clearance < 1.0

    def test_bearing_width_matches_disc_thickness(self):
        assert CFG.bearings.ecc_width == CFG.disc.thickness

    def test_lobe_eccentricity_consistent(self):
        assert CFG.shaft.eccentricity == CFG.gear.eccentricity

    def test_lobe_press_fit_within_tolerance(self):
        interference_r = CFG.shaft.bearing_seat_od / 2.0 - CFG.bearings.ecc_bore / 2.0
        assert interference_r <= 0.1


class TestLobeAxialAlignment:

    def test_lobe1_z_matches_disc1(self):
        assert stack_positions(CFG)["z_disc1"] == CFG.stack_up.z_disc1

    def test_lobe2_z_matches_disc2(self):
        assert stack_positions(CFG)["z_disc2"] == CFG.stack_up.z_disc2

    def test_inter_disc_gap_free_of_lobes(self):
        gap = CFG.stack_up.z_disc2 - (CFG.stack_up.z_disc1 + CFG.disc.thickness)
        assert abs(gap - CFG.disc.inter_disc_spacer) < 0.01


class TestEccentricOrbit:

    NUM_ANGLES = 360

    def test_disc_bearing_swept_envelope_fits_housing(self):
        g = CFG.gear
        _, max_r = compute_profile_radii(compute_epitrochoid(R=g.ring_pin_circle_radius, r=g.ring_pin_radius, N=g.num_ring_pins, e=g.eccentricity, num_points=CFG.profile.num_points))
        assert max_r + g.eccentricity < CFG.housing.bore_dia / 2.0

    def test_lobe_center_orbit_radius_equals_eccentricity(self):
        e = CFG.shaft.eccentricity
        for step in range(self.NUM_ANGLES):
            phi = 2 * math.pi * step / self.NUM_ANGLES
            assert abs(math.hypot(e * math.cos(phi), e * math.sin(phi)) - e) < 1e-10

    def test_lobe_fills_bearing_bore(self):
        oversize = CFG.shaft.bearing_seat_od - CFG.bearings.ecc_bore
        assert 0 <= oversize <= 0.2


@pytest.fixture(scope="module")
def stack():
    return stack_positions(CFG)


@pytest.fixture(scope="module")
def shaft():
    return cycloidal_eccentric_shaft.build()


@pytest.fixture(scope="module")
def bearing1(stack):
    return Pos(stack["x_disc1"], 0, stack["z_disc1"]) * bearing_6003._envelope()


@pytest.fixture(scope="module")
def bearing2(stack):
    return Pos(stack["x_disc2"], 0, stack["z_disc2"]) * bearing_6003._envelope()


@pytest.fixture(scope="module")
def disc1(stack):
    return Pos(stack["x_disc1"], 0, stack["z_disc1"]) * cycloidal_disc_1.build()


@pytest.mark.slow
class TestLobeBearingInterference:
    """Boolean intersections between the shaft lobes and the bearing annuli (the lobe OD is
    0.10 mm over the bore: a small designed interference), and none with the disc."""

    def test_lobe1_interference_within_press_fit(self, shaft, bearing1):
        vol = interference(shaft, bearing1)
        assert vol < 50.0, f"Shaft/bearing-1 interference = {vol:.1f}mm^3 (too large for a press fit)"

    def test_lobe2_interference_within_press_fit(self, shaft, bearing2):
        vol = interference(shaft, bearing2)
        assert vol < 50.0, f"Shaft/bearing-2 interference = {vol:.1f}mm^3 (too large for a press fit)"

    def test_shaft_passes_through_both_bearing_bores(self, shaft):
        bb = shaft.bounding_box()
        assert bb.min.Z <= CFG.stack_up.z_disc1
        assert bb.max.Z >= CFG.stack_up.z_disc2 + CFG.bearings.ecc_width

    def test_full_stack_no_shaft_disc_interference(self, shaft, disc1):
        vol = interference(shaft, disc1)
        assert vol < 1.0, f"Shaft/disc interference = {vol:.1f}mm^3 (should be ~0)"

    def test_full_stack_no_bearing_disc_interference(self, disc1, bearing1):
        vol = interference(disc1, bearing1)
        assert vol < 1.0, f"Bearing/disc interference = {vol:.1f}mm^3 (should be ~0)"
