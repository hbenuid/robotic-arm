"""Cycloidal disc geometry (ported from cycloidal_drive tests/test_disc_geometry.py).

  1. Profile geometry - lobe count, radii, closure, smoothness, self-intersection
  2. Clearances - bore-to-pin wall, pin-to-lobe wall, disc-fits-housing
  3. Meshing simulation - ring-pin interference / contact over one input revolution
  4. The built solid - topology, bounding box, chamfer, volume sanity
  5. Disc <-> purchased-part fitment; 6. both discs at their orbit positions
"""
import math

import numpy as np
import pytest
from build123d import Pos

import parts
from lib import reference as R
from lib.cycloidal.profiles import compute_epitrochoid, compute_profile_radii, profile_points
from tests.cycloidal.helpers import CFG, annulus, end_face, interference, is_inside, radial_extent, ring_pins

cycloidal_disc_1 = parts.load("cycloidal_disc_1")
cycloidal_disc_2 = parts.load("cycloidal_disc_2")


@pytest.fixture(scope="module")
def profile_pts():
    return compute_epitrochoid(
        R=CFG.gear.ring_pin_circle_radius, r=CFG.gear.ring_pin_radius, N=CFG.gear.num_ring_pins,
        e=CFG.gear.eccentricity, num_points=CFG.profile.num_points,
    )


@pytest.fixture(scope="module")
def profile_array(profile_pts):
    return np.array(profile_pts)


@pytest.fixture(scope="module")
def profile_radii(profile_array):
    return np.sqrt(profile_array[:, 0] ** 2 + profile_array[:, 1] ** 2)


# ===================================================================
# 1. Profile geometry
# ===================================================================


class TestProfileGeometry:

    def test_lobe_count(self, profile_radii):
        """The number of radial-distance local maxima must equal num_lobes (20)."""
        r = profile_radii
        n = len(r)
        peaks = sum(1 for i in range(n) if r[i] > r[(i - 1) % n] and r[i] > r[(i + 1) % n])
        assert peaks == CFG.gear.num_lobes, f"Expected {CFG.gear.num_lobes} lobes, found {peaks} radial peaks"

    def test_profile_radii_range(self, profile_pts):
        """max ~ R - r + e (53.5), min ~ R - r - e (50.5), +/-2 mm (approximations)."""
        Rr, r, e = CFG.gear.ring_pin_circle_radius, CFG.gear.ring_pin_radius, CFG.gear.eccentricity
        min_r, max_r = compute_profile_radii(profile_pts)
        assert abs(max_r - (Rr - r + e)) < 2.0, f"Max radius {max_r:.2f} too far from {Rr - r + e:.2f}"
        assert abs(min_r - (Rr - r - e)) < 2.0, f"Min radius {min_r:.2f} too far from {Rr - r - e:.2f}"

    def test_profile_closure(self, profile_array):
        """endpoint=False: the wrap gap must look like one more step."""
        step_distances = np.linalg.norm(np.diff(profile_array, axis=0), axis=1)
        avg_step = np.mean(step_distances)
        wrap_distance = np.linalg.norm(profile_array[0] - profile_array[-1])
        assert wrap_distance < 3.0 * avg_step, f"Profile not closed: wrap gap {wrap_distance:.4f} vs avg step {avg_step:.4f}"

    def test_profile_smoothness(self, profile_array):
        """No cusps: consecutive tangents never turn more than 15 deg (~3.6 deg expected)."""
        pts = profile_array
        n = len(pts)
        tangents = np.empty_like(pts)
        tangents[:-1] = pts[1:] - pts[:-1]
        tangents[-1] = pts[0] - pts[-1]
        max_angle_deg = 0.0
        for i in range(n):
            t1, t2 = tangents[(i - 1) % n], tangents[i]
            cos_a = np.clip(np.dot(t1, t2) / (np.linalg.norm(t1) * np.linalg.norm(t2) + 1e-15), -1.0, 1.0)
            max_angle_deg = max(max_angle_deg, math.degrees(math.acos(cos_a)))
        assert max_angle_deg < 15.0, f"Profile has sharp kink: max angle change = {max_angle_deg:.2f} deg"

    def test_no_self_intersection(self, profile_array):
        """Sampled segment-pair intersection check (every 10th segment)."""
        pts = profile_array
        n = len(pts)
        step = 10

        def segments_intersect(p1, p2, p3, p4):
            d1, d2 = p2 - p1, p4 - p3
            cross_d = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(cross_d) < 1e-12:
                return False
            d3 = p3 - p1
            t = (d3[0] * d2[1] - d3[1] * d2[0]) / cross_d
            u = (d3[0] * d1[1] - d3[1] * d1[0]) / cross_d
            return 0.0 < t < 1.0 and 0.0 < u < 1.0

        for i in range(0, n, step):
            p1, p2 = pts[i], pts[(i + 1) % n]
            for j in range(i + 2 * step, n - step, step):
                if i == 0 and j >= n - 2 * step:
                    continue
                assert not segments_intersect(p1, p2, pts[j], pts[(j + 1) % n]), f"Self-intersection between segments {i} and {j}"


# ===================================================================
# 2. Clearance checks
# ===================================================================


class TestClearances:

    def test_center_bore_to_pin_hole_wall(self):
        """Wall between the centre bore and the output-pin holes >= 5 mm (spec: 8.4)."""
        pin_circle_r = CFG.disc.output_pin_circle_dia / 2.0
        pin_hole_r = CFG.disc.output_pin_hole_dia / 2.0
        bore_r = CFG.disc.center_bore_dia / 2.0
        wall = (pin_circle_r - pin_hole_r) - bore_r
        assert wall >= 5.0, f"Bore-to-pin-hole wall = {wall:.2f}mm, need >= 5mm"

    def test_pin_hole_to_lobe_valley_wall(self, profile_pts):
        """Wall between the pin-hole outer edge and the profile's inner radius >= 10 mm."""
        pin_circle_r = CFG.disc.output_pin_circle_dia / 2.0
        pin_hole_r = CFG.disc.output_pin_hole_dia / 2.0
        min_r, _ = compute_profile_radii(profile_pts)
        wall = min_r - (pin_circle_r + pin_hole_r)
        assert wall >= 10.0, f"Pin-hole-to-lobe-valley wall = {wall:.2f}mm, need >= 10mm"

    def test_disc_fits_in_housing(self, profile_pts):
        """The swept envelope (max radius + e) stays inside the housing bore."""
        _, max_r = compute_profile_radii(profile_pts)
        swept_r = max_r + CFG.gear.eccentricity
        assert swept_r < CFG.housing.bore_dia / 2.0, f"Disc swept radius {swept_r:.2f}mm exceeds housing bore"


# ===================================================================
# 3. Meshing / engagement simulation
# ===================================================================


def _transform_profile_to_housing(profile_array, input_angle_rad):
    """Disc centre orbits at (e cos phi, e sin phi); the disc rotates by -phi / N_lobes."""
    e, n_lobes, phi = CFG.gear.eccentricity, CFG.gear.num_lobes, input_angle_rad
    disc_rot = -phi / n_lobes
    c, s = math.cos(disc_rot), math.sin(disc_rot)
    rotated = profile_array @ np.array([[c, -s], [s, c]]).T
    return rotated + np.array([e * math.cos(phi), e * math.sin(phi)])


def _ring_pin_centers():
    Rr, N = CFG.gear.ring_pin_circle_radius, CFG.gear.num_ring_pins
    angles = np.linspace(0, 2 * np.pi, N, endpoint=False)
    return np.column_stack([Rr * np.cos(angles), Rr * np.sin(angles)])


def _min_distances_to_profile(pin_center, profile_pts):
    """Minimum point-to-segment distance from a pin centre to the profile polyline."""
    p, a = pin_center, profile_pts
    b = np.roll(profile_pts, -1, axis=0)
    ab, ap = b - a, p - a
    t = np.clip(np.sum(ap * ab, axis=1) / (np.sum(ab * ab, axis=1) + 1e-15), 0.0, 1.0)
    closest = a + t[:, np.newaxis] * ab
    return np.linalg.norm(p - closest, axis=1).min()


class TestMeshing:

    NUM_ANGLES = 360

    def test_ring_pin_no_interference(self, profile_array):
        """No ring pin penetrates the profile at any input angle (0.15 mm numerical tolerance)."""
        pin_r = CFG.gear.ring_pin_radius
        pin_centers = _ring_pin_centers()
        for step in range(self.NUM_ANGLES):
            phi = 2 * math.pi * step / self.NUM_ANGLES
            transformed = _transform_profile_to_housing(profile_array, phi)
            for k, pc in enumerate(pin_centers):
                d = _min_distances_to_profile(pc, transformed)
                assert d >= pin_r - 0.15, f"Ring pin {k} penetrates disc at {math.degrees(phi):.1f} deg: {d:.3f} < {pin_r}"

    def test_ring_pin_contact_exists(self, profile_array):
        """At every input angle at least one ring pin is in near-contact (< 0.5 mm)."""
        pin_r = CFG.gear.ring_pin_radius
        pin_centers = _ring_pin_centers()
        for step in range(self.NUM_ANGLES):
            phi = 2 * math.pi * step / self.NUM_ANGLES
            transformed = _transform_profile_to_housing(profile_array, phi)
            min_gap = min(abs(_min_distances_to_profile(pc, transformed) - pin_r) for pc in pin_centers)
            assert min_gap < 0.5, f"No ring pin in contact at {math.degrees(phi):.1f} deg: closest gap = {min_gap:.3f}mm"


# ===================================================================
# 4. The built solid
# ===================================================================


@pytest.fixture(scope="module")
def disc_solid():
    return cycloidal_disc_1.build()


@pytest.mark.slow
class TestSolid:

    def test_solid_is_valid(self, disc_solid):
        assert len(disc_solid.solids()) == 1
        assert disc_solid.is_valid

    def test_bounding_box_dimensions(self, disc_solid):
        """XY extent ~ disc OD (~108 mm), Z extent = thickness."""
        size = disc_solid.bounding_box().size
        assert abs(size.X - 108.0) < 5.0, f"X extent {size.X:.2f}mm, expected ~108mm"
        assert abs(size.Y - 108.0) < 5.0, f"Y extent {size.Y:.2f}mm, expected ~108mm"
        assert abs(size.Z - CFG.disc.thickness) < 0.1

    def test_lobe_chamfer_applied(self, disc_solid):
        """The bottom face's radial extent is `chamfer` smaller than the disc's (45 deg)."""
        chamfer = CFG.disc.lobe_chamfer
        if chamfer <= 0:
            pytest.skip("Chamfer disabled")
        delta = radial_extent(disc_solid) - radial_extent(end_face(disc_solid, "min"))
        assert abs(delta - chamfer) < 0.1, f"Bottom-face radial reduction = {delta:.3f}mm, expected {chamfer}mm"

    def test_lobe_chamfer_symmetric(self, disc_solid):
        """Top and bottom faces have equal extents - no 'wrong side up' for printing."""
        if CFG.disc.lobe_chamfer <= 0:
            pytest.skip("Chamfer disabled")
        top, bot = radial_extent(end_face(disc_solid, "max")), radial_extent(end_face(disc_solid, "min"))
        assert abs(top - bot) < 0.05, f"Top extent {top:.3f} != bottom extent {bot:.3f} (chamfer not symmetric)"

    def test_volume_sanity(self, disc_solid):
        """Between a 70 %-filled annulus (valley radius) minus the pin holes and a full tip-radius disc."""
        vol = R.solid_volume(disc_solid)
        bore_r, t = CFG.disc.center_bore_dia / 2.0, CFG.disc.thickness
        pin_hole_r, n_pins = CFG.disc.output_pin_hole_dia / 2.0, CFG.disc.output_pin_count
        lower = math.pi * (49.0 ** 2 - bore_r ** 2) * t * 0.7 - n_pins * math.pi * pin_hole_r ** 2 * t
        upper = math.pi * 55.0 ** 2 * t
        assert lower < vol < upper, f"Volume {vol:.0f}mm^3 outside [{lower:.0f}, {upper:.0f}]"


# ===================================================================
# 5. Part fitment - disc <-> purchased parts
# ===================================================================


class TestDiscFitment:

    def test_6003_bearing_od_fits_disc_bore(self):
        clearance = CFG.disc.center_bore_dia - CFG.bearings.ecc_od
        assert clearance > 0, "6003 OD exceeds the disc bore"
        assert clearance < 1.0, f"Excessive clearance {clearance:.2f}mm between bearing and bore"

    def test_6003_bearing_width_matches_disc_thickness(self):
        assert CFG.bearings.ecc_width == CFG.disc.thickness

    def test_output_pin_fits_disc_hole(self):
        clearance = CFG.disc.output_pin_hole_dia - CFG.disc.output_pin_dia
        assert clearance > 0
        assert clearance >= 2 * CFG.gear.eccentricity, f"Pin hole clearance {clearance:.2f}mm < 2*eccentricity"

    def test_output_hole_backlash_budget(self):
        """Radial slack beyond the orbit (hole_r - pin_r - e) is the designed backlash: 0.15..0.30 mm
        (~+/-0.38 deg at the 30 mm pin circle); 7.4 mm is the practical floor for 4 rigid pins."""
        d, e = CFG.disc, CFG.gear.eccentricity
        slack = d.output_pin_hole_dia / 2.0 - d.output_pin_dia / 2.0 - e
        assert 0.15 <= slack <= 0.30, f"Output-hole radial slack {slack:.3f}mm outside 0.15-0.30mm"

    def test_eccentric_shaft_seat_fits_6003_bore(self):
        assert CFG.shaft.bearing_seat_od >= CFG.bearings.ecc_bore
        assert CFG.shaft.bearing_seat_od - CFG.bearings.ecc_bore <= 0.2

    @pytest.mark.slow
    def test_6003_bearing_no_interference_with_disc(self):
        """Both at the origin: the bearing sits inside the disc bore, no overlap."""
        b = CFG.bearings
        vol = interference(cycloidal_disc_1.build(), annulus(b.ecc_od, b.ecc_bore, b.ecc_width))
        assert vol < 1.0, f"Bearing/disc interference volume = {vol:.1f}mm^3 (should be ~0)"


# ===================================================================
# 6. Assembly-level meshing - both discs at their orbit positions
# ===================================================================


class TestAssemblyMeshing:
    """Both discs as built (with the profile rotation) clear all ring pins at their static
    assembly orbits. Pure numpy on the rotated epitrochoid."""

    def _disc_points_in_housing(self, phase_offset_deg, orbit_xy):
        return np.array(profile_points(CFG, phase_offset_deg)) + np.array(orbit_xy)

    def test_disc2_phase_value(self):
        assert math.isclose(CFG.gear.disc2_phase_deg, -9.0)
        assert cycloidal_disc_1.phase_deg(CFG) == 0.0
        assert cycloidal_disc_2.phase_deg(CFG) == CFG.gear.disc2_phase_deg

    def test_disc1_no_ring_pin_overlap(self):
        e, pin_r = CFG.gear.eccentricity, CFG.gear.ring_pin_radius
        pts = self._disc_points_in_housing(0.0, (e, 0))
        for pc in _ring_pin_centers():
            assert _min_distances_to_profile(pc, pts) >= pin_r - 0.15

    def test_disc2_no_ring_pin_overlap(self):
        """Regression: a 180 deg assembly rotation is a no-op on a 20-lobe disc (10 lobe pitches);
        only the -9 deg profile phase keeps disc 2's lobes off the pins."""
        e, pin_r = CFG.gear.eccentricity, CFG.gear.ring_pin_radius
        pts = self._disc_points_in_housing(CFG.gear.disc2_phase_deg, (-e, 0))
        for pc in _ring_pin_centers():
            assert _min_distances_to_profile(pc, pts) >= pin_r - 0.15

    def test_disc2_output_holes_clear_pins(self):
        """Disc 2's holes at disc-local 0/90/180/270 accommodate the stationary output pins
        after the (-e, 0) translation."""
        d, e = CFG.disc, CFG.gear.eccentricity
        pin_r, hole_r, pin_circle_r = d.output_pin_dia / 2.0, d.output_pin_hole_dia / 2.0, d.output_pin_circle_dia / 2.0
        for k in range(d.output_pin_count):
            a = math.radians(k * 360.0 / d.output_pin_count)
            pin_xy = (pin_circle_r * math.cos(a), pin_circle_r * math.sin(a))
            hole_xy = (pin_circle_r * math.cos(a) - e, pin_circle_r * math.sin(a))
            margin = hole_r - pin_r - math.hypot(pin_xy[0] - hole_xy[0], pin_xy[1] - hole_xy[1])
            assert margin >= 0.2 - 1e-6, f"Pin {k}/disc-2 hole margin {margin:.2f}mm < 0.2mm"


@pytest.fixture(scope="module")
def disc1_built(stack):
    return Pos(stack["x_disc1"], 0, stack["z_disc1"]) * cycloidal_disc_1.build()


@pytest.fixture(scope="module")
def disc2_built(stack):
    return Pos(stack["x_disc2"], 0, stack["z_disc2"]) * cycloidal_disc_2.build()


@pytest.fixture(scope="module")
def pins(stack):
    return ring_pins(CFG, stack["z_ring_pins"])


@pytest.mark.slow
class TestAssemblyInterference:
    """Boolean checks on the as-built discs (chamfers included) + ring pins at their stack
    positions."""

    def test_disc1_no_pin_interference(self, disc1_built, pins):
        vol = interference(disc1_built, pins)
        assert vol < 1.0, f"Disc 1 / ring-pin overlap = {vol:.2f}mm^3"

    def test_disc2_no_pin_interference(self, disc2_built, pins):
        """Regression: the buggy identical-discs build produced multi-mm^3 overlap here."""
        vol = interference(disc2_built, pins)
        assert vol < 1.0, f"Disc 2 / ring-pin overlap = {vol:.2f}mm^3"

    def test_discs_are_distinct_parts(self):
        """Disc 1's lobe tip is a valley on disc 2 (half a lobe pitch apart) - point probes on
        the un-translated discs (a boolean cut between the two spline solids takes minutes)."""
        d1, d2 = cycloidal_disc_1.build(), cycloidal_disc_2.build()
        z = CFG.disc.thickness / 2.0
        tip = max(profile_points(CFG), key=lambda p: math.hypot(*p))
        r = math.hypot(*tip)
        x, y = tip[0] * (r - 0.6) / r, tip[1] * (r - 0.6) / r     # just inside disc 1's tip
        assert is_inside(d1, x, y, z), "disc 1's own lobe tip must be material"
        assert not is_inside(d2, x, y, z), "disc 2 must have a valley where disc 1 has a lobe tip (phase not applied?)"
        tip2 = max(profile_points(CFG, CFG.gear.disc2_phase_deg), key=lambda p: math.hypot(*p))
        r2 = math.hypot(*tip2)
        x2, y2 = tip2[0] * (r2 - 0.6) / r2, tip2[1] * (r2 - 0.6) / r2
        assert is_inside(d2, x2, y2, z) and not is_inside(d1, x2, y2, z)
