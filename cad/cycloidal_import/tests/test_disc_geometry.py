"""Validation test suite for the cycloidal disc geometry.

Tests cover:
  1. Profile geometry — lobe count, radii, closure, smoothness, self-intersection
  2. Clearance checks — bore-to-pin wall, pin-to-lobe wall, disc-fits-housing
  3. Meshing simulation — ring pin interference and contact at sampled input angles
  4. CadQuery solid — valid topology, bounding box, volume sanity
"""

import math
import sys
import os

import numpy as np
import pytest

# Allow imports from project root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DEFAULT_CONFIG
from src.profiles import compute_epitrochoid, compute_profile_radii

# ---------------------------------------------------------------------------
# Shared fixtures
# ---------------------------------------------------------------------------

CFG = DEFAULT_CONFIG


@pytest.fixture(scope="module")
def profile_points():
    """Generate the epitrochoid profile once for all tests."""
    return compute_epitrochoid(
        R=CFG.gear.ring_pin_circle_radius,
        r=CFG.gear.ring_pin_radius,
        N=CFG.gear.num_ring_pins,
        e=CFG.gear.eccentricity,
        num_points=CFG.profile.num_points,
    )


@pytest.fixture(scope="module")
def profile_array(profile_points):
    """Profile as an (N, 2) numpy array."""
    return np.array(profile_points)


@pytest.fixture(scope="module")
def profile_radii(profile_array):
    """Radial distance from origin for each profile point."""
    return np.sqrt(profile_array[:, 0] ** 2 + profile_array[:, 1] ** 2)


# ===================================================================
# 1. Profile geometry
# ===================================================================


class TestProfileGeometry:

    def test_lobe_count(self, profile_radii):
        """The number of radial-distance local maxima must equal num_lobes (20)."""
        # Detect peaks: point[i] > point[i-1] AND point[i] > point[i+1]
        # Use circular comparison (wrap around)
        r = profile_radii
        n = len(r)
        peaks = 0
        for i in range(n):
            if r[i] > r[(i - 1) % n] and r[i] > r[(i + 1) % n]:
                peaks += 1
        assert peaks == CFG.gear.num_lobes, (
            f"Expected {CFG.gear.num_lobes} lobes, found {peaks} radial peaks"
        )

    def test_profile_radii_range(self, profile_points):
        """Min/max radii should be within expected bounds.

        Approximate bounds:
          max_radius ≈ R - r + e  (lobe tip approaches ring pin circle minus pin radius plus ecc)
          min_radius ≈ R - r - e  (lobe valley)
        With R=54, r=2, e=1.5:  max ≈ 53.5, min ≈ 50.5
        Allow ±2mm tolerance since these are approximations.
        """
        R = CFG.gear.ring_pin_circle_radius  # 54
        r = CFG.gear.ring_pin_radius  # 2
        e = CFG.gear.eccentricity  # 1.5

        min_r, max_r = compute_profile_radii(profile_points)

        expected_max = R - r + e  # 53.5
        expected_min = R - r - e  # 50.5

        assert abs(max_r - expected_max) < 2.0, (
            f"Max radius {max_r:.2f} too far from expected {expected_max:.2f}"
        )
        assert abs(min_r - expected_min) < 2.0, (
            f"Min radius {min_r:.2f} too far from expected {expected_min:.2f}"
        )

    def test_profile_closure(self, profile_array):
        """Profile should be periodic — first point close to the extrapolated wrap point."""
        # Since endpoint=False in linspace, check that the first and last points
        # form a smooth transition (distance ≈ one step's arc length)
        step_distances = np.linalg.norm(np.diff(profile_array, axis=0), axis=1)
        avg_step = np.mean(step_distances)

        wrap_distance = np.linalg.norm(profile_array[0] - profile_array[-1])
        # Wrap distance should be similar to a normal step (within 3x)
        assert wrap_distance < 3.0 * avg_step, (
            f"Profile not closed: wrap gap {wrap_distance:.4f}mm vs avg step {avg_step:.4f}mm"
        )

    def test_profile_smoothness(self, profile_array):
        """No sharp angle changes between consecutive segments (no cusps/kinks).

        The angle between consecutive tangent vectors should never exceed a
        reasonable threshold. For 2000 points on a 20-lobe profile, the
        expected angle change per step is ~360°*20/2000 = 3.6° for the
        fastest-varying component. Allow up to 15° as a generous bound.
        """
        pts = profile_array
        n = len(pts)

        # Tangent vectors (with circular wrap)
        tangents = np.empty_like(pts)
        tangents[:-1] = pts[1:] - pts[:-1]
        tangents[-1] = pts[0] - pts[-1]

        # Angle between consecutive tangents
        max_angle_deg = 0.0
        for i in range(n):
            t1 = tangents[(i - 1) % n]
            t2 = tangents[i]
            cos_a = np.dot(t1, t2) / (np.linalg.norm(t1) * np.linalg.norm(t2) + 1e-15)
            cos_a = np.clip(cos_a, -1.0, 1.0)
            angle = math.degrees(math.acos(cos_a))
            if angle > max_angle_deg:
                max_angle_deg = angle

        assert max_angle_deg < 15.0, (
            f"Profile has sharp kink: max angle change = {max_angle_deg:.2f}°"
        )

    def test_no_self_intersection(self, profile_array):
        """The profile should not cross itself.

        Uses a winding-number approach: the signed area of the polygon should
        be non-zero and the polygon should have consistent winding (all cross
        products of consecutive edges same sign for a convex-ish curve, but
        cycloidal profiles are not convex). Instead, we do a sampled segment
        intersection check on every 10th segment pair to keep runtime reasonable.
        """
        pts = profile_array
        n = len(pts)
        step = 10  # check every 10th segment pair for speed

        def segments_intersect(p1, p2, p3, p4):
            """Test if segment p1-p2 intersects segment p3-p4."""
            d1 = p2 - p1
            d2 = p4 - p3
            cross_d = d1[0] * d2[1] - d1[1] * d2[0]
            if abs(cross_d) < 1e-12:
                return False  # parallel
            d3 = p3 - p1
            t = (d3[0] * d2[1] - d3[1] * d2[0]) / cross_d
            u = (d3[0] * d1[1] - d3[1] * d1[0]) / cross_d
            return 0.0 < t < 1.0 and 0.0 < u < 1.0

        for i in range(0, n, step):
            p1 = pts[i]
            p2 = pts[(i + 1) % n]
            # Only check against non-adjacent segments
            for j in range(i + 2 * step, n - step, step):
                # Skip adjacent segments at wrap-around
                if i == 0 and j >= n - 2 * step:
                    continue
                p3 = pts[j]
                p4 = pts[(j + 1) % n]
                assert not segments_intersect(p1, p2, p3, p4), (
                    f"Self-intersection detected between segments {i} and {j}"
                )


# ===================================================================
# 2. Clearance checks
# ===================================================================


class TestClearances:

    def test_center_bore_to_pin_hole_wall(self):
        """Wall between center bore and output pin holes must be >= 5mm.

        Inner edge of pin hole = pin_circle_r - pin_hole_r
        Outer edge of bore = center_bore_dia / 2
        Wall = inner_edge - outer_edge
        Spec says 8.4mm.
        """
        pin_circle_r = CFG.disc.output_pin_circle_dia / 2.0  # 30
        pin_hole_r = CFG.disc.output_pin_hole_dia / 2.0  # 4
        bore_r = CFG.disc.center_bore_dia / 2.0  # 17.6

        wall = (pin_circle_r - pin_hole_r) - bore_r
        assert wall >= 5.0, (
            f"Bore-to-pin-hole wall = {wall:.2f}mm, need >= 5mm"
        )

    def test_pin_hole_to_lobe_valley_wall(self, profile_points):
        """Wall between output pin hole outer edge and disc profile inner radius >= 10mm.

        Outer edge of pin hole = pin_circle_r + pin_hole_r
        Disc min radius = profile min radius
        Wall = min_radius - outer_edge
        Spec says ~15mm.
        """
        pin_circle_r = CFG.disc.output_pin_circle_dia / 2.0  # 30
        pin_hole_r = CFG.disc.output_pin_hole_dia / 2.0  # 4
        min_r, _ = compute_profile_radii(profile_points)

        wall = min_r - (pin_circle_r + pin_hole_r)
        assert wall >= 10.0, (
            f"Pin-hole-to-lobe-valley wall = {wall:.2f}mm, need >= 10mm"
        )

    def test_disc_fits_in_housing(self, profile_points):
        """Max disc radius + eccentricity must not exceed housing bore radius.

        During operation the disc center wobbles by ±e, so the swept
        envelope radius = max_profile_radius + e.
        """
        _, max_r = compute_profile_radii(profile_points)
        e = CFG.gear.eccentricity
        housing_bore_r = CFG.housing.bore_dia / 2.0  # 58

        swept_r = max_r + e
        assert swept_r < housing_bore_r, (
            f"Disc swept radius {swept_r:.2f}mm exceeds housing bore {housing_bore_r:.2f}mm"
        )


# ===================================================================
# 3. Meshing / engagement simulation
# ===================================================================


def _transform_profile_to_housing(profile_array, input_angle_rad):
    """Transform disc profile points to housing frame for a given input angle.

    Kinematics:
      - Disc center orbits at (e·cos(φ), e·sin(φ))
      - Disc body rotates by -φ / N_lobes relative to housing
    """
    e = CFG.gear.eccentricity
    n_lobes = CFG.gear.num_lobes
    phi = input_angle_rad

    # Disc rotation angle
    disc_rot = -phi / n_lobes

    # Rotation matrix
    c, s = math.cos(disc_rot), math.sin(disc_rot)
    rot = np.array([[c, -s], [s, c]])

    # Rotate then translate
    rotated = profile_array @ rot.T
    offset = np.array([e * math.cos(phi), e * math.sin(phi)])
    return rotated + offset


def _ring_pin_centers():
    """Return (N, 2) array of ring pin center positions in housing frame."""
    R = CFG.gear.ring_pin_circle_radius
    N = CFG.gear.num_ring_pins
    angles = np.linspace(0, 2 * np.pi, N, endpoint=False)
    return np.column_stack([R * np.cos(angles), R * np.sin(angles)])


def _min_distances_to_profile(pin_center, profile_pts):
    """Minimum distance from a single pin center to the closest profile segment.

    Computes point-to-segment distance for all segments and returns the minimum.
    """
    # Vector approach for all segments at once
    p = pin_center
    a = profile_pts
    b = np.roll(profile_pts, -1, axis=0)

    ab = b - a
    ap = p - a
    # Parameter t clamped to [0, 1]
    t = np.sum(ap * ab, axis=1) / (np.sum(ab * ab, axis=1) + 1e-15)
    t = np.clip(t, 0.0, 1.0)
    # Closest point on each segment
    closest = a + t[:, np.newaxis] * ab
    dists = np.linalg.norm(p - closest, axis=1)
    return dists.min()


class TestMeshing:

    NUM_ANGLES = 360  # sample one full input revolution

    def test_ring_pin_no_interference(self, profile_array):
        """No ring pin should penetrate the disc profile at any input angle.

        For each sampled input angle, the minimum distance from every ring pin
        center to the disc profile must be >= pin_radius (minus small tolerance
        for numerical error).
        """
        pin_r = CFG.gear.ring_pin_radius  # 2.0
        pin_centers = _ring_pin_centers()
        tolerance = 0.15  # mm, numerical tolerance

        for step in range(self.NUM_ANGLES):
            phi = 2 * math.pi * step / self.NUM_ANGLES
            transformed = _transform_profile_to_housing(profile_array, phi)

            for k, pc in enumerate(pin_centers):
                d = _min_distances_to_profile(pc, transformed)
                assert d >= pin_r - tolerance, (
                    f"Ring pin {k} penetrates disc at input angle "
                    f"{math.degrees(phi):.1f}°: distance={d:.3f}mm < pin_r={pin_r}mm"
                )

    def test_ring_pin_contact_exists(self, profile_array):
        """At each input angle, at least one ring pin should be near-contact.

        Near-contact means distance from pin center to profile ≈ pin_radius
        (within a contact tolerance band).
        """
        pin_r = CFG.gear.ring_pin_radius
        pin_centers = _ring_pin_centers()
        contact_tolerance = 0.5  # mm — how close counts as "in contact"

        for step in range(self.NUM_ANGLES):
            phi = 2 * math.pi * step / self.NUM_ANGLES
            transformed = _transform_profile_to_housing(profile_array, phi)

            min_gap = float("inf")
            for pc in pin_centers:
                d = _min_distances_to_profile(pc, transformed)
                gap = abs(d - pin_r)
                if gap < min_gap:
                    min_gap = gap

            assert min_gap < contact_tolerance, (
                f"No ring pin in contact at input angle {math.degrees(phi):.1f}°: "
                f"closest gap = {min_gap:.3f}mm"
            )


# ===================================================================
# 4. CadQuery solid validation
# ===================================================================


class TestCadQuerySolid:

    @pytest.fixture(scope="class")
    def disc_solid(self):
        cq = pytest.importorskip("cadquery")
        from src.cycloidal_disc import build_cycloidal_disc

        return build_cycloidal_disc()

    def test_solid_is_valid(self, disc_solid):
        """The built solid should be non-null and have exactly one solid."""
        solids = disc_solid.solids().vals()
        assert len(solids) == 1, f"Expected 1 solid, got {len(solids)}"

    def test_bounding_box_dimensions(self, disc_solid):
        """Bounding box should match expected disc envelope.

        XY extent ≈ disc OD (~108mm), Z extent = disc thickness (10mm).
        """
        bb = disc_solid.val().BoundingBox()
        x_size = bb.xmax - bb.xmin
        y_size = bb.ymax - bb.ymin
        z_size = bb.zmax - bb.zmin

        expected_od = 108.0  # approximate disc OD
        assert abs(x_size - expected_od) < 5.0, (
            f"X extent {x_size:.2f}mm, expected ~{expected_od}mm"
        )
        assert abs(y_size - expected_od) < 5.0, (
            f"Y extent {y_size:.2f}mm, expected ~{expected_od}mm"
        )
        assert abs(z_size - CFG.disc.thickness) < 0.1, (
            f"Z extent {z_size:.2f}mm, expected {CFG.disc.thickness}mm"
        )

    def test_lobe_chamfer_applied(self, disc_solid):
        """Bottom face max radial extent should be `chamfer` smaller than the
        disc's overall extent. At 45°, going down by `chamfer` in Z reduces
        the radius by `chamfer`. Catches silent CadQuery chamfer failures.
        """
        chamfer = CFG.disc.lobe_chamfer
        if chamfer <= 0:
            pytest.skip("Chamfer disabled")

        disc_bb = disc_solid.val().BoundingBox()
        disc_max = max(abs(disc_bb.xmin), abs(disc_bb.xmax),
                       abs(disc_bb.ymin), abs(disc_bb.ymax))

        bottom_bb = disc_solid.faces("<Z").val().BoundingBox()
        bottom_max = max(abs(bottom_bb.xmin), abs(bottom_bb.xmax),
                         abs(bottom_bb.ymin), abs(bottom_bb.ymax))

        delta = disc_max - bottom_max
        assert abs(delta - chamfer) < 0.1, (
            f"Bottom-face radial reduction = {delta:.3f}mm, "
            f"expected chamfer = {chamfer}mm"
        )

    def test_lobe_chamfer_symmetric(self, disc_solid):
        """Top and bottom faces should have equal max radial extent so the
        disc has no 'wrong side up' for printing.
        """
        if CFG.disc.lobe_chamfer <= 0:
            pytest.skip("Chamfer disabled")

        top_bb = disc_solid.faces(">Z").val().BoundingBox()
        bot_bb = disc_solid.faces("<Z").val().BoundingBox()
        top_max = max(abs(top_bb.xmin), abs(top_bb.xmax),
                      abs(top_bb.ymin), abs(top_bb.ymax))
        bot_max = max(abs(bot_bb.xmin), abs(bot_bb.xmax),
                      abs(bot_bb.ymin), abs(bot_bb.ymax))
        assert abs(top_max - bot_max) < 0.05, (
            f"Top extent {top_max:.3f}mm != bottom extent {bot_max:.3f}mm "
            f"(chamfer not symmetric)"
        )

    def test_volume_sanity(self, disc_solid):
        """Volume should be between reasonable bounds.

        Lower bound: a thin annulus (bore to min_radius) × thickness
        Upper bound: full circle at max_radius × thickness
        Both minus the 4 output pin holes.
        """
        import cadquery as cq

        vol = disc_solid.val().Volume()  # mm³

        # Rough bounds
        min_r = 49.0  # approximate lobe valley radius
        max_r = 55.0  # approximate lobe tip radius
        bore_r = CFG.disc.center_bore_dia / 2.0
        t = CFG.disc.thickness
        pin_hole_r = CFG.disc.output_pin_hole_dia / 2.0
        n_pins = CFG.disc.output_pin_count

        lower = math.pi * (min_r**2 - bore_r**2) * t * 0.7  # 70% fill factor for lobes
        lower -= n_pins * math.pi * pin_hole_r**2 * t
        upper = math.pi * max_r**2 * t

        assert lower < vol < upper, (
            f"Volume {vol:.0f}mm³ outside expected range [{lower:.0f}, {upper:.0f}]"
        )


# ===================================================================
# 5. Part fitment — disc ↔ purchased parts
# ===================================================================


class TestDiscFitment:
    """Verify that purchased parts mate correctly with the cycloidal disc."""

    # -- Parametric checks (fast) ------------------------------------

    def test_6003_bearing_od_fits_disc_bore(self):
        """6003 bearing OD must be smaller than disc center bore (clearance fit)."""
        bearing_od = CFG.bearings.ecc_od  # 35.0mm
        bore_dia = CFG.disc.center_bore_dia  # 35.2mm
        clearance = bore_dia - bearing_od
        assert clearance > 0, (
            f"6003 bearing OD ({bearing_od}mm) exceeds disc bore ({bore_dia}mm)"
        )
        assert clearance < 1.0, (
            f"Excessive clearance {clearance:.2f}mm between bearing and bore"
        )

    def test_6003_bearing_width_matches_disc_thickness(self):
        """6003 bearing width must equal disc thickness (both 10mm)."""
        assert CFG.bearings.ecc_width == CFG.disc.thickness, (
            f"Bearing width {CFG.bearings.ecc_width}mm != "
            f"disc thickness {CFG.disc.thickness}mm"
        )

    def test_output_pin_fits_disc_hole(self):
        """Output pin diameter must be smaller than disc pin hole."""
        pin_dia = CFG.disc.output_pin_dia  # 4.0mm
        hole_dia = CFG.disc.output_pin_hole_dia  # 8.0mm
        clearance = hole_dia - pin_dia
        assert clearance > 0, (
            f"Output pin ({pin_dia}mm) won't fit disc hole ({hole_dia}mm)"
        )
        # Clearance must accommodate eccentricity: need >= 2*e
        min_clearance = 2 * CFG.gear.eccentricity  # 3.0mm
        assert clearance >= min_clearance, (
            f"Pin hole clearance {clearance:.2f}mm < 2*eccentricity {min_clearance}mm"
        )

    def test_output_hole_backlash_budget(self):
        """Output-pin-hole radial slack sets the drive's designed backlash.

        Slack beyond the kinematic orbit requirement
        (slack = hole_r - pin_r - e) is the lost motion at the output pins.
        At the 30mm pin circle, 0.2mm slack is about +/-0.38 deg of output
        backlash. Keep it in a band: too tight (<0.15mm) over-constrains the
        4 rigid steel pins against printed-hole position error; too loose
        (>0.30mm) re-grows backlash. The ring-pin mesh is a zero-clearance
        theoretical epitrochoid, so this hole is the dominant designed source.
        7.4mm is the practical floor; below ~7.2mm the pins bind after shrink.
        """
        d = CFG.disc
        e = CFG.gear.eccentricity
        slack = d.output_pin_hole_dia / 2.0 - d.output_pin_dia / 2.0 - e
        assert 0.15 <= slack <= 0.30, (
            f"Output-hole radial slack {slack:.3f}mm outside 0.15-0.30mm "
            f"backlash budget (hole {d.output_pin_hole_dia}mm)"
        )

    def test_eccentric_shaft_seat_fits_6003_bore(self):
        """Shaft bearing seat OD must be >= 6003 bearing bore (light press)."""
        assert CFG.shaft.bearing_seat_od >= CFG.bearings.ecc_bore, (
            f"Shaft seat OD {CFG.shaft.bearing_seat_od}mm < "
            f"6003 bore {CFG.bearings.ecc_bore}mm"
        )
        assert CFG.shaft.bearing_seat_od - CFG.bearings.ecc_bore <= 0.2, (
            f"Shaft seat OD {CFG.shaft.bearing_seat_od}mm exceeds "
            f"6003 bore {CFG.bearings.ecc_bore}mm by more than 0.2mm"
        )

    # -- CadQuery boolean interference check (thorough) --------------

    @pytest.fixture(scope="class")
    def disc_and_bearing(self):
        cq = pytest.importorskip("cadquery")
        from src.cycloidal_disc import build_cycloidal_disc
        from src.purchased_parts import build_bearing_6003

        return build_cycloidal_disc(), build_bearing_6003()

    def test_6003_bearing_no_interference_with_disc(self, disc_and_bearing):
        """Boolean intersection of bearing and disc solids must be empty.

        Both parts are built at origin — the bearing sits inside the
        disc bore, so their solid volumes must not overlap.
        """
        disc, bearing = disc_and_bearing
        interference = disc.intersect(bearing)
        vol = interference.val().Volume()
        assert vol < 1.0, (
            f"Bearing/disc interference volume = {vol:.1f}mm³ (should be ~0)"
        )


# ===================================================================
# 6. Assembly-level meshing — both discs at their orbit positions
# ===================================================================


class TestAssemblyMeshing:
    """Both discs as built (with profile rotation) must clear all ring pins
    at their static assembly orbit positions. Point-cloud based: pure numpy
    on the rotated epitrochoid, no CadQuery booleans needed.
    """

    def _disc_points_in_housing(self, phase_offset_deg, orbit_xy):
        pts = compute_epitrochoid(
            R=CFG.gear.ring_pin_circle_radius,
            r=CFG.gear.ring_pin_radius,
            N=CFG.gear.num_ring_pins,
            e=CFG.gear.eccentricity,
            num_points=CFG.profile.num_points,
        )
        if phase_offset_deg != 0.0:
            a = math.radians(phase_offset_deg)
            c, s = math.cos(a), math.sin(a)
            pts = [(c * x - s * y, s * x + c * y) for x, y in pts]
        return np.array(pts) + np.array(orbit_xy)

    def test_disc2_phase_value(self):
        assert math.isclose(CFG.gear.disc2_phase_deg, -9.0)

    def test_disc1_no_ring_pin_overlap(self):
        e = CFG.gear.eccentricity
        pin_r = CFG.gear.ring_pin_radius
        pts = self._disc_points_in_housing(0.0, (e, 0))
        for pc in _ring_pin_centers():
            d = _min_distances_to_profile(pc, pts)
            assert d >= pin_r - 0.15

    def test_disc2_no_ring_pin_overlap(self):
        """Regression test: catches the original bug where disc 2's lobes
        penetrate ring pins because the 180° assembly rotation is a no-op
        for an N_lobes=20 disc (180° = 10·18° = 10 lobe pitches).
        """
        e = CFG.gear.eccentricity
        pin_r = CFG.gear.ring_pin_radius
        pts = self._disc_points_in_housing(
            CFG.gear.disc2_phase_deg, (-e, 0)
        )
        for pc in _ring_pin_centers():
            d = _min_distances_to_profile(pc, pts)
            assert d >= pin_r - 0.15

    def test_disc2_output_holes_clear_pins(self):
        """Disc 2's output holes (at disc-local 0/90/180/270) must
        accommodate the stationary output pins (at housing 0/90/180/270)
        when disc 2 is translated to (-e, 0).
        """
        d = CFG.disc
        e = CFG.gear.eccentricity
        pin_r = d.output_pin_dia / 2.0
        hole_r = d.output_pin_hole_dia / 2.0
        pin_circle_r = d.output_pin_circle_dia / 2.0
        for k in range(d.output_pin_count):
            a = math.radians(k * 360.0 / d.output_pin_count)
            pin_xy = (pin_circle_r * math.cos(a),
                      pin_circle_r * math.sin(a))
            hole_xy = (pin_circle_r * math.cos(a) - e,
                       pin_circle_r * math.sin(a))
            center_dist = math.hypot(pin_xy[0] - hole_xy[0],
                                     pin_xy[1] - hole_xy[1])
            margin = hole_r - pin_r - center_dist
            assert margin >= 0.2 - 1e-6, (
                f"Pin {k}/disc-2 hole margin {margin:.2f}mm < 0.2mm"
            )


class TestAssemblyInterference:
    """CadQuery-level boolean test: as-built discs + ring pins must not
    meaningfully overlap. Complements the point-cloud tests in
    TestAssemblyMeshing by validating the actual extruded geometry
    (including chamfers).
    """

    @pytest.fixture(scope="class")
    def disc1_built(self):
        pytest.importorskip("cadquery")
        from src.cycloidal_disc import build_cycloidal_disc

        e = CFG.gear.eccentricity
        z = CFG.stack_up.z_disc1
        return build_cycloidal_disc().translate((e, 0, z))

    @pytest.fixture(scope="class")
    def disc2_built(self):
        pytest.importorskip("cadquery")
        from src.cycloidal_disc import build_cycloidal_disc

        e = CFG.gear.eccentricity
        z = CFG.stack_up.z_disc2
        return build_cycloidal_disc(
            phase_offset_deg=CFG.gear.disc2_phase_deg
        ).translate((-e, 0, z))

    @pytest.fixture(scope="class")
    def ring_pins(self):
        pytest.importorskip("cadquery")
        from src.purchased_parts import build_ring_pins

        s = CFG.stack_up
        bore_zone = s.disc_zone + s.output_clearance
        pin_engagement = (CFG.gear.ring_pin_length - bore_zone) / 2.0
        z_pins = s.z_motor_plate_inner - pin_engagement
        return build_ring_pins().translate((0, 0, z_pins))

    def test_disc1_no_pin_interference(self, disc1_built, ring_pins):
        vol = disc1_built.intersect(ring_pins).val().Volume()
        assert vol < 1.0, f"Disc 1 / ring-pin overlap = {vol:.2f}mm³"

    def test_disc2_no_pin_interference(self, disc2_built, ring_pins):
        """Regression: buggy code produced multi-mm³ overlap here."""
        vol = disc2_built.intersect(ring_pins).val().Volume()
        assert vol < 1.0, f"Disc 2 / ring-pin overlap = {vol:.2f}mm³"

    def test_discs_are_distinct_parts(self, disc1_built, disc2_built):
        """Disc 1 and disc 2 must not be identical (the bug was treating
        them as the same part). After undoing their orbit translations,
        their epitrochoid profiles must be visibly rotated relative to
        each other (~half a lobe pitch), so a boolean cut leaves a
        non-trivial residual.
        """
        e = CFG.gear.eccentricity
        d1 = disc1_built.translate((-e, 0, 0))
        d2 = disc2_built.translate((e, 0, 0))
        diff = d1.cut(d2).val().Volume()
        assert diff > 10.0, (
            f"Disc 1 and disc 2 appear identical (cut volume {diff:.2f}mm³); "
            "phase_offset_deg may not have been applied"
        )
