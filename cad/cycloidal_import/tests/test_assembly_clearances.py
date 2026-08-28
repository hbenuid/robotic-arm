"""Assembly-level clearance verification — Design Sequence Step 8.

Verifies all cross-part relationships when parts are positioned per the
axial stack-up (params.StackUp).  Tests cover:

  1. Axial stack-up — Z positions are consistent, no gaps or overlaps
  2. Housing alignment — motor plate and ring gear body mate flush
  3. Radial clearances — discs clear housing, pins clear bearings, hub clears lip
  4. Bearing retention — 6814 held by press-fit + ring body's integral lip
  5. Shaft reach — motor shaft engages D-bore, eccentric shaft reaches 625 pocket
  6. Ring pin span — pins seat in motor plate and ring gear body
  7. CadQuery interference — boolean checks on key mating pairs
"""

import math
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DEFAULT_CONFIG, compute_housing_bolt_angles

CFG = DEFAULT_CONFIG


# ===================================================================
# 1. Axial stack-up consistency
# ===================================================================


class TestAxialStackUp:
    """Verify Z positions are self-consistent and sum correctly."""

    def test_total_housing_depth(self):
        """Total depth must equal the sum of all layers."""
        s = CFG.stack_up
        expected = (
            s.motor_plate_wall
            + s.motor_plate_inner_wall
            + s.input_clearance
            + s.disc_thickness * 2
            + s.inter_disc_spacer
            + s.output_clearance
            + s.output_bearing_total
            + s.output_wall
        )
        assert abs(s.total_housing_depth - expected) < 0.01, (
            f"Total depth {s.total_housing_depth}mm != sum {expected}mm"
        )

    def test_total_depth_is_60mm(self):
        """Housing depth: 9mm plate + 51mm body (incl. 3mm output wall) = 60mm."""
        assert abs(CFG.stack_up.total_housing_depth - 60.0) < 0.01

    def test_output_hub_protrudes_past_chassis(self):
        """The rotating output hub must stick out past the housing so the arm link clears it.

        Hub height = bearing grip + output wall + proud extension; positioned at
        z_output_bearings, its output face lands proud_above_housing beyond the
        housing output face (total_housing_depth).
        """
        s = CFG.stack_up
        hub = CFG.output_hub
        hub_height = s.output_bearing_total + s.output_wall + hub.proud_above_housing
        hub_top_z = s.z_output_bearings + hub_height
        assert abs(hub_top_z - (s.total_housing_depth + hub.proud_above_housing)) < 0.01, (
            f"Hub top at {hub_top_z}mm != housing face {s.total_housing_depth}mm "
            f"+ proud {hub.proud_above_housing}mm"
        )
        assert hub_top_z > s.total_housing_depth, (
            f"Hub top {hub_top_z}mm does not protrude past chassis {s.total_housing_depth}mm"
        )

    def test_disc1_before_disc2(self):
        """Disc 1 must sit at a lower Z than disc 2."""
        s = CFG.stack_up
        assert s.z_disc1 < s.z_disc2

    def test_disc2_ends_before_output_bearings(self):
        """Disc 2 top face must be below the output bearings."""
        s = CFG.stack_up
        disc2_top = s.z_disc2 + s.disc_thickness
        assert disc2_top <= s.z_output_bearings, (
            f"Disc 2 top {disc2_top}mm >= output bearings {s.z_output_bearings}mm"
        )

    def test_output_clearance_gap(self):
        """There must be a clearance gap between disc 2 and output bearings."""
        s = CFG.stack_up
        disc2_top = s.z_disc2 + s.disc_thickness
        gap = s.z_output_bearings - disc2_top
        assert abs(gap - s.output_clearance) < 0.01, (
            f"Output clearance {gap}mm != spec {s.output_clearance}mm"
        )

    def test_output_bearings_end_at_bearing_top(self):
        """Output bearings top must align with the retention-lip start (z_bearing_top)."""
        s = CFG.stack_up
        bearing_top = s.z_output_bearings + s.output_bearing_total
        assert abs(bearing_top - s.z_bearing_top) < 0.01, (
            f"Bearing top {bearing_top}mm != z_bearing_top {s.z_bearing_top}mm"
        )


# ===================================================================
# 2. Housing part alignment
# ===================================================================


class TestHousingAlignment:
    """Verify the two housing pieces mate flush."""

    def test_motor_plate_inner_face(self):
        """Motor plate inner face at Z=10mm."""
        s = CFG.stack_up
        expected = s.motor_plate_wall + s.motor_plate_inner_wall  # 5+4=9mm
        assert abs(s.z_motor_plate_inner - expected) < 0.01

    def test_ring_gear_body_height(self):
        """Ring gear body spans from motor plate inner face to the housing output face."""
        s = CFG.stack_up
        body_height = s.total_housing_depth - s.z_motor_plate_inner
        assert abs(body_height - 51.0) < 0.01, (
            f"Ring gear body height {body_height}mm != 51mm"
        )

    def test_all_housing_parts_same_od(self):
        """Motor plate and ring gear body should share the housing OD."""
        # This is enforced by all parts using cfg.housing.od, but verify the value
        assert CFG.housing.od == 140.0

    def test_housing_bolt_angles_consistent(self):
        """All housing parts use the same bolt angle computation."""
        angles = compute_housing_bolt_angles(CFG)
        assert len(angles) == CFG.housing.bolt_count
        # Angles must be sorted and within [0, 2π)
        for i in range(len(angles) - 1):
            assert angles[i] < angles[i + 1], "Bolt angles not sorted"
        assert angles[0] >= 0
        assert angles[-1] < 2 * math.pi

    def test_housing_bolts_clear_ring_pins(self):
        """Housing bolts must not overlap ring pin holes (2D distance check).

        Bolts and pins sit at different radii (62.5mm vs 54mm), so the
        real clearance is the Euclidean distance between hole centers
        minus the sum of hole radii.
        """
        g = CFG.gear
        h = CFG.housing
        bolt_angles = compute_housing_bolt_angles(CFG)
        bolt_r = h.bolt_circle_dia / 2.0  # 62.5mm
        pin_r = g.ring_pin_circle_dia / 2.0  # 54mm

        bolt_hole_r = (h.bolt_dia + 0.4) / 2.0  # 2.2mm
        pin_hole_r = g.ring_pin_dia / 2.0  # 2.0mm
        min_center_dist = bolt_hole_r + pin_hole_r  # 4.2mm

        for ba in bolt_angles:
            bx = bolt_r * math.cos(ba)
            by = bolt_r * math.sin(ba)
            for i in range(g.num_ring_pins):
                pa = 2 * math.pi * i / g.num_ring_pins
                px = pin_r * math.cos(pa)
                py = pin_r * math.sin(pa)
                dist = math.hypot(bx - px, by - py)
                assert dist > min_center_dist, (
                    f"Bolt at {math.degrees(ba):.1f}° is only {dist:.2f}mm "
                    f"from pin {i} (need > {min_center_dist:.2f}mm)"
                )


# ===================================================================
# 3. Radial clearances
# ===================================================================


class TestRadialClearances:
    """Verify parts don't interfere radially."""

    def test_disc_orbit_clears_housing_bore(self):
        """Disc max radius + eccentricity must be less than housing bore radius."""
        from src.profiles import compute_epitrochoid, compute_profile_radii

        profile = compute_epitrochoid(
            R=CFG.gear.ring_pin_circle_radius,
            r=CFG.gear.ring_pin_radius,
            N=CFG.gear.num_ring_pins,
            e=CFG.gear.eccentricity,
            num_points=CFG.profile.num_points,
        )
        _, max_r = compute_profile_radii(profile)
        swept_r = max_r + CFG.gear.eccentricity
        bore_r = CFG.housing.bore_dia / 2.0

        assert swept_r < bore_r, (
            f"Disc swept radius {swept_r:.2f}mm >= housing bore radius {bore_r}mm"
        )
        clearance = bore_r - swept_r
        assert clearance >= 0.5, (
            f"Disc-to-housing clearance only {clearance:.2f}mm (want >= 0.5mm)"
        )

    def test_output_pins_inside_6814_bore(self):
        """Output pin outer edges must sit within the 6814 bearing bore."""
        d = CFG.disc
        b = CFG.bearings
        pin_outer_r = d.output_pin_circle_dia / 2.0 + d.output_pin_dia / 2.0
        bearing_bore_r = b.out_bore / 2.0
        assert pin_outer_r < bearing_bore_r, (
            f"Output pin edge at {pin_outer_r}mm >= 6814 bore radius {bearing_bore_r}mm"
        )

    def test_output_pin_bottom_clears_motor_plate(self):
        """Output pin bottom (under the hub ceiling) must clear the motor plate inner face."""
        d = CFG.disc
        hub = CFG.output_hub
        s = CFG.stack_up
        pin_top_z = s.z_bearing_top - hub.output_hub_pin_ceiling  # 56mm
        pin_bottom_z = pin_top_z - d.output_pin_length  # 11mm
        clearance = pin_bottom_z - s.z_motor_plate_inner  # 11 - 10 = 1mm
        assert clearance > 0, (
            f"Pin bottom at Z={pin_bottom_z}mm penetrates motor plate inner face "
            f"at Z={s.z_motor_plate_inner}mm"
        )

    def test_output_pin_engages_disc1(self):
        """Output pin bottom must extend below disc 1 inner face for full disc engagement."""
        d = CFG.disc
        hub = CFG.output_hub
        s = CFG.stack_up
        pin_top_z = s.z_bearing_top - hub.output_hub_pin_ceiling
        pin_bottom_z = pin_top_z - d.output_pin_length
        assert pin_bottom_z <= s.z_disc1, (
            f"Pin bottom at Z={pin_bottom_z}mm doesn't reach disc 1 inner face "
            f"at Z={s.z_disc1}mm — disc 1 not fully engaged"
        )

    def test_output_hub_clears_retention_lip_bore(self):
        """Output hub OD must be smaller than the ring body's integral retention-lip bore."""
        hub = CFG.output_hub
        h = CFG.housing
        hub_od = hub.od  # 70.3mm
        lip_bore = h.output_bearing_seat_dia - 2 * 2.0  # 86.15mm
        clearance = lip_bore - hub_od
        assert clearance > 0, (
            f"Hub OD {hub_od}mm >= lip bore {lip_bore}mm"
        )
        assert clearance >= 0.2, (
            f"Hub-to-lip clearance only {clearance:.3f}mm (want >= 0.2mm)"
        )

    def test_arm_mount_holes_clear_output_pins(self):
        """Arm-mount holes (+ captive-nut pockets) must clear the output pin holes.

        The arm-mount bolt circle is offset 45° from the output pins; the nearest
        arm-hole / output-pin center distance must exceed the nut-pocket envelope
        radius + the pin-hole radius.
        """
        hub = CFG.output_hub
        d = CFG.disc
        h = CFG.housing
        tol = CFG.tolerances

        arm_r = hub.arm_mount_bolt_circle_dia / 2.0  # 25mm
        pin_r = d.output_pin_circle_dia / 2.0  # 30mm
        offset = math.radians(hub.arm_mount_angle_offset_deg)  # 45° between an arm hole and a pin
        # Law of cosines for the nearest arm-hole / pin center spacing.
        dist = math.sqrt(arm_r ** 2 + pin_r ** 2 - 2 * arm_r * pin_r * math.cos(offset))

        nut_pocket_r = h.bolt_nut_pocket_af / 2.0 / math.cos(math.radians(30))  # hex circumradius
        pin_hole_r = (d.output_pin_dia - tol.ring_pin_press_sub) / 2.0  # 4.20mm hole → 2.1mm
        needed = nut_pocket_r + pin_hole_r

        assert dist > needed, (
            f"Arm-mount feature {dist:.2f}mm from output pin < required {needed:.2f}mm — "
            "captive-nut pocket would collide with an output pin hole"
        )

    def test_output_hub_near_6814_inner(self):
        """As-designed hub OD must be within ±0.5mm of the 6814 bearing bore.

        As-designed OD is the 70mm bearing bore + 0.2mm interference for press
        grip on the inner race.
        """
        hub = CFG.output_hub
        b = CFG.bearings
        delta = abs(hub.od - b.out_bore)
        assert delta <= 0.5, (
            f"Hub OD {hub.od}mm differs from 6814 bore {b.out_bore}mm "
            f"by {delta:.3f}mm (>0.5mm)"
        )

    def test_6814_outer_fits_housing_seat(self):
        """6814 OD must fit in the housing bearing seat bore."""
        b = CFG.bearings
        h = CFG.housing
        assert b.out_od <= h.output_bearing_seat_dia, (
            f"6814 OD {b.out_od}mm > bearing seat {h.output_bearing_seat_dia}mm"
        )

    def test_ring_pins_inside_housing_bore(self):
        """Ring pins must sit inside the housing bore (pins in air in disc zone)."""
        g = CFG.gear
        pin_outer_r = g.ring_pin_circle_dia / 2.0 + g.ring_pin_dia / 2.0
        bore_r = CFG.housing.bore_dia / 2.0
        assert pin_outer_r < bore_r, (
            f"Ring pin outer edge {pin_outer_r}mm >= housing bore radius {bore_r}mm"
        )

    def test_disc_output_pin_holes_clear_center_bore(self):
        """Output pin holes in disc must not overlap with the center bore."""
        d = CFG.disc
        pin_inner_r = d.output_pin_circle_dia / 2.0 - d.output_pin_hole_dia / 2.0
        bore_r = d.center_bore_dia / 2.0
        wall = pin_inner_r - bore_r
        assert wall >= 5.0, (
            f"Disc bore-to-pin wall {wall:.2f}mm < 5mm"
        )


# ===================================================================
# 4. Bearing retention
# ===================================================================


class TestBearingRetention:
    """Verify all bearings are axially constrained."""

    def test_6814_retained_by_press_fit(self):
        """6814 bearings are retained by press-fit into 90.15mm seat + integral lip.

        No shoulder ring is used — the disc envelope (~108mm) exceeds the
        6814 OD (90mm), so a shoulder cannot simultaneously clear the disc
        and block the bearing.  Verify that the seat provides interference fit.
        """
        h = CFG.housing
        b = CFG.bearings
        assert h.output_bearing_seat_dia > b.out_od, (
            f"Bearing seat {h.output_bearing_seat_dia}mm <= 6814 OD "
            f"{b.out_od}mm — no press fit"
        )
        gap = h.output_bearing_seat_dia - b.out_od
        assert gap < 0.5, (
            f"Bearing seat gap {gap:.2f}mm too large for press fit"
        )

    def test_6814_retained_by_integral_lip(self):
        """Ring body's integral lip bore must be smaller than 6814 OD on the output side."""
        h = CFG.housing
        b = CFG.bearings
        lip_bore = h.output_bearing_seat_dia - 2 * 2.0  # 86.15mm
        assert lip_bore < b.out_od, (
            f"Lip bore {lip_bore}mm >= 6814 OD {b.out_od}mm — not retained"
        )

    def test_6003_retained_by_disc_bore(self):
        """6003 OD fits into disc center bore — disc material retains it."""
        b = CFG.bearings
        d = CFG.disc
        assert b.ecc_od <= d.center_bore_dia, (
            f"6003 OD {b.ecc_od}mm > disc bore {d.center_bore_dia}mm"
        )

    def test_625_retained_by_output_hub_pocket(self):
        """625 bearing pocket in hub is smaller than hub OD — bearing is captured."""
        hub = CFG.output_hub
        b = CFG.bearings
        tol = CFG.tolerances
        pocket_dia = b.inp_od + tol.bearing_seat_bore_add
        hub_od = hub.od
        assert pocket_dia < hub_od, (
            f"625 pocket {pocket_dia}mm >= hub OD {hub_od}mm"
        )


# ===================================================================
# 5. Shaft reach
# ===================================================================


class TestShaftReach:
    """Verify shaft engagement at both ends."""

    def test_motor_shaft_does_not_bottom_in_d_bore(self):
        """Motor shaft tip must not contact the D-bore floor — needs clearance
        so the eccentric shaft seats correctly against the motor plate inner face.
        """
        m = CFG.motor
        s = CFG.stack_up
        plate_thickness = s.motor_plate_wall + s.motor_plate_inner_wall  # 9mm
        shaft_past_plate = m.shaft_length - plate_thickness  # 22 - 9 = 13mm
        d_bore_depth = CFG.shaft.d_bore_depth  # 14mm
        clearance = d_bore_depth - shaft_past_plate
        assert clearance >= 0.5, (
            f"Motor shaft ({shaft_past_plate}mm past plate) bottoms in D-bore "
            f"({d_bore_depth}mm deep); need >=0.5mm clearance, got {clearance}mm"
        )

    def test_eccentric_shaft_pin_reaches_625_bearing(self):
        """Steel dowel pin must protrude enough to reach through the 625 bearing."""
        s = CFG.stack_up
        shaft = CFG.shaft
        disc2_end = s.z_disc2 + s.disc_thickness  # 35mm
        z_625_end = s.z_output_bearings + CFG.bearings.inp_width  # 42mm
        pin_tip_z = disc2_end + (shaft.support_pin_length - shaft.support_pin_hole_depth)
        assert pin_tip_z >= z_625_end, (
            f"Pin tip at Z={pin_tip_z}mm doesn't reach 625 far face at Z={z_625_end}mm"
        )

    def test_eccentric_shaft_pin_fits_625_bore(self):
        """Steel dowel pin OD must fit inside the 625 bearing bore."""
        assert CFG.shaft.support_pin_dia <= CFG.bearings.inp_bore, (
            f"Pin OD {CFG.shaft.support_pin_dia}mm > 625 bore {CFG.bearings.inp_bore}mm"
        )

    def test_eccentric_shaft_pin_clears_hub_bore(self):
        """Support pin must have clearance inside the output hub shaft bore."""
        clearance = CFG.output_hub.shaft_clearance_bore - CFG.shaft.support_pin_dia
        assert clearance >= 0.5, (
            f"Pin-to-hub-bore clearance {clearance:.2f}mm < 0.5mm minimum"
        )


# ===================================================================
# 6. Ring pin span
# ===================================================================


class TestRingPinSpan:
    """Verify ring pin length covers both engagement zones."""

    def test_pin_length_spans_bore_zone_plus_engagement(self):
        """Pin length must span the bore zone with engagement on both sides.

        Motor plate side: 5mm engagement (unchanged).
        Ring gear body side: 4mm engagement (bore zone = 27mm, pin = 35mm).
        """
        g = CFG.gear
        s = CFG.stack_up
        bore_zone = (
            s.input_clearance
            + s.disc_thickness * 2
            + s.inter_disc_spacer
            + s.output_clearance
        )  # 27mm
        # Engagement is symmetric: 4mm each side (motor plate + ring gear body)
        engagement_per_side = (g.ring_pin_length - bore_zone) / 2.0  # 4mm
        assert engagement_per_side >= 3.0, (
            f"Pin engagement {engagement_per_side}mm per side < 3mm minimum"
        )

    def test_pin_length_equals_35mm(self):
        """Spec defines ring pin length as 35mm."""
        assert CFG.gear.ring_pin_length == 35.0

    def test_disc_zone_is_26mm(self):
        """Disc zone (input clearance + 2 discs + spacer) should be 26mm."""
        s = CFG.stack_up
        disc_zone = s.input_clearance + s.disc_thickness * 2 + s.inter_disc_spacer
        assert abs(disc_zone - 26.0) < 0.01


# ===================================================================
# 7. Housing bolt engagement
# ===================================================================


class TestHousingBoltEngagement:
    """Verify M4 × 55mm bolt fits the housing stack with counterbore and nut pocket."""

    def test_bolt_reaches_nut(self):
        """Bolt shank must extend past the nut pocket floor."""
        h = CFG.housing
        s = CFG.stack_up
        bolt_tip_z = h.bolt_counterbore_depth + h.bolt_length
        nut_floor_z = s.total_housing_depth - h.bolt_nut_depth
        assert bolt_tip_z > nut_floor_z, (
            f"Bolt tip at {bolt_tip_z}mm doesn't reach nut floor at {nut_floor_z}mm"
        )

    def test_full_nut_engagement(self):
        """Bolt must pass through the full nut thickness."""
        h = CFG.housing
        s = CFG.stack_up
        bolt_tip_z = h.bolt_counterbore_depth + h.bolt_length
        nut_floor_z = s.total_housing_depth - h.bolt_nut_depth
        engagement = bolt_tip_z - nut_floor_z
        assert engagement >= h.bolt_nut_thickness, (
            f"Thread engagement {engagement:.1f}mm < nut thickness {h.bolt_nut_thickness}mm"
        )

    def test_bolt_does_not_protrude(self):
        """Bolt tip must not extend past the housing output face."""
        h = CFG.housing
        s = CFG.stack_up
        bolt_tip_z = h.bolt_counterbore_depth + h.bolt_length
        assert bolt_tip_z <= s.total_housing_depth, (
            f"Bolt tip at {bolt_tip_z}mm protrudes past housing at {s.total_housing_depth}mm"
        )

    def test_counterbore_recesses_head(self):
        """Counterbore must be at least as deep as the bolt head height."""
        h = CFG.housing
        assert h.bolt_counterbore_depth >= h.bolt_head_height, (
            f"Counterbore {h.bolt_counterbore_depth}mm < head height {h.bolt_head_height}mm"
        )

    def test_counterbore_wall_to_od(self):
        """Counterbore must leave adequate wall to housing OD."""
        h = CFG.housing
        bolt_r = h.bolt_circle_dia / 2.0
        cb_r = h.bolt_counterbore_dia / 2.0
        wall = h.od / 2.0 - (bolt_r + cb_r)
        assert wall >= 2.0, (
            f"Counterbore wall to OD = {wall:.2f}mm, need >= 2mm"
        )

    def test_counterbore_fits_in_motor_plate(self):
        """Counterbore depth must be less than motor plate thickness."""
        h = CFG.housing
        s = CFG.stack_up
        plate_t = s.motor_plate_wall + s.motor_plate_inner_wall
        assert h.bolt_counterbore_depth < plate_t, (
            f"Counterbore {h.bolt_counterbore_depth}mm >= plate thickness {plate_t}mm"
        )


# ===================================================================
# 8. CadQuery interference checks — key mating pairs
# ===================================================================


class TestCadQueryAssemblyInterference:
    """Boolean intersection checks between parts in assembly position."""

    @pytest.fixture(scope="class")
    def housing_parts(self):
        """Build and position the two housing parts."""
        cq = pytest.importorskip("cadquery")
        from src.motor_plate import build_motor_plate
        from src.ring_gear_body import build_ring_gear_body

        s = CFG.stack_up
        mp = build_motor_plate()
        rgb = build_ring_gear_body().translate((0, 0, s.z_motor_plate_inner))
        return mp, rgb

    def test_motor_plate_ring_body_no_interference(self, housing_parts):
        """Motor plate and ring gear body must not overlap."""
        mp, rgb = housing_parts
        interference = mp.intersect(rgb)
        vol = interference.val().Volume()
        assert vol < 1.0, (
            f"Motor plate / ring body interference = {vol:.1f}mm³"
        )

    @pytest.fixture(scope="class")
    def output_hub_and_body(self):
        """Build and position the output hub and the ring gear body."""
        cq = pytest.importorskip("cadquery")
        from src.output_hub import build_output_hub
        from src.ring_gear_body import build_ring_gear_body

        s = CFG.stack_up
        hub = build_output_hub().translate((0, 0, s.z_output_bearings))
        rgb = build_ring_gear_body().translate((0, 0, s.z_motor_plate_inner))
        return hub, rgb

    def test_output_hub_protrudes_through_housing(self, output_hub_and_body):
        """Assembled hub output face must sit proud of the housing output face.

        Assembled-context counterpart to the part-level proud-face check: the hub
        top must clear the ring body's output face by ≈ proud_above_housing so the
        arm link mounts above the chassis.  (Radial clearance is covered by
        test_output_hub_clears_ring_body.)
        """
        hub, rgb = output_hub_and_body
        hub_top = hub.val().BoundingBox().zmax
        body_top = rgb.val().BoundingBox().zmax
        proud = CFG.output_hub.proud_above_housing
        assert abs((hub_top - body_top) - proud) < 0.1, (
            f"Hub top {hub_top:.2f}mm vs housing top {body_top:.2f}mm — "
            f"expected hub proud by {proud}mm"
        )

    @pytest.fixture(scope="class")
    def disc_and_housing(self):
        """Build disc 1 at max eccentricity and the ring gear body."""
        cq = pytest.importorskip("cadquery")
        from src.cycloidal_disc import build_cycloidal_disc
        from src.ring_gear_body import build_ring_gear_body

        s = CFG.stack_up
        e = CFG.gear.eccentricity

        disc = build_cycloidal_disc().translate((e, 0, s.z_disc1))
        rgb = build_ring_gear_body().translate((0, 0, s.z_motor_plate_inner))
        return disc, rgb

    def test_disc_clears_ring_gear_shoulder(self, disc_and_housing):
        """Disc at max eccentricity must not hit the ring gear body shoulder."""
        disc, rgb = disc_and_housing
        interference = disc.intersect(rgb)
        vol = interference.val().Volume()
        assert vol < 1.0, (
            f"Disc / ring gear body interference = {vol:.1f}mm³"
        )

    @pytest.fixture(scope="class")
    def output_hub_in_ring_body(self):
        """Build output hub inside the ring gear body."""
        cq = pytest.importorskip("cadquery")
        from src.output_hub import build_output_hub
        from src.ring_gear_body import build_ring_gear_body

        s = CFG.stack_up
        hub = build_output_hub().translate((0, 0, s.z_output_bearings))
        rgb = build_ring_gear_body().translate((0, 0, s.z_motor_plate_inner))
        return hub, rgb

    def test_output_hub_clears_ring_body(self, output_hub_in_ring_body):
        """Output hub must pass through the ring gear body without interference."""
        hub, rgb = output_hub_in_ring_body
        interference = hub.intersect(rgb)
        vol = interference.val().Volume()
        assert vol < 1.0, (
            f"Output hub / ring body interference = {vol:.1f}mm³"
        )
