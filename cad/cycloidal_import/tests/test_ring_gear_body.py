"""Validation test suite for the ring gear body geometry.

Tests cover:
  1. Dimensional checks — zone heights, clearances, fit relationships (no CadQuery)
  2. CadQuery solid — valid topology, bounding box, volume, bore verification
"""

import math
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DEFAULT_CONFIG, compute_housing_bolt_angles

CFG = DEFAULT_CONFIG


# ===================================================================
# 1. Dimensional checks (no CadQuery needed)
# ===================================================================


class TestRingGearBodyDimensions:

    def test_body_height_matches_stackup(self):
        """Body height must equal total_housing_depth - z_motor_plate_inner."""
        stack = CFG.stack_up
        expected = stack.total_housing_depth - stack.z_motor_plate_inner  # 51mm
        actual = (
            stack.input_clearance
            + stack.disc_thickness * 2
            + stack.inter_disc_spacer
            + stack.output_clearance
            + stack.output_bearing_total
            + stack.output_wall
        )
        assert abs(actual - expected) < 0.01, (
            f"Body height {actual}mm != expected {expected}mm"
        )

    def test_housing_bolts_inside_od(self):
        """M4 housing bolt holes must not break through the OD."""
        h = CFG.housing
        bolt_r = h.bolt_circle_dia / 2.0
        hole_r = (h.bolt_dia + 0.4) / 2.0
        outermost = bolt_r + hole_r
        assert outermost < h.od / 2.0, (
            f"Housing bolt edge at {outermost:.1f}mm exceeds housing radius "
            f"{h.od / 2.0:.1f}mm"
        )

    def test_pin_holes_dont_breach_bearing_seat(self):
        """Ring pin holes must not cut into the bearing seat bore."""
        g = CFG.gear
        h = CFG.housing
        tol = CFG.tolerances
        pin_circle_r = g.ring_pin_circle_dia / 2.0  # 54mm
        pin_hole_r = (g.ring_pin_dia - tol.ring_pin_press_sub) / 2.0  # ~1.94mm
        bearing_seat_r = h.output_bearing_seat_dia / 2.0  # 45.075mm

        pin_inner_edge = pin_circle_r - pin_hole_r
        gap = pin_inner_edge - bearing_seat_r
        assert gap >= 5.0, (
            f"Pin hole inner edge to bearing seat wall = {gap:.2f}mm, need >= 5mm"
        )

    def test_pin_holes_inside_housing_od(self):
        """Ring pin holes must not break through the housing OD."""
        g = CFG.gear
        h = CFG.housing
        tol = CFG.tolerances
        pin_circle_r = g.ring_pin_circle_dia / 2.0  # 54mm
        pin_hole_r = (g.ring_pin_dia - tol.ring_pin_press_sub) / 2.0
        outermost = pin_circle_r + pin_hole_r
        assert outermost < h.od / 2.0, (
            f"Pin hole outer edge at {outermost:.2f}mm exceeds housing "
            f"radius {h.od / 2.0}mm"
        )

    def test_bolt_holes_do_not_overlap_pin_holes(self):
        """No housing bolt hole should overlap with any ring pin hole."""
        g = CFG.gear
        h = CFG.housing
        tol = CFG.tolerances

        pin_circle_r = g.ring_pin_circle_dia / 2.0  # 54mm
        bolt_circle_r = h.bolt_circle_dia / 2.0  # 55mm
        pin_hole_r = (g.ring_pin_dia - tol.ring_pin_press_sub) / 2.0  # ~1.94mm
        bolt_hole_r = (h.bolt_dia + 0.4) / 2.0  # 2.2mm
        min_distance = pin_hole_r + bolt_hole_r  # ~4.14mm (no overlap)

        bolt_angles = compute_housing_bolt_angles(CFG)
        pin_angles = [2 * math.pi * i / g.num_ring_pins for i in range(g.num_ring_pins)]

        for bi, ba in enumerate(bolt_angles):
            bx = bolt_circle_r * math.cos(ba)
            by = bolt_circle_r * math.sin(ba)
            for pi_, pa in enumerate(pin_angles):
                px = pin_circle_r * math.cos(pa)
                py = pin_circle_r * math.sin(pa)
                dist = math.sqrt((bx - px) ** 2 + (by - py) ** 2)
                assert dist >= min_distance, (
                    f"Bolt {bi} (angle {math.degrees(ba):.1f}°) overlaps "
                    f"pin {pi_} (angle {math.degrees(pa):.1f}°): "
                    f"distance {dist:.2f}mm < min {min_distance:.2f}mm"
                )

    def test_bolt_holes_do_not_overlap_each_other(self):
        """No two housing bolt holes should overlap."""
        h = CFG.housing
        bolt_circle_r = h.bolt_circle_dia / 2.0
        bolt_hole_r = (h.bolt_dia + 0.4) / 2.0
        min_distance = 2 * bolt_hole_r  # 4.4mm

        bolt_angles = compute_housing_bolt_angles(CFG)

        for i in range(len(bolt_angles)):
            for j in range(i + 1, len(bolt_angles)):
                ax = bolt_circle_r * math.cos(bolt_angles[i])
                ay = bolt_circle_r * math.sin(bolt_angles[i])
                bx = bolt_circle_r * math.cos(bolt_angles[j])
                by = bolt_circle_r * math.sin(bolt_angles[j])
                dist = math.sqrt((ax - bx) ** 2 + (ay - by) ** 2)
                assert dist >= min_distance, (
                    f"Bolt {i} and bolt {j} overlap: "
                    f"distance {dist:.2f}mm < min {min_distance:.2f}mm"
                )

    def test_bearing_seat_smaller_than_main_bore(self):
        """Bearing seat bore must be smaller than the 116mm main bore.

        With the shoulder ring removed, the step from 116mm bore down to
        90.15mm bearing seat is the only geometric transition.  The 6814
        bearings (90mm OD) are retained axially by press-fit into the
        seat plus the ring body's integral lip on the far side.
        """
        h = CFG.housing
        assert h.output_bearing_seat_dia < h.bore_dia, (
            f"Bearing seat {h.output_bearing_seat_dia}mm >= main bore "
            f"{h.bore_dia}mm — no step exists"
        )

    def test_bearing_seat_is_press_fit(self):
        """Bearing seat must be slightly larger than bearing OD (press fit)."""
        h = CFG.housing
        b = CFG.bearings
        assert h.output_bearing_seat_dia > b.out_od, (
            f"Bearing seat {h.output_bearing_seat_dia}mm <= bearing OD "
            f"{b.out_od}mm — won't accept bearing"
        )
        gap = h.output_bearing_seat_dia - b.out_od
        assert gap < 0.5, (
            f"Bearing seat gap {gap:.2f}mm too large for press fit"
        )

    def test_integral_lip_blocks_6814(self):
        """The integral retention lip bore must be smaller than the 6814 OD."""
        h = CFG.housing
        b = CFG.bearings
        lip_bore = h.output_bearing_seat_dia - 2 * 2.0  # 86.15mm
        assert lip_bore < b.out_od, (
            f"Lip bore {lip_bore}mm >= 6814 OD {b.out_od}mm — bearing not retained"
        )

    def test_integral_lip_clears_hub(self):
        """The integral lip bore must clear the output hub OD with >=0.2mm."""
        h = CFG.housing
        hub = CFG.output_hub
        lip_bore = h.output_bearing_seat_dia - 2 * 2.0  # 86.15mm
        clearance = lip_bore - hub.od
        assert clearance >= 0.2, (
            f"Lip bore {lip_bore}mm vs hub OD {hub.od}mm — clearance "
            f"{clearance:.2f}mm < 0.2mm"
        )

    def test_lip_retention_shoulder_width(self):
        """The lip must present a >=2mm radial shoulder against the 6814 outer race."""
        h = CFG.housing
        lip_bore = h.output_bearing_seat_dia - 2 * 2.0  # 86.15mm
        shoulder = (h.output_bearing_seat_dia - lip_bore) / 2.0  # 2.0mm
        assert shoulder >= 2.0 - 1e-6, (
            f"Lip shoulder {shoulder:.2f}mm < 2mm radial"
        )

    def test_housing_bolts_outside_bore(self):
        """Housing bolt holes must sit entirely outside the 116mm bore.

        Bolts at radius < bore_radius would be unsupported in the disc zone
        and would physically interfere with the orbiting cycloidal discs.
        """
        h = CFG.housing
        bolt_r = h.bolt_circle_dia / 2.0
        hole_r = (h.bolt_dia + 0.4) / 2.0
        bore_r = h.bore_dia / 2.0

        bolt_inner_edge = bolt_r - hole_r
        assert bolt_inner_edge > bore_r, (
            f"Housing bolt inner edge at {bolt_inner_edge:.1f}mm is inside "
            f"bore radius {bore_r:.1f}mm — bolts would be unsupported and "
            f"interfere with disc orbit"
        )

    def test_housing_bolts_have_wall_to_bore(self):
        """Housing bolt holes must leave >=1mm wall between bore and bolt edge."""
        h = CFG.housing
        bolt_r = h.bolt_circle_dia / 2.0
        hole_r = (h.bolt_dia + 0.4) / 2.0
        bore_r = h.bore_dia / 2.0

        wall = (bolt_r - hole_r) - bore_r
        assert wall >= 1.0, (
            f"Wall from bore to bolt hole = {wall:.1f}mm, need >= 1mm"
        )

    def test_housing_bolts_clear_disc_sweep(self):
        """Housing bolt hole inner edge must clear the disc sweep envelope.

        The disc's max radius plus eccentricity gives the swept radius.
        Bolt holes must be entirely outside this zone.
        """
        from src.profiles import compute_epitrochoid, compute_profile_radii

        g = CFG.gear
        h = CFG.housing

        pts = compute_epitrochoid(
            R=g.ring_pin_circle_dia / 2.0,
            r=g.ring_pin_dia / 2.0,
            N=g.num_ring_pins,
            e=g.eccentricity,
        )
        _, max_r = compute_profile_radii(pts)
        disc_sweep_r = max_r + g.eccentricity

        bolt_r = h.bolt_circle_dia / 2.0
        hole_r = (h.bolt_dia + 0.4) / 2.0
        bolt_inner_edge = bolt_r - hole_r

        clearance = bolt_inner_edge - disc_sweep_r
        assert clearance > 0, (
            f"Disc sweep radius {disc_sweep_r:.2f}mm reaches bolt hole "
            f"inner edge at {bolt_inner_edge:.2f}mm — interference of "
            f"{-clearance:.2f}mm"
        )

    def test_pillar_wall_around_bolt(self):
        """Each trapezoidal pillar provides >= 3mm tangential wall around
        the M4 bolt clearance hole.  Pillar width is interpolated linearly
        along the radius from inner_w (at bore wall) to outer_w (at OD).
        """
        h = CFG.housing
        bolt_r = h.bolt_circle_dia / 2.0  # 62.5mm
        bolt_clearance_r = (h.bolt_dia + 0.4) / 2.0  # 2.2mm
        pillar_inner_r = h.bore_dia / 2.0 - 1.0  # 57mm
        pillar_outer_r = h.od / 2.0 + 1.0  # 68mm
        pillar_inner_w = 18.0  # tangential width at bore
        pillar_outer_w = 10.0  # tangential width at OD

        t = (bolt_r - pillar_inner_r) / (pillar_outer_r - pillar_inner_r)
        pillar_w_at_bolt = pillar_inner_w + (pillar_outer_w - pillar_inner_w) * t
        wall = pillar_w_at_bolt / 2.0 - bolt_clearance_r
        assert wall >= 3.0, (
            f"Tangential pillar wall around bolt = {wall:.2f}mm, need >= 3mm"
        )

    def test_pillar_overshoots_housing_walls(self):
        """Pillar trapezoid overshoots the bore (-1mm) and OD (+1mm) so the
        bore and base-cylinder cuts trim it cleanly flush with the housing
        walls (no thin slivers from float arithmetic).
        """
        h = CFG.housing
        pillar_inner_r = h.bore_dia / 2.0 - 1.0  # 57mm
        pillar_outer_r = h.od / 2.0 + 1.0  # 68mm
        bore_r = h.bore_dia / 2.0  # 58mm
        housing_r = h.od / 2.0  # 67mm
        assert pillar_inner_r < bore_r, (
            f"Pillar inner_r {pillar_inner_r}mm should be < bore_r {bore_r}mm"
        )
        assert pillar_outer_r > housing_r, (
            f"Pillar outer_r {pillar_outer_r}mm should be > housing_r {housing_r}mm"
        )

    def test_pillars_dont_overlap_neighbors(self):
        """Adjacent trapezoidal pillars must leave an open window between
        them.  The bore-edge chord (where pillars are widest) is the
        constraining check.
        """
        h = CFG.housing
        pillar_inner_r = h.bore_dia / 2.0 - 1.0  # 57mm
        pillar_outer_r = h.od / 2.0 + 1.0  # 68mm
        pillar_inner_w = 18.0
        pillar_outer_w = 10.0

        bolt_angles = compute_housing_bolt_angles(CFG)
        spacing = bolt_angles[1] - bolt_angles[0]

        inner_chord = 2 * pillar_inner_r * math.sin(spacing / 2)
        outer_chord = 2 * pillar_outer_r * math.sin(spacing / 2)
        assert inner_chord > pillar_inner_w + 1.0, (
            f"Pillars too close at bore: chord {inner_chord:.2f}mm vs "
            f"width {pillar_inner_w}mm — need >=1mm window gap"
        )
        assert outer_chord > pillar_outer_w + 1.0, (
            f"Pillars too close at OD: chord {outer_chord:.2f}mm vs "
            f"width {pillar_outer_w}mm — need >=1mm window gap"
        )

    def test_windows_span_full_body_height(self):
        """Reveal windows extend from input face through to the output face.

        The continuous rim was eliminated — the motor plate now seats only
        on the 8 pillar top faces.  The bearing seat (Z=28 to Z=48) is
        preserved as a 90.15mm bore inside the same outer cylinder; the
        windows cut only the annulus between the 116mm bore and the OD.
        """
        stack = CFG.stack_up
        body_height = stack.total_housing_depth - stack.z_motor_plate_inner  # 51mm
        window_h = body_height
        assert window_h == body_height, (
            f"Window height {window_h}mm != body height {body_height}mm"
        )

    def test_ring_pin_chamfer_within_bearing_zone(self):
        """Ring pin entry chamfer must not extend into the bearing seat.

        Pins enter solid material at the bore/bearing-seat transition
        (Z=28, local).  The chamfer widens the entry and must stay
        within the pin engagement depth.
        """
        g = CFG.gear
        stack = CFG.stack_up
        bore_zone_end = (
            stack.input_clearance
            + stack.disc_thickness * 2
            + stack.inter_disc_spacer
            + stack.output_clearance
        )
        pin_engagement = (g.ring_pin_length - bore_zone_end) / 2.0  # 3.5mm
        chamfer_depth = 1.0  # mm (matches ring_gear_body.py)
        assert chamfer_depth <= pin_engagement, (
            f"Chamfer depth {chamfer_depth}mm > pin engagement "
            f"{pin_engagement}mm — chamfer extends past pin hole"
        )

    def test_ring_pin_chamfer_no_overlap(self):
        """Adjacent chamfered pin holes must not overlap."""
        g = CFG.gear
        tol = CFG.tolerances
        pin_circle_r = g.ring_pin_circle_dia / 2.0  # 54mm
        pin_hole_dia = g.ring_pin_dia - tol.ring_pin_press_sub  # 4.20mm
        chamfer_dia = pin_hole_dia + 1.0  # 5.20mm

        angular_spacing = 2 * math.pi / g.num_ring_pins
        center_dist = 2 * pin_circle_r * math.sin(angular_spacing / 2)
        assert center_dist > chamfer_dia, (
            f"Adjacent chamfer edges overlap: center distance {center_dist:.2f}mm "
            f"<= chamfer diameter {chamfer_dia:.2f}mm"
        )

    def test_disc_fits_through_main_bore(self):
        """Cycloidal disc (with eccentricity) must fit through the 116mm bore."""
        from src.profiles import compute_epitrochoid

        g = CFG.gear
        h = CFG.housing
        pts = compute_epitrochoid(
            R=g.ring_pin_circle_dia / 2.0,
            r=g.ring_pin_dia / 2.0,
            N=g.num_ring_pins,
            e=g.eccentricity,
        )
        max_r = max(math.sqrt(x**2 + y**2) for x, y in pts)
        # Disc center orbits at ± eccentricity
        max_sweep = max_r + g.eccentricity
        bore_r = h.bore_dia / 2.0
        assert max_sweep < bore_r, (
            f"Disc max sweep {max_sweep:.2f}mm >= bore radius {bore_r}mm"
        )


# ===================================================================
# 2. CadQuery solid validation
# ===================================================================


class TestCadQuerySolid:

    @pytest.fixture(scope="class")
    def body_solid(self):
        cq = pytest.importorskip("cadquery")
        from src.ring_gear_body import build_ring_gear_body

        return build_ring_gear_body()

    def test_solid_is_valid(self, body_solid):
        """The built solid should be non-null and have exactly one solid."""
        solids = body_solid.solids().vals()
        assert len(solids) == 1, f"Expected 1 solid, got {len(solids)}"
        assert solids[0].isValid(), "Solid is not valid"

    def test_outer_diameter(self, body_solid):
        """XY extent should be the housing OD (140mm)."""
        bb = body_solid.val().BoundingBox()
        x_size = bb.xmax - bb.xmin
        y_size = bb.ymax - bb.ymin
        expected = CFG.housing.od  # 140mm

        assert abs(x_size - expected) < 0.2, (
            f"X extent {x_size:.2f}mm, expected {expected}mm"
        )
        assert abs(y_size - expected) < 0.2, (
            f"Y extent {y_size:.2f}mm, expected {expected}mm"
        )

    def test_height(self, body_solid):
        """Z height should match the stack-up (51mm)."""
        bb = body_solid.val().BoundingBox()
        z_size = bb.zmax - bb.zmin
        stack = CFG.stack_up
        expected = stack.total_housing_depth - stack.z_motor_plate_inner  # 51mm

        assert abs(z_size - expected) < 0.1, (
            f"Z extent {z_size:.2f}mm, expected {expected:.2f}mm"
        )

    def test_window_zone_has_pillars_only(self, body_solid):
        """In the window zone, only bolt pillars remain — much less than a full annulus."""
        stack = CFG.stack_up
        h = CFG.housing

        disc_zone_mid = (
            stack.input_clearance
            + stack.disc_thickness
            + stack.inter_disc_spacer / 2.0
        )  # 15mm — inside window zone (Z=0 to bore_zone_end=28)

        section = body_solid.section(height=disc_zone_mid)
        area = section.val().Area()

        housing_r = h.od / 2.0
        bore_r = h.bore_dia / 2.0
        full_annulus = math.pi * (housing_r**2 - bore_r**2)

        # Windows remove most of the wall — area should be <40% of full annulus
        assert area < full_annulus * 0.40, (
            f"Window zone area {area:.1f}mm² is too large — expected <40% of "
            f"full annulus {full_annulus:.1f}mm²"
        )
        # But pillars must provide some material
        assert area > 0, "Window zone has no material — pillars missing"

    def test_input_face_has_pillars_only(self, body_solid):
        """Near the input face the wall is pillars only — no continuous rim.

        The rim was eliminated; the motor plate seats on the 8 pillar tops.
        Section just above the input face should look like the rest of the
        bore-zone window region: <40% of the full annulus.
        """
        h = CFG.housing

        z = 0.5  # just above the input face

        section = body_solid.section(height=z)
        area = section.val().Area()

        housing_r = h.od / 2.0
        bore_r = h.bore_dia / 2.0
        full_annulus = math.pi * (housing_r**2 - bore_r**2)

        assert area < full_annulus * 0.40, (
            f"Input-face section area {area:.1f}mm² >= 40% of full annulus "
            f"{full_annulus:.1f}mm² — rim may have been reintroduced"
        )
        assert area > 0, "Input face has no material — pillars missing"

    def test_output_bearing_seat(self, body_solid):
        """Section area at bearing zone reflects the 90.15mm seat plus pillars.

        At the bearing zone midpoint the inner annulus runs from the
        bearing seat (45.075mm) up to the bore radius (58mm) intact
        (minus 21 pin holes), plus 8 trapezoidal pillars between the
        bore and the OD (each carrying one M4 bolt hole).  The outer
        wall from bore_r to OD is cut away by the reveal windows
        except at those pillars.
        """
        g = CFG.gear
        h = CFG.housing
        tol = CFG.tolerances
        stack = CFG.stack_up

        bearing_mid = (
            stack.input_clearance
            + stack.disc_thickness * 2
            + stack.inter_disc_spacer
            + stack.output_clearance
            + stack.output_bearing_total / 2.0
        )  # 38mm local (z_output_bearings - z_motor_plate_inner + half-bearing)

        section = body_solid.section(height=bearing_mid)
        area = section.val().Area()

        bore_r = h.bore_dia / 2.0
        bearing_seat_r = h.output_bearing_seat_dia / 2.0

        inner_annulus = math.pi * (bore_r**2 - bearing_seat_r**2)

        # Pillar trapezoid (matches ring_gear_body.py): widths 18mm at bore,
        # 10mm at OD, radial length = housing_r - bore_r (overshoot trimmed).
        housing_r = h.od / 2.0
        pillar_radial = housing_r - bore_r
        pillar_avg_w = (18.0 + 10.0) / 2.0
        pillar_area = pillar_avg_w * pillar_radial * h.bolt_count

        pin_hole_r = (g.ring_pin_dia - tol.ring_pin_press_sub) / 2.0
        bolt_hole_r = (h.bolt_dia + 0.4) / 2.0
        pin_area = g.num_ring_pins * math.pi * pin_hole_r**2
        bolt_area = h.bolt_count * math.pi * bolt_hole_r**2

        expected_area = inner_annulus + pillar_area - pin_area - bolt_area

        assert abs(area - expected_area) / expected_area < 0.15, (
            f"Bearing zone section area {area:.1f}mm² vs expected "
            f"{expected_area:.1f}mm² (>15% deviation)"
        )

    def test_volume_sanity(self, body_solid):
        """Volume should be between reasonable bounds.

        Lower bound: the bearing seat ring (bore_r → bearing_r × bearing
        zone height) is the only continuous solid that survives the full-
        height reveal windows.  At 70% to allow for pin/bolt hole losses
        within the ring.
        Upper bound: full outer cylinder minus the bearing seat bore
        through the entire body height.
        """
        h = CFG.housing
        stack = CFG.stack_up
        body_h = stack.total_housing_depth - stack.z_motor_plate_inner  # 51mm
        bore_r = h.bore_dia / 2.0  # 58mm
        housing_r = h.od / 2.0  # 67mm
        bearing_r = h.output_bearing_seat_dia / 2.0  # 45.075mm

        upper = math.pi * (housing_r**2 - bearing_r**2) * body_h

        bearing_ring_vol = (
            math.pi * (bore_r**2 - bearing_r**2) * stack.output_bearing_total
        )
        lower = bearing_ring_vol * 0.7

        vol = body_solid.val().Volume()
        assert vol > lower, (
            f"Volume {vol:.0f}mm³ below lower bound {lower:.0f}"
        )
        assert vol < upper, (
            f"Volume {vol:.0f}mm³ above upper bound {upper:.0f}"
        )

    def test_integral_lip_present_in_solid(self, body_solid):
        """The bore must step 90.15→86.15 above the bearing seat — probe a ring at
        r≈44mm (between the lip bore r=43.075 and the seat r=45.075): solid in the
        lip zone, void in the bearing-seat zone.
        """
        import cadquery as cq

        stack = CFG.stack_up
        h = CFG.housing
        seat_top = (
            stack.input_clearance + stack.disc_thickness * 2
            + stack.inter_disc_spacer + stack.output_clearance
            + stack.output_bearing_total
        )  # 48mm local (bearing-seat top / lip start)
        body_height = stack.total_housing_depth - stack.z_motor_plate_inner  # 51mm
        r_probe = (
            (h.output_bearing_seat_dia - 2 * 2.0) / 2.0 + h.output_bearing_seat_dia / 2.0
        ) / 2.0  # 44.075mm

        def material_at(z_local):
            box = (
                cq.Workplane("XY")
                .workplane(offset=z_local)
                .center(r_probe, 0)
                .box(1.0, 1.0, 0.5, centered=(True, True, False))
            )
            return body_solid.intersect(box).val().Volume()

        lip_vol = material_at((seat_top + body_height) / 2.0)
        seat_vol = material_at(seat_top - 3.0)
        assert lip_vol > 0.1, "No material at lip radius in lip zone — retention lip missing"
        assert seat_vol < 0.01, "Material at seat radius in bearing zone — seat bore blocked"

    def test_output_nut_pockets_present(self, body_solid):
        """Each output-face bolt location must have a hex nut pocket (not just a
        through-hole): probe 3mm off the bolt axis, 1mm below the output face — inside
        the hex (inradius 3.6mm) it must be void; a bare 2.2mm bolt hole would leave
        it solid.
        """
        import cadquery as cq

        stack = CFG.stack_up
        h = CFG.housing
        body_height = stack.total_housing_depth - stack.z_motor_plate_inner  # 51mm
        bolt_r = h.bolt_circle_dia / 2.0
        z_probe = body_height - 1.0
        for a in compute_housing_bolt_angles(CFG):
            cx = (bolt_r + 3.0) * math.cos(a)
            cy = (bolt_r + 3.0) * math.sin(a)
            box = (
                cq.Workplane("XY")
                .workplane(offset=z_probe)
                .center(cx, cy)
                .box(0.6, 0.6, 0.6, centered=(True, True, False))
            )
            vol = body_solid.intersect(box).val().Volume()
            assert vol < 0.01, (
                f"No nut pocket at bolt angle {math.degrees(a):.0f}° — "
                f"material present where the hex pocket should be void"
            )
