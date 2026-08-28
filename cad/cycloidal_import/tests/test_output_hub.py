"""Validation test suite for the output hub geometry.

Tests cover:
  1. Dimensional checks — clearances, fit relationships, hole spacing (no CadQuery)
  2. CadQuery solid — valid topology, bounding box, volume
"""

import math
import sys
import os

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.params import DEFAULT_CONFIG

CFG = DEFAULT_CONFIG


# ===================================================================
# 1. Dimensional checks (no CadQuery needed)
# ===================================================================


class TestOutputHubDimensions:

    def test_hub_od_near_bearing_bore(self):
        """As-designed hub OD must be within ±0.5mm of the 6814 bearing bore.

        As-designed OD is the 70mm bearing bore + 0.2mm interference for press
        grip on the inner race.
        """
        hub = CFG.output_hub
        b = CFG.bearings
        delta = abs(hub.od - b.out_bore)
        assert delta <= 0.5, (
            f"Hub OD {hub.od}mm differs from bearing bore {b.out_bore}mm "
            f"by {delta:.3f}mm (>0.5mm)"
        )

    def test_hub_od_is_press_fit(self):
        """As-designed hub OD must be close to the bearing bore (within 0.5mm)."""
        hub = CFG.output_hub
        b = CFG.bearings
        gap = abs(b.out_bore - hub.od)
        assert gap < 0.5, (
            f"Hub-to-bearing gap {gap:.3f}mm too large for press fit"
        )

    def test_hub_height_matches_bearing_stack(self):
        """Hub height must equal output bearing total (20mm)."""
        stack = CFG.stack_up
        assert stack.output_bearing_total == CFG.bearings.out_width * CFG.bearings.out_qty, (
            f"Output bearing total {stack.output_bearing_total}mm != "
            f"{CFG.bearings.out_width}mm × {CFG.bearings.out_qty}"
        )

    def test_proud_extension_is_5mm(self):
        """Spec §5.3: the hub output face sits 5mm proud of the housing (z=65).

        Guards the design value so a param drift shows up here rather than only
        in the (parametric) geometry tests below.
        """
        stack = CFG.stack_up
        hub = CFG.output_hub
        assert hub.proud_above_housing == pytest.approx(5.0), (
            f"proud_above_housing = {hub.proud_above_housing}mm, spec says 5.0mm"
        )
        hub_top_z = (
            stack.z_output_bearings
            + stack.output_bearing_total
            + stack.output_wall
            + hub.proud_above_housing
        )
        assert hub_top_z == pytest.approx(65.0), (
            f"Hub top at z={hub_top_z}mm, spec says z=65mm "
            f"(housing face {stack.total_housing_depth}mm + 5mm)"
        )

    def test_shaft_bore_clears_spine(self):
        """Shaft clearance bore must be larger than eccentric shaft spine."""
        hub = CFG.output_hub
        s = CFG.shaft
        clearance = hub.shaft_clearance_bore - s.spine_od
        assert clearance > 0, (
            f"Shaft bore {hub.shaft_clearance_bore}mm <= spine OD {s.spine_od}mm"
        )
        assert clearance >= 0.2, (
            f"Shaft clearance {clearance}mm too tight (need >= 0.2mm)"
        )

    def test_625_pocket_clears_shaft_bore(self):
        """625 bearing pocket must be larger than shaft clearance bore."""
        hub = CFG.output_hub
        b = CFG.bearings
        tol = CFG.tolerances
        pocket_dia = b.inp_od + tol.bearing_seat_bore_add  # 16.2mm
        assert pocket_dia > hub.shaft_clearance_bore, (
            f"625 pocket {pocket_dia}mm <= shaft bore {hub.shaft_clearance_bore}mm"
        )

    def test_625_pocket_inside_hub(self):
        """625 bearing pocket must fit well within the hub OD."""
        hub = CFG.output_hub
        b = CFG.bearings
        tol = CFG.tolerances
        pocket_r = (b.inp_od + tol.bearing_seat_bore_add) / 2.0
        hub_r = hub.od / 2.0
        wall = hub_r - pocket_r
        assert wall >= 5.0, (
            f"Wall from 625 pocket to hub OD = {wall:.2f}mm, need >= 5mm"
        )

    def test_output_pin_holes_inside_hub(self):
        """Output pin hole outer edges must not breach the hub OD."""
        d = CFG.disc
        hub = CFG.output_hub
        pin_outer = d.output_pin_circle_dia / 2.0 + d.output_pin_dia / 2.0
        hub_r = hub.od / 2.0
        assert pin_outer < hub_r, (
            f"Output pin edge at {pin_outer}mm >= hub radius {hub_r:.3f}mm"
        )

    def test_output_pin_holes_clear_625_pocket(self):
        """Output pin holes must not overlap the 625 bearing pocket."""
        d = CFG.disc
        b = CFG.bearings
        tol = CFG.tolerances
        pin_inner = d.output_pin_circle_dia / 2.0 - d.output_pin_dia / 2.0
        pocket_r = (b.inp_od + tol.bearing_seat_bore_add) / 2.0
        wall = pin_inner - pocket_r
        assert wall >= 2.0, (
            f"Pin hole to 625 pocket wall = {wall:.2f}mm, need >= 2mm"
        )

    def test_output_pin_holes_clear_bearing_bore(self):
        """Output pin holes must sit inside the 6814 bearing bore.

        The pins pass through the hub, which sits inside the bearing.
        Pin outer edges must not extend past the bearing bore.
        """
        d = CFG.disc
        b = CFG.bearings
        pin_outer = d.output_pin_circle_dia / 2.0 + d.output_pin_dia / 2.0
        bearing_bore_r = b.out_bore / 2.0
        assert pin_outer < bearing_bore_r, (
            f"Pin edge at {pin_outer}mm >= bearing bore radius {bearing_bore_r}mm"
        )

    def test_output_pin_hole_is_clearance(self):
        """Hub pin hole must be larger than the dowel pin (clearance fit, ring-pin convention)."""
        d = CFG.disc
        tol = CFG.tolerances
        hole_dia = d.output_pin_dia - tol.ring_pin_press_sub  # ring_pin_press_sub is -0.20
        assert hole_dia > d.output_pin_dia, (
            f"Hole {hole_dia}mm not larger than pin {d.output_pin_dia}mm"
        )
        clearance = hole_dia - d.output_pin_dia
        assert 0.10 <= clearance <= 0.30, (
            f"Clearance {clearance:.3f}mm outside expected PETG range (0.10–0.30mm)"
        )


# ===================================================================
# 2. CadQuery solid validation
# ===================================================================


class TestCadQuerySolid:

    @pytest.fixture(scope="class")
    def hub_solid(self):
        cq = pytest.importorskip("cadquery")
        from src.output_hub import build_output_hub

        return build_output_hub()

    def test_solid_is_valid(self, hub_solid):
        """The built solid should be non-null and have exactly one solid."""
        solids = hub_solid.solids().vals()
        assert len(solids) == 1, f"Expected 1 solid, got {len(solids)}"
        assert solids[0].isValid(), "Solid is not valid"

    def test_outer_diameter(self, hub_solid):
        """XY extent should match hub OD (70.3mm)."""
        bb = hub_solid.val().BoundingBox()
        x_size = bb.xmax - bb.xmin
        y_size = bb.ymax - bb.ymin
        hub = CFG.output_hub
        expected = hub.od  # 70.3mm

        assert abs(x_size - expected) < 0.2, (
            f"X extent {x_size:.2f}mm, expected {expected}mm"
        )
        assert abs(y_size - expected) < 0.2, (
            f"Y extent {y_size:.2f}mm, expected {expected}mm"
        )

    def test_height(self, hub_solid):
        """Z height = bearing grip + output wall + proud extension (28mm).

        The hub rises past the housing so the arm-mount face is proud of the chassis.
        """
        bb = hub_solid.val().BoundingBox()
        z_size = bb.zmax - bb.zmin
        stack = CFG.stack_up
        hub = CFG.output_hub
        expected = stack.output_bearing_total + stack.output_wall + hub.proud_above_housing  # 28mm

        assert abs(z_size - expected) < 0.1, (
            f"Z extent {z_size:.2f}mm, expected {expected:.2f}mm"
        )

    def test_output_face_proud_of_housing(self, hub_solid):
        """Top (output) face must sit proud of the housing output face.

        Hub bottom is at global z_output_bearings; its top must clear
        total_housing_depth (housing output face) by proud_above_housing.
        """
        bb = hub_solid.val().BoundingBox()
        stack = CFG.stack_up
        hub = CFG.output_hub
        # Solid is built at local Z; translated to z_output_bearings in the assembly.
        global_top = stack.z_output_bearings + bb.zmax
        expected_top = stack.total_housing_depth + hub.proud_above_housing  # 60 + 5 = 65

        assert abs(global_top - expected_top) < 0.1, (
            f"Output face at z={global_top:.2f}mm, expected {expected_top:.2f}mm "
            f"({hub.proud_above_housing}mm proud of housing at z={stack.total_housing_depth}mm)"
        )

    def test_arm_mount_holes_through(self, hub_solid):
        """The 4 arm-mount holes must pass fully through the hub (both faces open)."""
        from cadquery.occ_impl.geom import Vector
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.TopAbs import TopAbs_OUT

        hub = CFG.output_hub
        bb = hub_solid.val().BoundingBox()
        arm_r = hub.arm_mount_bolt_circle_dia / 2.0  # 25mm
        off = math.radians(hub.arm_mount_angle_offset_deg)
        solid = hub_solid.val()
        classifier = BRepClass3d_SolidClassifier(solid.wrapped)

        for i in range(hub.arm_mount_bolt_count):
            a = off + 2 * math.pi * i / hub.arm_mount_bolt_count
            x, y = arm_r * math.cos(a), arm_r * math.sin(a)
            # Probe the hole axis just inside both faces — both must be empty (OUT).
            for z in (bb.zmin + 0.5, bb.zmax - 0.5):
                classifier.Perform(Vector(x, y, z).toPnt(), 1e-3)
                assert classifier.State() == TopAbs_OUT, (
                    f"Arm-mount hole at angle {math.degrees(a):.0f}°, z={z:.1f}mm "
                    "not open — through-hole missing"
                )

    def test_arm_mount_nut_pockets_present(self, hub_solid):
        """Captive hex-nut pockets must be cut into the inner face only.

        Probe a point offset from the pocket axis by more than the through-hole
        radius but less than the hex apothem (inscribed circle), so it lands inside
        the pocket for any hex orientation but outside the 4.4mm bolt hole.  It must
        be empty near the inner face (in the pocket) yet solid up in the proud
        section (above the pocket) — confirming a localized counterbore.
        """
        from cadquery.occ_impl.geom import Vector
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.TopAbs import TopAbs_OUT, TopAbs_IN, TopAbs_ON

        hub = CFG.output_hub
        h = CFG.housing
        bb = hub_solid.val().BoundingBox()
        arm_r = hub.arm_mount_bolt_circle_dia / 2.0
        off = math.radians(hub.arm_mount_angle_offset_deg)
        apothem = h.bolt_nut_pocket_af / 2.0  # 3.6mm — hex inscribed radius
        hole_r = (h.bolt_dia + 0.4) / 2.0  # 2.2mm through-hole radius
        offset = (apothem + hole_r) / 2.0  # ~2.9mm: inside hex (any rotation), outside hole

        solid = hub_solid.val()
        classifier = BRepClass3d_SolidClassifier(solid.wrapped)
        a = off  # first pocket
        probe_r = arm_r + offset
        x, y = probe_r * math.cos(a), probe_r * math.sin(a)

        # Inside the pocket near the inner face → empty.
        classifier.Perform(Vector(x, y, h.bolt_nut_depth / 2.0).toPnt(), 1e-3)
        assert classifier.State() == TopAbs_OUT, (
            "No hex nut pocket near the inner face at the arm-mount bolt circle"
        )
        # Same XY up in the proud section (above the pocket) → solid.
        classifier.Perform(Vector(x, y, bb.zmax - 1.0).toPnt(), 1e-3)
        assert classifier.State() in (TopAbs_IN, TopAbs_ON), (
            "Nut pocket extends too far — proud section should be solid here"
        )

    def test_pin_holes_blind_from_output_face(self, hub_solid):
        """Pin holes must be blind from the output side — a 1mm ceiling stays solid.

        Sample a point on the 60mm pin circle at Z near the top of the hub.
        That point should be inside the hub solid (not inside a pin hole).
        """
        hub = CFG.output_hub
        d = CFG.disc
        height = CFG.stack_up.output_bearing_total
        pin_circle_r = d.output_pin_circle_dia / 2.0  # 30mm
        # Probe point: directly above pin #1 (angle 0), at Z=hub_height-0.25mm
        # (mid-ceiling). Inside the ceiling => inside the solid.
        probe_z = height - hub.output_hub_pin_ceiling / 2.0
        probe_xy = (pin_circle_r, 0.0)
        solid = hub_solid.val()

        from cadquery.occ_impl.geom import Vector
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.TopAbs import TopAbs_IN, TopAbs_ON

        classifier = BRepClass3d_SolidClassifier(solid.wrapped)
        classifier.Perform(Vector(probe_xy[0], probe_xy[1], probe_z).toPnt(), 1e-3)
        state = classifier.State()
        assert state in (TopAbs_IN, TopAbs_ON), (
            f"Probe point at pin #1 / Z={probe_z}mm not inside hub — "
            "blind ceiling above output pin holes is missing"
        )

    def test_central_lightening_pocket(self, hub_solid):
        """The proud arm-mount face carries a central lightening recess: empty
        at the centre near the top, with a solid floor above the bearing-grip
        zone (and a sound wall out to the arm-bolt circle)."""
        from cadquery.occ_impl.geom import Vector
        from OCP.BRepClass3d import BRepClass3d_SolidClassifier
        from OCP.TopAbs import TopAbs_OUT, TopAbs_IN, TopAbs_ON

        hub = CFG.output_hub
        stack = CFG.stack_up
        if hub.arm_mount_pocket_dia <= 0:
            pytest.skip("lightening pocket disabled")

        bb = hub_solid.val().BoundingBox()
        clf = BRepClass3d_SolidClassifier(hub_solid.val().wrapped)

        # Centre, just below the top (output) face → inside the pocket (empty).
        clf.Perform(Vector(0, 0, bb.zmax - 0.5).toPnt(), 1e-4)
        assert clf.State() == TopAbs_OUT, "central lightening pocket missing"

        # Just below the pocket floor (grip-zone top + floor) → solid slab.
        z_floor = stack.output_bearing_total + hub.arm_mount_pocket_floor
        clf.Perform(Vector(0, 0, z_floor - 0.3).toPnt(), 1e-4)
        assert clf.State() in (TopAbs_IN, TopAbs_ON), "pocket floor missing"

        # Wall to the arm-bolt circle stays solid near the top (pocket clears it).
        arm_r = hub.arm_mount_bolt_circle_dia / 2.0
        clf.Perform(Vector(arm_r, 0, bb.zmax - 0.5).toPnt(), 1e-4)
        assert clf.State() in (TopAbs_IN, TopAbs_ON), (
            "pocket should not reach the arm-bolt circle"
        )

    def test_volume_sanity(self, hub_solid):
        """Volume should be between reasonable bounds.

        Lower bound: solid cylinder minus all holes and pockets.
        Upper bound: solid cylinder minus only shaft bore.
        """
        hub = CFG.output_hub
        d = CFG.disc
        b = CFG.bearings
        tol = CFG.tolerances
        stack = CFG.stack_up

        h = CFG.housing
        hub_r = hub.od / 2.0
        bearing_grip = stack.output_bearing_total  # 20mm — shaft bore depth
        height = bearing_grip + stack.output_wall + hub.proud_above_housing  # 28mm total
        shaft_r = hub.shaft_clearance_bore / 2.0

        # Upper: full cylinder minus the shaft bore (only as deep as the grip zone)
        upper = math.pi * hub_r ** 2 * height - math.pi * shaft_r ** 2 * bearing_grip

        # Lower: subtract all features generously
        pocket_r = (b.inp_od + tol.bearing_seat_bore_add) / 2.0
        pocket_vol = math.pi * pocket_r ** 2 * b.inp_width
        pin_hole_dia = d.output_pin_dia - tol.ring_pin_press_sub  # 4.20mm clearance
        pin_hole_depth = bearing_grip - hub.output_hub_pin_ceiling  # 19mm blind
        pin_vol = d.output_pin_count * math.pi * (pin_hole_dia / 2.0) ** 2 * pin_hole_depth
        # 4× M4 arm-mount through-holes (full height) + captive hex-nut pockets
        arm_hole_r = (h.bolt_dia + 0.4) / 2.0  # 2.2mm
        arm_hole_vol = hub.arm_mount_bolt_count * math.pi * arm_hole_r ** 2 * height
        nut_pocket_vol = (
            hub.arm_mount_bolt_count
            * math.pi * (h.bolt_nut_pocket_af / 2.0) ** 2  # over-estimate pocket as a disc
            * h.bolt_nut_depth
        )
        # Central lightening recess in the proud arm-mount face
        lightening_vol = (
            math.pi * (hub.arm_mount_pocket_dia / 2.0) ** 2
            * (height - bearing_grip - hub.arm_mount_pocket_floor)
            if hub.arm_mount_pocket_dia > 0
            else 0.0
        )
        lower = (
            math.pi * hub_r ** 2 * height
            - math.pi * shaft_r ** 2 * bearing_grip
            - pocket_vol
            - pin_vol
            - arm_hole_vol
            - nut_pocket_vol
            - lightening_vol
        ) * 0.9  # 10% margin for overlap

        vol = hub_solid.val().Volume()
        assert vol > lower, f"Volume {vol:.0f}mm³ below lower bound {lower:.0f}"
        assert vol < upper, f"Volume {vol:.0f}mm³ above upper bound {upper:.0f}"
