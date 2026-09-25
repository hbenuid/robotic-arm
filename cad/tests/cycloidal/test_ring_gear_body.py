"""Ring gear body (ported from cycloidal_drive tests/test_ring_gear_body.py).

  1. Dimensional checks - zone heights, clearances, fit relationships (config only)
  2. The built solid - topology, bounding box, section areas, lip / nut-pocket probes, volume
"""
import math

import pytest

import parts
from lib import reference as R
from lib.cycloidal import compute_housing_bolt_angles, ring_pin_engagement, ring_pin_hole_dia
from lib.cycloidal.housing import PILLAR_OVERSHOOT
from lib.cycloidal.profiles import compute_epitrochoid, compute_profile_radii
from tests.cycloidal.helpers import CFG
from tests.helpers import probe_volume, section_area

cycloidal_ring_gear_body = parts.load("cycloidal_ring_gear_body")
BODY_H = CFG.stack_up.ring_gear_body_height      # 51


class TestRingGearBodyDimensions:

    def test_body_height_matches_stackup(self):
        stack = CFG.stack_up
        actual = (stack.input_clearance + stack.disc_thickness * 2 + stack.inter_disc_spacer
                  + stack.output_clearance + stack.output_bearing_total + stack.output_wall)
        assert abs(actual - BODY_H) < 0.01

    def test_housing_bolts_inside_od(self):
        h, tol = CFG.housing, CFG.tolerances
        assert h.bolt_circle_dia / 2.0 + (h.bolt_dia + tol.bolt_clearance_add) / 2.0 < h.od / 2.0

    def test_pin_holes_dont_breach_bearing_seat(self):
        """The ring-pin holes keep >= 5 mm to the bearing seat bore."""
        g, h = CFG.gear, CFG.housing
        gap = (g.ring_pin_circle_dia / 2.0 - ring_pin_hole_dia(CFG) / 2.0) - h.output_bearing_seat_dia / 2.0
        assert gap >= 5.0, f"Pin hole inner edge to bearing seat wall = {gap:.2f}mm, need >= 5mm"

    def test_pin_holes_inside_housing_od(self):
        g, h = CFG.gear, CFG.housing
        assert g.ring_pin_circle_dia / 2.0 + ring_pin_hole_dia(CFG) / 2.0 < h.od / 2.0

    def test_bolt_holes_do_not_overlap_pin_holes(self):
        g, h, tol = CFG.gear, CFG.housing, CFG.tolerances
        pin_circle_r, bolt_circle_r = g.ring_pin_circle_dia / 2.0, h.bolt_circle_dia / 2.0
        min_distance = ring_pin_hole_dia(CFG) / 2.0 + (h.bolt_dia + tol.bolt_clearance_add) / 2.0
        pin_angles = [2 * math.pi * i / g.num_ring_pins for i in range(g.num_ring_pins)]
        for bi, ba in enumerate(compute_housing_bolt_angles(CFG)):
            bx, by = bolt_circle_r * math.cos(ba), bolt_circle_r * math.sin(ba)
            for pi_, pa in enumerate(pin_angles):
                dist = math.hypot(bx - pin_circle_r * math.cos(pa), by - pin_circle_r * math.sin(pa))
                assert dist >= min_distance, f"Bolt {bi} overlaps pin {pi_}: distance {dist:.2f}mm < {min_distance:.2f}mm"

    def test_bolt_holes_do_not_overlap_each_other(self):
        h, tol = CFG.housing, CFG.tolerances
        r, min_distance = h.bolt_circle_dia / 2.0, h.bolt_dia + tol.bolt_clearance_add
        angles = compute_housing_bolt_angles(CFG)
        for i in range(len(angles)):
            for j in range(i + 1, len(angles)):
                dist = math.hypot(r * math.cos(angles[i]) - r * math.cos(angles[j]), r * math.sin(angles[i]) - r * math.sin(angles[j]))
                assert dist >= min_distance

    def test_bearing_seat_smaller_than_main_bore(self):
        assert CFG.housing.output_bearing_seat_dia < CFG.housing.bore_dia

    def test_bearing_seat_is_press_fit(self):
        gap = CFG.housing.output_bearing_seat_dia - CFG.bearings.out_od
        assert 0 < gap < 0.5

    def test_integral_lip_blocks_6814(self):
        assert CFG.housing.lip_bore_dia < CFG.bearings.out_od

    def test_integral_lip_clears_hub(self):
        clearance = CFG.housing.lip_bore_dia - CFG.output_hub.od
        assert clearance >= 0.2, f"Lip bore vs hub OD clearance {clearance:.2f}mm < 0.2mm"

    def test_lip_retention_shoulder_width(self):
        h = CFG.housing
        assert (h.output_bearing_seat_dia - h.lip_bore_dia) / 2.0 >= 2.0 - 1e-6

    def test_housing_bolts_outside_bore(self):
        """Bolts inside the bore would be unsupported and hit the orbiting discs."""
        h, tol = CFG.housing, CFG.tolerances
        assert h.bolt_circle_dia / 2.0 - (h.bolt_dia + tol.bolt_clearance_add) / 2.0 > h.bore_dia / 2.0

    def test_housing_bolts_have_wall_to_bore(self):
        h, tol = CFG.housing, CFG.tolerances
        wall = (h.bolt_circle_dia / 2.0 - (h.bolt_dia + tol.bolt_clearance_add) / 2.0) - h.bore_dia / 2.0
        assert wall >= 1.0

    def test_housing_bolts_clear_disc_sweep(self):
        g, h, tol = CFG.gear, CFG.housing, CFG.tolerances
        pts = compute_epitrochoid(R=g.ring_pin_circle_radius, r=g.ring_pin_radius, N=g.num_ring_pins, e=g.eccentricity)
        _, max_r = compute_profile_radii(pts)
        bolt_inner_edge = h.bolt_circle_dia / 2.0 - (h.bolt_dia + tol.bolt_clearance_add) / 2.0
        assert bolt_inner_edge - (max_r + g.eccentricity) > 0

    def test_pillar_wall_around_bolt(self):
        """Each pillar leaves >= 3 mm tangential wall around the M4 hole (width interpolated
        linearly from the bore to the OD)."""
        h, tol = CFG.housing, CFG.tolerances
        bolt_r = h.bolt_circle_dia / 2.0
        pillar_inner_r, pillar_outer_r = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + PILLAR_OVERSHOOT
        t = (bolt_r - pillar_inner_r) / (pillar_outer_r - pillar_inner_r)
        pillar_w_at_bolt = h.pillar_inner_w + (h.pillar_outer_w - h.pillar_inner_w) * t
        wall = pillar_w_at_bolt / 2.0 - (h.bolt_dia + tol.bolt_clearance_add) / 2.0
        assert wall >= 3.0, f"Tangential pillar wall around bolt = {wall:.2f}mm, need >= 3mm"

    def test_pillar_overshoots_housing_walls(self):
        h = CFG.housing
        assert h.bore_dia / 2.0 - PILLAR_OVERSHOOT < h.bore_dia / 2.0
        assert h.od / 2.0 + PILLAR_OVERSHOOT > h.od / 2.0

    def test_pillars_dont_overlap_neighbors(self):
        """Adjacent pillars leave >= 1 mm of open window at the bore and at the OD."""
        h = CFG.housing
        pillar_inner_r, pillar_outer_r = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + PILLAR_OVERSHOOT
        angles = compute_housing_bolt_angles(CFG)
        spacing = angles[1] - angles[0]
        assert 2 * pillar_inner_r * math.sin(spacing / 2) > h.pillar_inner_w + 1.0
        assert 2 * pillar_outer_r * math.sin(spacing / 2) > h.pillar_outer_w + 1.0

    def test_windows_span_full_body_height(self):
        """The reveal windows run the full body height (no continuous rim; the motor plate seats
        on the 8 pillar tops)."""
        assert BODY_H == CFG.stack_up.total_housing_depth - CFG.stack_up.z_motor_plate_inner

    def test_ring_pin_chamfer_within_bearing_zone(self):
        assert CFG.housing.ring_pin_entry_chamfer_depth <= ring_pin_engagement(CFG)

    def test_ring_pin_chamfer_no_overlap(self):
        g, h = CFG.gear, CFG.housing
        chamfer_dia = ring_pin_hole_dia(CFG) + h.ring_pin_entry_chamfer_add
        center_dist = 2 * g.ring_pin_circle_radius * math.sin(math.pi / g.num_ring_pins)
        assert center_dist > chamfer_dia

    def test_disc_fits_through_main_bore(self):
        g, h = CFG.gear, CFG.housing
        pts = compute_epitrochoid(R=g.ring_pin_circle_radius, r=g.ring_pin_radius, N=g.num_ring_pins, e=g.eccentricity)
        max_r = max(math.hypot(x, y) for x, y in pts)
        assert max_r + g.eccentricity < h.bore_dia / 2.0


@pytest.fixture(scope="module")
def body_solid():
    return cycloidal_ring_gear_body.build()


@pytest.mark.slow
class TestSolid:

    def test_solid_is_valid(self, body_solid):
        assert len(body_solid.solids()) == 1
        assert body_solid.is_valid

    def test_outer_diameter(self, body_solid):
        size = body_solid.bounding_box().size
        assert abs(size.X - CFG.housing.od) < 0.2
        assert abs(size.Y - CFG.housing.od) < 0.2

    def test_height(self, body_solid):
        assert abs(body_solid.bounding_box().size.Z - BODY_H) < 0.1

    def test_window_zone_has_pillars_only(self, body_solid):
        """In the window zone only the bolt pillars remain: < 40 % of the full annulus."""
        stack, h = CFG.stack_up, CFG.housing
        z = stack.input_clearance + stack.disc_thickness + stack.inter_disc_spacer / 2.0     # 15
        area = section_area(body_solid, z)
        full_annulus = math.pi * ((h.od / 2.0) ** 2 - (h.bore_dia / 2.0) ** 2)
        assert 0 < area < full_annulus * 0.40, f"Window zone area {area:.1f}mm^2 too large"

    def test_input_face_has_pillars_only(self, body_solid):
        """Just above the input face: pillars only, no continuous rim."""
        h = CFG.housing
        area = section_area(body_solid, 0.5)
        full_annulus = math.pi * ((h.od / 2.0) ** 2 - (h.bore_dia / 2.0) ** 2)
        assert 0 < area < full_annulus * 0.40, f"Input-face section area {area:.1f}mm^2 - rim reintroduced?"

    def test_output_bearing_seat(self, body_solid):
        """At the bearing-zone midpoint: the bore->seat annulus (minus pin holes) plus the 8
        pillars (minus bolt holes), within 15 %."""
        g, h, stack = CFG.gear, CFG.housing, CFG.stack_up
        z = stack.bore_zone + stack.output_bearing_total / 2.0       # 38
        area = section_area(body_solid, z)
        bore_r, seat_r, housing_r = h.bore_dia / 2.0, h.output_bearing_seat_dia / 2.0, h.od / 2.0
        inner_annulus = math.pi * (bore_r ** 2 - seat_r ** 2)
        pillar_area = (h.pillar_inner_w + h.pillar_outer_w) / 2.0 * (housing_r - bore_r) * h.bolt_count
        pin_area = g.num_ring_pins * math.pi * (ring_pin_hole_dia(CFG) / 2.0) ** 2
        bolt_area = h.bolt_count * math.pi * ((h.bolt_dia + CFG.tolerances.bolt_clearance_add) / 2.0) ** 2
        expected = inner_annulus + pillar_area - pin_area - bolt_area
        assert abs(area - expected) / expected < 0.15, f"Bearing zone section area {area:.1f}mm^2 vs expected {expected:.1f}mm^2"

    def test_volume_sanity(self, body_solid):
        """Between 70 % of the bearing-seat ring and the outer cylinder minus the seat bore."""
        h, stack = CFG.housing, CFG.stack_up
        bore_r, housing_r, seat_r = h.bore_dia / 2.0, h.od / 2.0, h.output_bearing_seat_dia / 2.0
        upper = math.pi * (housing_r ** 2 - seat_r ** 2) * BODY_H
        lower = math.pi * (bore_r ** 2 - seat_r ** 2) * stack.output_bearing_total * 0.7
        vol = R.solid_volume(body_solid)
        assert lower < vol < upper, f"Volume {vol:.0f}mm^3 outside ({lower:.0f}, {upper:.0f})"

    def test_integral_lip_present_in_solid(self, body_solid):
        """Probe a ring between the lip bore (r 43.075) and the seat (r 45.075): material in the lip
        zone, void in the bearing-seat zone."""
        h, stack = CFG.housing, CFG.stack_up
        seat_top = stack.bore_zone + stack.output_bearing_total          # 48
        r_probe = (h.lip_bore_dia / 2.0 + h.output_bearing_seat_dia / 2.0) / 2.0
        assert probe_volume(body_solid, (r_probe, 0.0), (seat_top + BODY_H) / 2.0) > 0.1, "retention lip missing"
        assert probe_volume(body_solid, (r_probe, 0.0), seat_top - 3.0) < 0.01, "seat bore blocked"

    def test_output_nut_pockets_present(self, body_solid):
        """3 mm off each bolt axis, 1 mm below the output face, inside the hex (inradius 3.6): void."""
        h = CFG.housing
        bolt_r = h.bolt_circle_dia / 2.0
        for a in compute_housing_bolt_angles(CFG):
            xy = ((bolt_r + 3.0) * math.cos(a), (bolt_r + 3.0) * math.sin(a))
            assert probe_volume(body_solid, xy, BODY_H - 1.0, size=0.6, height=0.6) < 0.01, f"No nut pocket at bolt angle {math.degrees(a):.0f} deg"
