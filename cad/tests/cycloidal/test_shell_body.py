"""The turning shell's body round the discs (lib/cycloidal/housing.py build_shell_body, printed as one with j1_link) and
the shell's stack (grown from the port's ring gear body tests, cycloidal_drive tests/test_ring_gear_body.py; the port's
body itself is matched by tests/cycloidal/test_port.py).

  1. Dimensional checks - the stack symmetric about the middle of the discs, clearances, fits (config only)
  2. The built solid - topology, bounding box, section areas, lip / seat / pin ring / nut-pocket probes, volume
"""
import math

import pytest

from lib import reference as R
from lib.cycloidal import (
    arm_zone,
    compute_housing_bolt_angles,
    housing_bolt_points,
    hub_flange,
    ring_pin_engagement,
    ring_pin_hole_dia,
    ring_pin_points,
    shell_ends,
    sleeve_end,
    stack_positions,
)
from lib.cycloidal.housing import build_shell_body
from lib.cycloidal.layout import PILLAR_OVERSHOOT
from lib.cycloidal.profiles import compute_epitrochoid, compute_profile_radii
from tests.cycloidal.helpers import CFG
from tests.helpers import is_inside, probe_volume, section_area

S = stack_positions(CFG)
(END_0, END_1), (Z0, FLANGE) = shell_ends(CFG), arm_zone(CFG)   # -13, 61; 9, 39
SEAT = CFG.stack_up.z_output_bearings                             # 48
MID = (S["z_disc1"] + S["z_disc2"] + CFG.disc.thickness) / 2.0    # 24: the middle of the discs


class TestShellDimensions:

    def test_the_shell_is_symmetric_about_the_middle_of_the_discs(self):
        """From the discs out, each end: the stack's clearance, a plate (the motor plate / the hub's flange), a 6814,
        the lip - so the ends, the plates' faces, the 6814s and the ring pins mirror about the middle of the discs."""
        s, b, sh = CFG.stack_up, CFG.bearings, CFG.shell
        assert s.output_clearance == s.input_clearance + hub_flange(CFG) and hub_flange(CFG) == s.z_motor_plate_inner
        assert s.output_bearing_total == b.out_width and s.output_wall == sh.end_lip
        for a, b_ in ((END_0, END_1), (Z0, FLANGE), (S["z_6814_2"], SEAT + b.out_width), (0.0, SEAT),
                      (S["z_ring_pins"], S["z_ring_pins"] + CFG.gear.ring_pin_length)):
            assert a + b_ == pytest.approx(2 * MID), (a, b_)
        assert (END_0, Z0, FLANGE, SEAT, END_1) == (-13.0, 9.0, 39.0, 48.0, 61.0)
        assert SEAT + b.out_width / 2.0 - (S["z_6814_2"] + b.out_width / 2.0) == 58.0      # the 6814s' centres

    def test_the_ring_pins_reach_both_pin_rings(self):
        """5 into the pin ring at each end past the plates' gap (the port's 3.5 into its plate and its wall)."""
        assert ring_pin_engagement(CFG) == pytest.approx(5.0)
        assert S["z_ring_pins"] == pytest.approx(Z0 - 5.0)

    def test_the_yoke_legs_and_the_hub_and_sleeve_ends(self):
        """The hub's face end_plate_gap past the hub end (on the yoke's leg); the sleeve through the motor-side leg."""
        sh = CFG.shell
        assert S["hub_top"] == pytest.approx(END_1 + sh.end_plate_gap)
        assert sleeve_end(CFG) == pytest.approx(END_0 - sh.end_plate_gap - sh.yoke_leg)
        assert S["hub_top"] - MID == pytest.approx(MID - (END_0 - sh.end_plate_gap))       # the legs' inner faces mirror

    def test_housing_bolts_inside_od(self):
        h, tol = CFG.housing, CFG.tolerances
        assert h.bolt_circle_dia / 2.0 + (h.bolt_dia + tol.bolt_clearance_add) / 2.0 < h.od / 2.0

    def test_pin_holes_dont_breach_the_seats(self):
        """The ring-pin holes keep >= 2.5 mm to the 6814 seat bore (lib/cycloidal/params.py RING_INSET: 2.8 mm)."""
        g, h = CFG.gear, CFG.housing
        gap = (g.ring_pin_circle_dia / 2.0 - ring_pin_hole_dia(CFG) / 2.0) - h.output_bearing_seat_dia / 2.0
        assert gap >= 2.5, f"Pin hole inner edge to bearing seat wall = {gap:.2f}mm, need >= 2.5mm"

    def test_pin_holes_in_the_pin_rings(self):
        """The pins stand in the pin rings' wall: past ring_bore_dia, inside the od."""
        g, h, sh = CFG.gear, CFG.housing, CFG.shell
        assert g.ring_pin_circle_dia / 2.0 - ring_pin_hole_dia(CFG) / 2.0 - sh.ring_bore_dia / 2.0 == pytest.approx(1.9)
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

    def test_the_plates_clear_the_pin_rings(self):
        """The held plates (the motor plate, the hub's flange) inside the turning pin rings, 1 a side."""
        sh = CFG.shell
        assert (sh.ring_bore_dia - sh.plate_dia) / 2.0 == pytest.approx(1.0)
        assert sh.plate_dia == CFG.bearings.out_od                 # the 6814's outer race's diameter: it can't pass the plate

    def test_integral_lips_block_the_6814s(self):
        assert CFG.housing.lip_bore_dia < CFG.bearings.out_od

    def test_integral_lip_clears_hub_and_sleeve(self):
        clearance = CFG.housing.lip_bore_dia - max(CFG.output_hub.od, CFG.shell.sleeve_od)
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

    def test_pillars_dont_overlap_neighbors(self):
        """Adjacent pillars leave >= 1 mm of open window at the bore and at the OD."""
        h = CFG.housing
        pillar_inner_r, pillar_outer_r = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + PILLAR_OVERSHOOT
        angles = compute_housing_bolt_angles(CFG)
        spacing = angles[1] - angles[0]
        assert 2 * pillar_inner_r * math.sin(spacing / 2) > h.pillar_inner_w + 1.0
        assert 2 * pillar_outer_r * math.sin(spacing / 2) > h.pillar_outer_w + 1.0

    def test_disc_fits_through_main_bore(self):
        g, h = CFG.gear, CFG.housing
        pts = compute_epitrochoid(R=g.ring_pin_circle_radius, r=g.ring_pin_radius, N=g.num_ring_pins, e=g.eccentricity)
        max_r = max(math.hypot(x, y) for x, y in pts)
        assert max_r + g.eccentricity < h.bore_dia / 2.0

    def test_housing_bolts_end_in_their_nuts(self):
        """The M4 x 65s run end to end: their heads' tops 0.5 inside the shell ring's end, their ends flush with the
        nuts' outer faces, the nuts sunk in the body's hub end."""
        h = CFG.housing
        assert S["z_housing_bolts"] == pytest.approx(END_0 + h.bolt_counterbore_depth - h.bolt_head_height)
        end = S["z_housing_bolts"] + h.bolt_head_height + h.bolt_length
        assert end == pytest.approx(S["z_housing_nuts"] + h.bolt_nut_thickness) and end < END_1


@pytest.fixture(scope="module")
def body_solid():
    return build_shell_body()


@pytest.mark.slow
class TestSolid:

    def test_numbers(self, body_solid):
        """One valid solid; the od on X (the pillars at 0 / 180 degrees), on Y the chamfered corners of the pillars at
        +/-60 and +/-120 degrees; the plate's face (the shell ring's) to the hub end."""
        assert len(body_solid.solids()) == 1 and body_solid.is_valid
        assert R.solid_volume(body_solid) == pytest.approx(99536.499, abs=0.5)
        bb = body_solid.bounding_box()
        assert (bb.size.X, bb.size.Y) == pytest.approx((CFG.housing.od, 116.023), abs=0.01)
        assert (bb.min.Z, bb.max.Z) == pytest.approx((Z0, END_1), abs=1e-6)

    def test_window_zone_has_pillars_only(self, body_solid):
        """Round the discs only the bolt pillars remain: < 40 % of the full annulus."""
        h = CFG.housing
        area = section_area(body_solid, S["z_disc2"] - 1.0)
        full_annulus = math.pi * ((h.od / 2.0) ** 2 - (h.bore_dia / 2.0) ** 2)
        assert 0 < area < full_annulus * 0.40, f"Window zone area {area:.1f}mm^2 too large"

    def test_windows_end_to_end(self, body_solid):
        """Midway between the pillars, past the bore's radius: void round the discs, the pin ring, the seat, the lip."""
        h = CFG.housing
        step, r = 2 * math.pi / h.bolt_count, (h.bore_dia + h.od) / 4.0
        for a in compute_housing_bolt_angles(CFG):
            for z in (Z0 + 1.0, 30.0, FLANGE + 4.0, SEAT + 5.0, END_1 - 1.5):
                assert not is_inside(body_solid, r * math.cos(a + step / 2.0), r * math.sin(a + step / 2.0), z), (math.degrees(a), z)

    def test_bore(self, body_solid):
        """From the discs out: the bore, the pin ring round the hub's flange, the seat, the lip - each open inside its
        bore, solid just outside it."""
        h, sh = CFG.housing, CFG.shell
        for r, z in ((sh.ring_bore_dia / 2.0, FLANGE + 4.5), (h.output_bearing_seat_dia / 2.0, SEAT + 5.0),
                     (h.lip_bore_dia / 2.0, END_1 - sh.end_lip / 2.0)):
            assert not is_inside(body_solid, r - 0.1, 0, z) and is_inside(body_solid, r + 0.3, 0, z), (r, z)
        assert not is_inside(body_solid, h.bore_dia / 2.0 - 0.1, 0, 30.0)

    def test_ring_pin_holes(self, body_solid):
        """Blind from the pin ring's face on the bore, past the pins' ends; solid beyond."""
        top = S["z_ring_pins"] + CFG.gear.ring_pin_length          # 44
        for x, y in ring_pin_points(CFG):
            assert not is_inside(body_solid, x, y, FLANGE + 0.5) and not is_inside(body_solid, x, y, top + 0.1)
            assert is_inside(body_solid, x, y, top + 1.0)

    def test_nut_pockets(self, body_solid):
        """The housing nuts' hex pockets from the hub end down to the nuts' seats (z_housing_nuts): void 3 mm off each
        bolt's axis over the nut, solid under its seat."""
        bolt_r = CFG.housing.bolt_circle_dia / 2.0
        for a in compute_housing_bolt_angles(CFG):
            xy = ((bolt_r + 3.0) * math.cos(a), (bolt_r + 3.0) * math.sin(a))
            assert probe_volume(body_solid, xy, S["z_housing_nuts"] + 1.0, size=0.6, height=0.6) < 0.01
            assert probe_volume(body_solid, xy, S["z_housing_nuts"] - 1.0, size=0.6, height=0.6) > 0.1
        for x, y in housing_bolt_points(CFG):
            assert not is_inside(body_solid, x, y, Z0 + 1.0)      # the bolt's hole through
