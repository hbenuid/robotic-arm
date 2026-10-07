"""Assembly-level checks of the cycloidal drive (ported from cycloidal_drive
tests/test_assembly_clearances.py) plus the module's own locks and its attachment to the arm.

  1. Axial stack-up - Z positions consistent, no gaps or overlaps
  2. Housing alignment - the shell ring and the body (j1_link's) mate flush; the turning shell's layout (ShellParams):
     its two 6814s, the sleeve round the motor, the stops that hold the shell along the axis
  3. Radial clearances - discs clear the housing, pins clear bearings, hub clears the lip
  4. Bearing retention - 6814 by press fit + integral lip; 6003 by the disc bore; 625 by the hub pocket
  5. Shaft reach - motor shaft in the D-bore, support pin through the 625
  6. Ring pin span; 7. housing bolt engagement
  8. The module's interference budget (every mating pair: the designed overlaps and the clean ones), the module
     totals, colors and layout locks, and the drive's pose in the arm (fits its neighbours, the shell's body in
     j1_link, the hub and the sleeve on the yoke's legs)
"""
import math

import pytest
from build123d import Compound, GeomType, Location, Vector

from assemblies import cycloidal_drive
from lib import placements as P
from lib import reference as R
from lib.cycloidal import (
    compute_housing_bolt_angles,
    hex_circumdiameter,
    hub_flange,
    hub_height,
    ring_pin_engagement,
    shell_ends,
    sleeve_end,
    stack_positions,
)
from lib.cycloidal.profiles import compute_epitrochoid, compute_profile_radii
from robot import frames as F
from tests import built
from tests.cycloidal.helpers import CFG
from tests.helpers import interference, module_tints

DRIVE_KEY = "cycloidal_drive#1"


# ===================================================================
# 1. Axial stack-up consistency
# ===================================================================


class TestAxialStackUp:

    def test_total_housing_depth(self):
        s = CFG.stack_up
        expected = (s.motor_plate_wall + s.motor_plate_inner_wall + s.input_clearance + s.disc_thickness * 2
                    + s.inter_disc_spacer + s.output_clearance + s.output_bearing_total + s.output_wall)
        assert abs(s.total_housing_depth - expected) < 0.01

    def test_total_depth_is_61mm(self):
        """9 mm plate + the body: the gear's 30, the hub's 9 mm flange, its 6814, the 3 mm lip."""
        assert abs(CFG.stack_up.total_housing_depth - 61.0) < 0.01

    def test_hub_reaches_the_yoke(self):
        """The held hub's face lies ShellParams.end_plate_gap past the shell's hub end, on the j1_coupler fork's hub-side
        leg (tests/yaw_coupler/)."""
        S, s = stack_positions(CFG), CFG.stack_up
        assert math.isclose(S["z_hub"] + hub_height(CFG), s.total_housing_depth + CFG.shell.end_plate_gap)
        assert math.isclose(S["z_hub"], s.z_output_bearings - hub_flange(CFG))

    def test_output_hub_protrudes_past_chassis(self):
        s, hub = CFG.stack_up, CFG.output_hub
        hub_top_z = stack_positions(CFG)["hub_top"]
        assert abs(hub_top_z - (s.total_housing_depth + hub.proud_above_housing)) < 0.01
        assert hub_top_z > s.total_housing_depth

    def test_disc1_before_disc2(self):
        assert CFG.stack_up.z_disc1 < CFG.stack_up.z_disc2

    def test_disc2_ends_before_output_bearings(self):
        s = CFG.stack_up
        assert s.z_disc2 + s.disc_thickness <= s.z_output_bearings

    def test_output_clearance_gap(self):
        s = CFG.stack_up
        assert abs((s.z_output_bearings - (s.z_disc2 + s.disc_thickness)) - s.output_clearance) < 0.01

    def test_output_bearings_end_at_bearing_top(self):
        s = CFG.stack_up
        assert abs(s.z_output_bearings + s.output_bearing_total - s.z_bearing_top) < 0.01


# ===================================================================
# 2. Housing part alignment
# ===================================================================


class TestHousingAlignment:

    def test_motor_plate_inner_face(self):
        """Motor plate inner face at z=9 (5 + 4)."""
        s = CFG.stack_up
        assert abs(s.z_motor_plate_inner - (s.motor_plate_wall + s.motor_plate_inner_wall)) < 0.01

    def test_body_height(self):
        """The shell's body (j1_link's, lib/cycloidal/housing.py build_shell_body): the motor plate's inner face to the
        hub end."""
        s = CFG.stack_up
        assert abs(s.ring_gear_body_height - 52.0) < 0.01

    def test_all_housing_parts_same_od(self):
        assert CFG.housing.od == 129.2

    def test_housing_bolt_angles_consistent(self):
        angles = compute_housing_bolt_angles(CFG)
        assert len(angles) == CFG.housing.bolt_count
        assert all(angles[i] < angles[i + 1] for i in range(len(angles) - 1))
        assert angles[0] >= 0 and angles[-1] < 2 * math.pi

    def test_housing_bolts_clear_ring_pins(self):
        """Bolt (62.5) and pin (54) circles: centre distance > hole radii sum everywhere."""
        g, h, tol = CFG.gear, CFG.housing, CFG.tolerances
        bolt_r, pin_r = h.bolt_circle_dia / 2.0, g.ring_pin_circle_dia / 2.0
        min_center_dist = (h.bolt_dia + tol.bolt_clearance_add) / 2.0 + g.ring_pin_dia / 2.0
        for ba in compute_housing_bolt_angles(CFG):
            for i in range(g.num_ring_pins):
                pa = 2 * math.pi * i / g.num_ring_pins
                dist = math.hypot(bolt_r * math.cos(ba) - pin_r * math.cos(pa), bolt_r * math.sin(ba) - pin_r * math.sin(pa))
                assert dist > min_center_dist


class TestTurningShell:
    """The carrier (hub + output pins) and the motor held, the shell turning on one 6814 at each end (ShellParams)."""

    def test_ratio_is_the_ring_pins(self):
        assert CFG.ratio == CFG.gear.num_ring_pins == 21

    def test_the_6814s_are_58_apart(self):
        """The hub's past its flange, the other just behind the motor plate - centres 58 apart (the port stacked both in
        the seat: 10), mirror images about the middle of the discs."""
        S, w, s = stack_positions(CFG), CFG.bearings.out_width, CFG.stack_up
        assert S["z_6814_1"] == s.z_output_bearings and S["z_6814_1"] + w == s.z_bearing_top and S["z_6814_2"] + w == 0.0
        assert S["z_6814_1"] - S["z_6814_2"] == 58.0

    def test_sleeve_clears_the_motor_and_its_board(self):
        """The sleeve's bore round the motor (the vendor model 42 x 43, 0.5 off-centre) and the MKS board (43 x 43)."""
        from lib.params import MKS_SERVO42D_W

        corner = max(math.hypot(21.0, 22.0), math.hypot(MKS_SERVO42D_W / 2.0, MKS_SERVO42D_W / 2.0))
        assert CFG.shell.sleeve_bore_dia / 2.0 - corner >= 0.5
        assert CFG.shell.sleeve_od == CFG.bearings.out_bore < CFG.output_hub.od   # slides on, presses on the seat

    def test_the_held_plate_sits_inside_the_shell_ring(self):
        """1 a side between the held plate and the turning ring; the ring-pin holes 1.9 outside the ring's bore."""
        from lib.cycloidal import ring_pin_hole_dia

        sh, g = CFG.shell, CFG.gear
        assert (sh.ring_bore_dia - sh.plate_dia) / 2.0 == 1.0
        assert math.isclose(g.ring_pin_circle_radius - ring_pin_hole_dia(CFG) / 2.0 - sh.ring_bore_dia / 2.0, 1.9)

    def test_the_shell_is_held_both_ways_along_the_axis(self):
        """At each end the shell's lip stops the 6814's outer race (and clears the sleeve / the hub), a held plate's face
        its inner race - the motor plate's, the hub's flange's -, each face relieved over the outer race."""
        sh, b, h = CFG.shell, CFG.bearings, CFG.housing
        assert max(sh.sleeve_od, CFG.output_hub.od) < h.lip_bore_dia < b.out_od and sh.end_lip > 0
        assert b.out_bore < sh.plate_relief_dia < b.out_od
        assert hub_flange(CFG) == CFG.stack_up.z_motor_plate_inner and sh.plate_dia > b.out_bore


# ===================================================================
# 3. Radial clearances
# ===================================================================


def _profile_max_r():
    g = CFG.gear
    return compute_profile_radii(compute_epitrochoid(R=g.ring_pin_circle_radius, r=g.ring_pin_radius, N=g.num_ring_pins, e=g.eccentricity, num_points=CFG.profile.num_points))[1]


class TestRadialClearances:

    def test_disc_orbit_clears_housing_bore(self):
        swept_r = _profile_max_r() + CFG.gear.eccentricity
        bore_r = CFG.housing.bore_dia / 2.0
        assert swept_r < bore_r
        assert bore_r - swept_r >= 0.5

    def test_output_pins_inside_6814_bore(self):
        d = CFG.disc
        assert d.output_pin_circle_dia / 2.0 + d.output_pin_dia / 2.0 < CFG.bearings.out_bore / 2.0

    def test_output_pin_bottom_clears_motor_plate(self):
        """Pin bottom (z 11) stays above the motor plate inner face (z 9)."""
        S = stack_positions(CFG)
        assert S["z_output_pins"] > CFG.stack_up.z_motor_plate_inner

    def test_output_pin_engages_disc1(self):
        S = stack_positions(CFG)
        assert S["z_output_pins"] <= CFG.stack_up.z_disc1

    def test_output_hub_clears_retention_lip_bore(self):
        clearance = CFG.housing.lip_bore_dia - CFG.output_hub.od
        assert clearance >= 0.2

    def test_arm_mount_holes_clear_output_pins(self):
        """Nearest arm-hole / output-pin spacing > nut-pocket circumradius + pin-hole radius."""
        hub, d, h, tol = CFG.output_hub, CFG.disc, CFG.housing, CFG.tolerances
        arm_r, pin_r = hub.arm_mount_bolt_circle_dia / 2.0, d.output_pin_circle_dia / 2.0
        offset = math.radians(hub.arm_mount_angle_offset_deg)
        dist = math.sqrt(arm_r ** 2 + pin_r ** 2 - 2 * arm_r * pin_r * math.cos(offset))
        needed = hex_circumdiameter(h.bolt_nut_pocket_af) / 2.0 + (d.output_pin_dia - tol.ring_pin_press_sub) / 2.0
        assert dist > needed

    def test_output_hub_near_6814_inner(self):
        assert abs(CFG.output_hub.od - CFG.bearings.out_bore) <= 0.5

    def test_6814_outer_fits_housing_seat(self):
        assert CFG.bearings.out_od <= CFG.housing.output_bearing_seat_dia

    def test_ring_pins_inside_housing_bore(self):
        g = CFG.gear
        assert g.ring_pin_circle_dia / 2.0 + g.ring_pin_dia / 2.0 < CFG.housing.bore_dia / 2.0

    def test_disc_output_pin_holes_clear_center_bore(self):
        d = CFG.disc
        assert (d.output_pin_circle_dia / 2.0 - d.output_pin_hole_dia / 2.0) - d.center_bore_dia / 2.0 >= 5.0


# ===================================================================
# 4. Bearing retention
# ===================================================================


class TestBearingRetention:

    def test_6814_retained_by_press_fit(self):
        gap = CFG.housing.output_bearing_seat_dia - CFG.bearings.out_od
        assert 0 < gap < 0.5

    def test_6814_retained_by_integral_lip(self):
        assert CFG.housing.lip_bore_dia < CFG.bearings.out_od

    def test_6003_retained_by_disc_bore(self):
        assert CFG.bearings.ecc_od <= CFG.disc.center_bore_dia

    def test_625_retained_by_output_hub_pocket(self):
        assert CFG.bearings.inp_od + CFG.tolerances.bearing_seat_bore_add < CFG.output_hub.od


# ===================================================================
# 5. Shaft reach
# ===================================================================


class TestShaftReach:

    def test_motor_shaft_does_not_bottom_in_d_bore(self):
        m, s = CFG.motor, CFG.stack_up
        shaft_past_plate = m.shaft_length - (s.motor_plate_wall + s.motor_plate_inner_wall)     # 13
        assert CFG.shaft.d_bore_depth - shaft_past_plate >= 0.5

    def test_eccentric_shaft_pin_reaches_625_bearing(self):
        s, shaft = CFG.stack_up, CFG.shaft
        pin_tip_z = s.z_disc2 + s.disc_thickness + (shaft.support_pin_length - shaft.support_pin_hole_depth)
        assert pin_tip_z >= stack_positions(CFG)["z_625"] + CFG.bearings.inp_width

    def test_eccentric_shaft_pin_fits_625_bore(self):
        assert CFG.shaft.support_pin_dia <= CFG.bearings.inp_bore

    def test_eccentric_shaft_pin_clears_hub_bore(self):
        assert CFG.output_hub.shaft_clearance_bore - CFG.shaft.support_pin_dia >= 0.5


# ===================================================================
# 6. Ring pin span
# ===================================================================


class TestRingPinSpan:

    def test_pin_length_spans_the_gear_plus_engagement(self):
        """40 mm pins: 5 mm into the pin ring at each end past the 30 mm between the plates (>= 3 mm)."""
        assert ring_pin_engagement(CFG) >= 3.0

    def test_pin_length_equals_40mm(self):
        assert CFG.gear.ring_pin_length == 40.0

    def test_disc_zone_is_26mm(self):
        assert abs(CFG.stack_up.disc_zone - 26.0) < 0.01


# ===================================================================
# 7. Housing bolt engagement
# ===================================================================


class TestHousingBoltEngagement:
    """The turning shell's M4 x 65: heads in the shell ring, end to end through the body into nuts sunk in it."""

    def test_full_nut_engagement(self):
        """The bolts' ends flush with the nuts' outer faces."""
        h, S = CFG.housing, stack_positions(CFG)
        assert math.isclose(S["z_housing_bolts"] + h.bolt_head_height + h.bolt_length, S["z_housing_nuts"] + h.bolt_nut_thickness)

    def test_bolt_ends_inside_the_body(self):
        """The nuts sunk in the body's hub end, short of it - so the bolts' ends stay off the yoke's leg; the heads
        sunk in the shell ring's end."""
        h, S, (end_0, end_1) = CFG.housing, stack_positions(CFG), shell_ends(CFG)
        assert CFG.stack_up.z_output_bearings < S["z_housing_nuts"] and S["z_housing_nuts"] + h.bolt_nut_thickness < end_1
        assert S["z_housing_bolts"] > end_0

    def test_counterbore_recesses_head(self):
        assert CFG.housing.bolt_counterbore_depth >= CFG.housing.bolt_head_height

    def test_counterbore_wall_to_od(self):
        h = CFG.housing
        assert h.od / 2.0 - (h.bolt_circle_dia / 2.0 + h.bolt_counterbore_dia / 2.0) >= 2.0

    def test_counterbore_fits_in_the_shell_ring(self):
        assert CFG.housing.bolt_counterbore_depth < CFG.bearings.out_width + CFG.shell.end_lip


# ===================================================================
# 8. The interference budget, module locks, the pose in the arm
# ===================================================================


@pytest.fixture(scope="module")
def drive():
    return built.model(cycloidal_drive.cycloidal_drive)


@pytest.fixture(scope="module")
def drive_world(drive):
    """The module at placements.json "cycloidal_drive#1" (the SolidWorks node's pose)."""
    return drive.moved(P.location(DRIVE_KEY, "world"))


class TestModuleLocks:

    def test_occurrences_follow_stack_positions(self, stack):
        """Every module row is placed by stack_positions (the drive repo's assembly.py numbers)."""
        want = {
            "cycloidal_disc_1": (stack["x_disc1"], 0, stack["z_disc1"]), "bearing_6003:1": (stack["x_disc1"], 0, stack["z_disc1"]),
            "cycloidal_disc_2": (stack["x_disc2"], 0, stack["z_disc2"]), "bearing_6003:2": (stack["x_disc2"], 0, stack["z_disc2"]),
            "bearing_6814:1": (0, 0, 48), "bearing_6814:2": (0, 0, -10),
            "cycloidal_eccentric_shaft": (0, 0, 0), "cycloidal_ring_pins": (0, 0, 4), "cycloidal_output_pins": (0, 0, 12),
            "nema17_48mm": (0, 0, 0), "mks_servo42d": (0, 0, stack["z_mks_board"]), "cycloidal_motor_bolts": (0, 0, -5), "cycloidal_motor_plate": (0, 0, 0),
            "cycloidal_shell_ring": (0, 0, 0),
            "cycloidal_output_hub": (0, 0, 39), "cycloidal_shaft_support_pin": (0, 0, 24),
            "bearing_625": (0, 0, 39), "cycloidal_housing_bolts": (0, 0, -12.5), "cycloidal_housing_nuts": (0, 0, 53.3),
        }
        got = {name if role is None else f"{name}:{role}": pos for name, role, pos in cycloidal_drive.OCCURRENCES}
        assert got.keys() == want.keys()
        for label, pos in want.items():
            assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(got[label], pos, strict=True)), label
        for name, _role, pos in cycloidal_drive.OCCURRENCES:      # rows are positions (data): translations only
            assert isinstance(pos, tuple) and len(pos) == 3 and all(isinstance(v, float) for v in pos), name

    def test_bodies_partition_the_rows(self):
        """stator + rotor (the robot description's rigid bodies) cover every row's part exactly
        once; the rotor is what turns with the shell (its body is j1_link's), with both 6814s (their outer races turn)."""
        names = {name for name, _, _ in cycloidal_drive.OCCURRENCES}
        bodies = cycloidal_drive.BODIES
        assert set(bodies) == {"stator", "rotor"}
        assert bodies["stator"] | bodies["rotor"] == names
        assert not (bodies["stator"] & bodies["rotor"])
        assert bodies["rotor"] == {"cycloidal_shell_ring", "cycloidal_ring_pins", "cycloidal_housing_bolts",
                                   "cycloidal_housing_nuts", "bearing_6814"}

    @pytest.mark.slow
    def test_module_totals_match_lock(self, drive):
        totals = cycloidal_drive.totals(shape=drive)
        for key in ("leaves", "solids"):
            assert totals[key] == cycloidal_drive.EXPECTED[key], key
        assert abs(totals["solid_volume"] - cycloidal_drive.EXPECTED["solid_volume"]) <= 0.5
        # X / Y: the shell's pillars, turned bolt_start_deg (the upper arm's centreline on the first), their chamfered
        # outer corners; Z: 48 motor + 14.1 MKS board behind the plate, 62 to the hub face
        assert totals["bbox_size"] == [127.775, 125.532, 124.1]
        bodies = {body: cycloidal_drive.totals(body, shape=drive) for body in cycloidal_drive.BODIES}
        for body, got in bodies.items():
            want = cycloidal_drive.EXPECTED["bodies"][body]
            for key in ("leaves", "solids"):
                assert got[key] == want[key], (body, key)
            assert abs(got["solid_volume"] - want["solid_volume"]) <= 0.5, body
        for key in ("leaves", "solids"):
            assert sum(t[key] for t in bodies.values()) == totals[key], key
        assert abs(sum(t["solid_volume"] for t in bodies.values()) - totals["solid_volume"]) <= 0.01

    @pytest.mark.slow
    def test_module_colors(self, drive):
        """Standalone: the purchased parts BOUGHT_TINT, the printed ones the module's TINT."""
        assert drive.label == "cycloidal_drive"
        assert module_tints(drive, cycloidal_drive.TINT) == {True: 13, False: 6}   # + the MKS board

    @pytest.mark.slow
    def test_module_interference_budget(self, drive):
        """Only the designed overlaps exist (mm^3): the two 6814 press fits (on the hub, on the motor plate's sleeve), the
        bolts through the solid nuts, the motor-bolt heads in the plate, the two 6003/lobe press fits, and the
        thread engagement of the motor bolts (front) and the MKS kit's M3x30 (rear) in the vendor motor's
        tapped holes (its holes are modelled at the M3 minor diameter, the bolts at the major). Every other mating pair
        is clean - the discs among them: disc 1 against the shaft and its 6003, both against the ring pins (the buggy
        identical-discs build overlapped the pins by several mm^3). The shell's body is j1_link's: TestPoseInTheArm. A
        boolean between the two spline discs takes minutes: tests/cycloidal/test_port.py tells them apart."""
        leaves = {c.label: c for c in drive.children}
        budget = {
            ("bearing_6814:1", "cycloidal_output_hub"): 331.0, ("bearing_6814:2", "cycloidal_motor_plate"): 331.0,
            ("cycloidal_housing_bolts", "cycloidal_housing_nuts"): 241.3,
            ("cycloidal_motor_bolts", "cycloidal_motor_plate"): 52.0, ("nema17_48mm", "cycloidal_motor_bolts"): 46.4,
            ("bearing_6003:1", "cycloidal_eccentric_shaft"): 27.0, ("bearing_6003:2", "cycloidal_eccentric_shaft"): 27.0,
            ("mks_servo42d", "nema17_48mm"): 159.4,
        }
        for (a, b), limit in budget.items():
            vol = interference(leaves[a], leaves[b])
            assert limit * 0.9 <= vol <= limit, f"{a} x {b}: {vol:.1f} mm^3 (designed ~{limit})"
        clean = [
            ("cycloidal_ring_pins", "cycloidal_motor_plate"), ("cycloidal_ring_pins", "cycloidal_output_hub"),
            ("cycloidal_output_pins", "cycloidal_output_hub"), ("cycloidal_housing_bolts", "cycloidal_motor_plate"),
            ("cycloidal_output_pins", "cycloidal_motor_plate"), ("cycloidal_output_hub", "cycloidal_shell_ring"),
            ("bearing_625", "cycloidal_output_hub"), ("bearing_625", "cycloidal_shaft_support_pin"),
            ("cycloidal_shaft_support_pin", "cycloidal_eccentric_shaft"), ("nema17_48mm", "cycloidal_motor_plate"),
            ("cycloidal_eccentric_shaft", "cycloidal_motor_plate"), ("bearing_6814:1", "bearing_6814:2"),
            ("mks_servo42d", "cycloidal_motor_bolts"), ("mks_servo42d", "cycloidal_motor_plate"),
            ("cycloidal_shell_ring", "cycloidal_motor_plate"), ("cycloidal_ring_pins", "cycloidal_shell_ring"),
            ("cycloidal_housing_bolts", "cycloidal_shell_ring"), ("bearing_6814:2", "cycloidal_shell_ring"),
            ("nema17_48mm", "cycloidal_shell_ring"), ("bearing_6814:1", "cycloidal_motor_plate"),
            ("bearing_6814:2", "cycloidal_output_hub"), ("bearing_6814:1", "cycloidal_ring_pins"),
            ("cycloidal_disc_1", "cycloidal_eccentric_shaft"), ("cycloidal_disc_1", "bearing_6003:1"),
            ("cycloidal_disc_1", "cycloidal_ring_pins"), ("cycloidal_disc_2", "cycloidal_ring_pins"),
            ("cycloidal_disc_1", "cycloidal_motor_plate"), ("cycloidal_disc_2", "cycloidal_output_hub"),
        ]
        for a, b in clean:
            vol = interference(leaves[a], leaves[b])
            assert vol < 1.0, f"{a} x {b}: {vol:.2f} mm^3"


@pytest.mark.slow
class TestPoseInTheArm:
    """The drive attached at the SolidWorks node's pose: axis horizontal (along -N), its hub and motor plate held by the
    j1_coupler fork's two legs, the shell's body printed with j1_link - and it IS the robot's shoulder_pitch joint."""

    def test_module_world_bbox_matches_solidworks_node(self, drive_world):
        """The SolidWorks node never carried the MKS board (2026-09-21): compare the module without it. The node
        holds the port's 8-pillar housing (lib/cycloidal/params.py LEGACY_CONFIG): across the axis the module lies inside
        its box (DEFAULT_CONFIG's housing is RING_INSET smaller all round); along the axis it starts at the node's motor
        end and runs to the held hub's face (hub_top) where the node ran to its hub face (65) - the node's box moved by the
        drive's shift (lib/placements.py SHIFTS: the yoke holds the middle of its discs on the base_yaw axis)."""
        sw = P.OCCURRENCES[DRIVE_KEY]["solidworks"]
        # the children keep their module-frame locations; the module's world pose sits on the Compound
        node = Compound([c for c in drive_world.children if c.label.split(":")[0] != "mks_servo42d"]).moved(drive_world.location)
        lo, size = R.bbox_min(node), R.bbox_size(node)
        sw_lo, sw_size = [a + d for a, d in zip(sw["world_bbox_min"], P.shift(DRIVE_KEY), strict=True)], sw["world_bbox_size"]
        world = P.location(DRIVE_KEY, "world")
        axis = (world * Location((0, 0, 1))).position - world.position
        along = max(range(3), key=lambda i: abs(tuple(axis)[i]))
        longer = stack_positions(CFG)["hub_top"] - 65.0
        for i in range(3):
            if i == along:
                assert abs(size[i] - longer - sw_size[i]) <= 1.5, (i, lo, size, sw_lo, sw_size)
                assert min(abs(lo[i] - sw_lo[i]), abs(lo[i] + size[i] - sw_lo[i] - sw_size[i])) <= 1.5, (i, lo, size, sw_lo, sw_size)
            else:
                assert lo[i] >= sw_lo[i] - 1.5 and lo[i] + size[i] <= sw_lo[i] + sw_size[i] + 1.5, (i, lo, size, sw_lo, sw_size)

    def test_drive_clears_arm_neighbours(self, drive_world):
        """No intersection with the base, j1_link (the shell's body: the shell ring on it, the housing bolts through its
        holes, the nuts in its pockets, the ring pins in its pin ring, the hub-end 6814 in its seat), the j1_coupler
        fork (the hub's face on its hub-side leg, the sleeve through the motor-side one) or the fork's cap - all
        contact at most."""
        for key in ("base#1", "j1_link#1", "j1_coupler#1", "j1_coupler_cap#1"):
            vol = interference(drive_world, built.placed(key))
            assert vol <= 1.0, f"drive x {key}: {vol:.1f} mm^3"

    def test_the_shell_in_j1_link_and_the_hub_and_sleeve_on_the_yoke(self):
        """j1_link holds the shell's body: its face on the shell ring (arm_zone's start - the body's end face, in pieces
        between its windows), its hub end (shell_ends); the held hub's face (hub_top) on the inner face of the
        j1_coupler fork's hub-side leg, the sleeve's end on the motor-side leg's outer face. Each seat: the planar faces
        perpendicular to the drive axis in its plane, their area together."""
        from lib.cycloidal import arm_zone

        world = P.location(DRIVE_KEY, "world")
        axis = (world * Location((0, 0, 1))).position - world.position      # the drive axis in world
        for key, z, area in (("j1_link#1", arm_zone(CFG)[0], 1500), ("j1_link#1", shell_ends(CFG)[1], 2000),
                             ("j1_coupler#1", stack_positions(CFG)["hub_top"], 2000), ("j1_coupler#1", sleeve_end(CFG), 500)):
            centre = (world * Location((0, 0, z))).position
            faces = [f for f in built.placed(key).faces().filter_by(GeomType.PLANE)
                     if abs(f.normal_at().dot(axis)) > 0.99 and abs((centre - f.center()).dot(f.normal_at())) < 0.1]
            seat = sum(f.area for f in faces)
            assert seat > area, f"{key}'s planar faces perpendicular to the drive axis at module z {z:.3f}: {seat:.0f} mm^2"
        assert axis.dot(Vector(*F.N)) < -0.99, "the drive axis should point along -N (toward j1_link)"
        joint = F.JOINT_BY_NAME["shoulder_pitch"]                       # the drive IS this joint
        off_axis = (Vector(*joint.origin_w) - world.position).cross(axis).length
        assert off_axis < 1e-3, f"shoulder_pitch origin is {off_axis:.4f} mm off the drive axis"
        assert Vector(*joint.axis_w).dot(axis) < -0.99, "shoulder_pitch turns about N = the drive's -Z"
