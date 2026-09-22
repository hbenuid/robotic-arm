"""Assembly-level checks of the cycloidal drive (ported from cycloidal_drive
tests/test_assembly_clearances.py) plus the module's own locks and its attachment to the arm.

  1. Axial stack-up - Z positions consistent, no gaps or overlaps
  2. Housing alignment - motor plate and ring gear body mate flush
  3. Radial clearances - discs clear the housing, pins clear bearings, hub clears the lip
  4. Bearing retention - 6814 by press fit + integral lip; 6003 by the disc bore; 625 by the hub pocket
  5. Shaft reach - motor shaft in the D-bore, support pin through the 625
  6. Ring pin span; 7. housing bolt engagement
  8. Boolean checks on the key mating pairs, the module's interference budget, the module totals
     and layout locks, and the drive's pose in the arm (fits its neighbours, hub face on j1_link)
"""
import math

import pytest
from build123d import Compound, GeomType, Location, Vector

from assemblies import cycloidal_drive
from lib.models import raw
from assemblies._occurrences import place_world
from tests.cycloidal.helpers import CFG, interference
from lib import placements as P
from lib import reference as R
from lib.cycloidal import compute_housing_bolt_angles, hex_circumdiameter, hub_height, stack_positions
from lib.cycloidal.profiles import compute_epitrochoid, compute_profile_radii
import parts
from robot import frames as F

cycloidal_disc_1 = parts.load("cycloidal_disc_1")
cycloidal_motor_plate = parts.load("cycloidal_motor_plate")
cycloidal_output_hub = parts.load("cycloidal_output_hub")
cycloidal_ring_gear_body = parts.load("cycloidal_ring_gear_body")
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

    def test_total_depth_is_60mm(self):
        """9 mm plate + 51 mm body (incl. the 3 mm output wall)."""
        assert abs(CFG.stack_up.total_housing_depth - 60.0) < 0.01

    def test_output_hub_protrudes_past_chassis(self):
        s, hub = CFG.stack_up, CFG.output_hub
        hub_top_z = s.z_output_bearings + hub_height(CFG)
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

    def test_ring_gear_body_height(self):
        s = CFG.stack_up
        assert abs(s.ring_gear_body_height - 51.0) < 0.01

    def test_all_housing_parts_same_od(self):
        assert CFG.housing.od == 140.0

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
        assert pin_tip_z >= s.z_output_bearings + CFG.bearings.inp_width

    def test_eccentric_shaft_pin_fits_625_bore(self):
        assert CFG.shaft.support_pin_dia <= CFG.bearings.inp_bore

    def test_eccentric_shaft_pin_clears_hub_bore(self):
        assert CFG.output_hub.shaft_clearance_bore - CFG.shaft.support_pin_dia >= 0.5


# ===================================================================
# 6. Ring pin span
# ===================================================================


class TestRingPinSpan:

    def test_pin_length_spans_bore_zone_plus_engagement(self):
        """35 mm pins: 3.5 mm engagement on each side of the 28 mm bore zone (>= 3 mm)."""
        engagement = (CFG.gear.ring_pin_length - CFG.stack_up.bore_zone) / 2.0
        assert engagement >= 3.0

    def test_pin_length_equals_35mm(self):
        assert CFG.gear.ring_pin_length == 35.0

    def test_disc_zone_is_26mm(self):
        assert abs(CFG.stack_up.disc_zone - 26.0) < 0.01


# ===================================================================
# 7. Housing bolt engagement
# ===================================================================


class TestHousingBoltEngagement:

    def test_bolt_reaches_nut(self):
        h, s = CFG.housing, CFG.stack_up
        assert h.bolt_counterbore_depth + h.bolt_length > s.total_housing_depth - h.bolt_nut_depth

    def test_full_nut_engagement(self):
        h, s = CFG.housing, CFG.stack_up
        engagement = h.bolt_counterbore_depth + h.bolt_length - (s.total_housing_depth - h.bolt_nut_depth)
        assert engagement >= h.bolt_nut_thickness

    def test_bolt_does_not_protrude(self):
        h, s = CFG.housing, CFG.stack_up
        assert h.bolt_counterbore_depth + h.bolt_length <= s.total_housing_depth

    def test_counterbore_recesses_head(self):
        assert CFG.housing.bolt_counterbore_depth >= CFG.housing.bolt_head_height

    def test_counterbore_wall_to_od(self):
        h = CFG.housing
        assert h.od / 2.0 - (h.bolt_circle_dia / 2.0 + h.bolt_counterbore_dia / 2.0) >= 2.0

    def test_counterbore_fits_in_motor_plate(self):
        s = CFG.stack_up
        assert CFG.housing.bolt_counterbore_depth < s.motor_plate_wall + s.motor_plate_inner_wall


# ===================================================================
# 8. Booleans on the key mating pairs, module locks, the pose in the arm
# ===================================================================


@pytest.fixture(scope="module")
def ring_body(stack):
    return Location((0, 0, stack["z_ring_gear_body"])) * cycloidal_ring_gear_body.build()


@pytest.fixture(scope="module")
def hub(stack):
    return Location((0, 0, stack["z_hub"])) * cycloidal_output_hub.build()


@pytest.fixture(scope="module")
def drive():
    return raw(cycloidal_drive.cycloidal_drive)


@pytest.fixture(scope="module")
def drive_world(drive):
    """The module at placements.json "cycloidal_drive#1" (the SolidWorks node's pose)."""
    return drive.moved(P.location(DRIVE_KEY, "world"))


@pytest.mark.slow
class TestMatingPairs:

    def test_motor_plate_ring_body_no_interference(self, ring_body):
        vol = interference(cycloidal_motor_plate.build(), ring_body)
        assert vol < 1.0, f"Motor plate / ring body interference = {vol:.1f}mm^3"

    def test_output_hub_protrudes_through_housing(self, hub, ring_body):
        proud = hub.bounding_box().max.Z - ring_body.bounding_box().max.Z
        assert abs(proud - CFG.output_hub.proud_above_housing) < 0.1

    def test_disc_clears_ring_gear_shoulder(self, stack, ring_body):
        disc = Location((stack["x_disc1"], 0, stack["z_disc1"])) * cycloidal_disc_1.build()
        vol = interference(disc, ring_body)
        assert vol < 1.0, f"Disc / ring gear body interference = {vol:.1f}mm^3"

    def test_output_hub_clears_ring_body(self, hub, ring_body):
        vol = interference(hub, ring_body)
        assert vol < 1.0, f"Output hub / ring body interference = {vol:.1f}mm^3"


class TestModuleLocks:

    def test_occurrences_follow_stack_positions(self, stack):
        """Every module row is placed by stack_positions (the drive repo's assembly.py numbers)."""
        want = {
            "cycloidal_disc_1": (stack["x_disc1"], 0, stack["z_disc1"]), "bearing_6003:1": (stack["x_disc1"], 0, stack["z_disc1"]),
            "cycloidal_disc_2": (stack["x_disc2"], 0, stack["z_disc2"]), "bearing_6003:2": (stack["x_disc2"], 0, stack["z_disc2"]),
            "bearing_6814:1": (0, 0, stack["z_6814_1"]), "bearing_6814:2": (0, 0, stack["z_6814_2"]),
            "cycloidal_eccentric_shaft": (0, 0, 0), "cycloidal_ring_pins": (0, 0, 5.5), "cycloidal_output_pins": (0, 0, 11),
            "nema17_48mm": (0, 0, 0), "mks_servo42d": (0, 0, stack["z_mks_board"]), "cycloidal_motor_bolts": (0, 0, -5), "cycloidal_motor_plate": (0, 0, 0),
            "cycloidal_ring_gear_body": (0, 0, 9), "cycloidal_output_hub": (0, 0, 37), "cycloidal_shaft_support_pin": (0, 0, 24),
            "bearing_625": (0, 0, 37), "cycloidal_housing_bolts": (0, 0, 0.5), "cycloidal_housing_nuts": (0, 0, 56),
        }
        got = {name if role is None else f"{name}:{role}": pos for name, role, pos in cycloidal_drive.OCCURRENCES}
        assert got.keys() == want.keys()
        for label, pos in want.items():
            assert all(math.isclose(a, b, abs_tol=1e-9) for a, b in zip(got[label], pos)), label
        for name, role, pos in cycloidal_drive.OCCURRENCES:      # rows are positions (data): translations only
            assert isinstance(pos, tuple) and len(pos) == 3 and all(isinstance(v, float) for v in pos), name

    def test_bodies_partition_the_rows(self):
        """stator + rotor (the robot description's rigid bodies) cover every row's part exactly
        once; the rotor is the hub side that j1_link is bolted to."""
        names = {name for name, _, _ in cycloidal_drive.OCCURRENCES}
        bodies = cycloidal_drive.BODIES
        assert set(bodies) == {"stator", "rotor"}
        assert bodies["stator"] | bodies["rotor"] == names
        assert not (bodies["stator"] & bodies["rotor"])
        assert bodies["rotor"] == {"cycloidal_output_hub", "cycloidal_output_pins", "bearing_625"}

    @pytest.mark.slow
    def test_module_totals_match_lock(self):
        totals = cycloidal_drive.totals()
        for key in ("leaves", "solids"):
            assert totals[key] == cycloidal_drive.EXPECTED[key], key
        assert abs(totals["solid_volume"] - cycloidal_drive.EXPECTED["solid_volume"]) <= 0.5
        assert totals["bbox_size"] == [140.0, 140.0, 127.1]   # 48 motor + 14.1 MKS board behind the plate, 65 to the hub face
        bodies = {body: cycloidal_drive.totals(body) for body in cycloidal_drive.BODIES}
        for body, got in bodies.items():
            want = cycloidal_drive.EXPECTED["bodies"][body]
            for key in ("leaves", "solids"):
                assert got[key] == want[key], (body, key)
            assert abs(got["solid_volume"] - want["solid_volume"]) <= 0.5, body
        for key in ("leaves", "solids"):
            assert sum(t[key] for t in bodies.values()) == totals[key], key
        assert abs(sum(t["solid_volume"] for t in bodies.values()) - totals["solid_volume"]) <= 0.01

    @pytest.mark.slow
    def test_module_interference_budget(self, drive):
        """Only the designed overlaps exist (mm^3): the two 6814/hub press fits, the bolts
        through the solid nuts, the motor-bolt heads in the plate, the two 6003/lobe press fits, and the
        thread engagement of the motor bolts (front) and the MKS kit's M3x30 (rear) in the vendor motor's
        tapped holes (its holes are modelled at the M3 minor diameter, the bolts at the major)."""
        leaves = {c.label: c for c in drive.children}
        budget = {
            ("bearing_6814:1", "cycloidal_output_hub"): 331.0, ("bearing_6814:2", "cycloidal_output_hub"): 331.0,
            ("cycloidal_housing_bolts", "cycloidal_housing_nuts"): 322.0,
            ("cycloidal_motor_bolts", "cycloidal_motor_plate"): 52.0, ("nema17_48mm", "cycloidal_motor_bolts"): 46.4,
            ("bearing_6003:1", "cycloidal_eccentric_shaft"): 27.0, ("bearing_6003:2", "cycloidal_eccentric_shaft"): 27.0,
            ("mks_servo42d", "nema17_48mm"): 159.4,
        }
        for (a, b), limit in budget.items():
            vol = interference(leaves[a], leaves[b])
            assert limit * 0.9 <= vol <= limit, f"{a} x {b}: {vol:.1f} mm^3 (designed ~{limit})"
        clean = [
            ("cycloidal_motor_plate", "cycloidal_ring_gear_body"), ("cycloidal_output_hub", "cycloidal_ring_gear_body"),
            ("cycloidal_ring_pins", "cycloidal_motor_plate"), ("cycloidal_ring_pins", "cycloidal_ring_gear_body"),
            ("cycloidal_output_pins", "cycloidal_output_hub"), ("cycloidal_housing_bolts", "cycloidal_motor_plate"),
            ("cycloidal_housing_bolts", "cycloidal_ring_gear_body"), ("cycloidal_housing_nuts", "cycloidal_ring_gear_body"),
            ("bearing_625", "cycloidal_output_hub"), ("bearing_625", "cycloidal_shaft_support_pin"),
            ("cycloidal_shaft_support_pin", "cycloidal_eccentric_shaft"), ("nema17_48mm", "cycloidal_motor_plate"),
            ("cycloidal_eccentric_shaft", "cycloidal_motor_plate"), ("bearing_6814:1", "cycloidal_ring_gear_body"),
            ("bearing_6814:2", "cycloidal_ring_gear_body"), ("bearing_6814:1", "bearing_6814:2"),
            ("mks_servo42d", "cycloidal_motor_bolts"), ("mks_servo42d", "cycloidal_motor_plate"),
        ]
        for a, b in clean:
            vol = interference(leaves[a], leaves[b])
            assert vol < 1.0, f"{a} x {b}: {vol:.2f} mm^3"


@pytest.mark.slow
class TestPoseInTheArm:
    """The drive attached at the SolidWorks node's pose: axis horizontal (along -N), housing in
    the j1_coupler yoke, hub face on j1_link - and it IS the robot's shoulder_pitch joint."""

    def test_module_world_bbox_matches_solidworks_node(self, drive_world):
        """The SolidWorks node never carried the MKS board (2026-09-21): compare the module without it."""
        sw = P.OCCURRENCES[DRIVE_KEY]["solidworks"]
        # the children keep their module-frame locations; the module's world pose sits on the Compound
        node = Compound([c for c in drive_world.children if c.label.split(":")[0] != "mks_servo42d"]).moved(drive_world.location)
        assert all(abs(a - b) <= 1.5 for a, b in zip(R.bbox_min(node), sw["world_bbox_min"])), (R.bbox_min(node), sw["world_bbox_min"])
        assert all(abs(a - b) <= 1.5 for a, b in zip(R.bbox_size(node), sw["world_bbox_size"])), (R.bbox_size(node), sw["world_bbox_size"])

    def test_drive_clears_arm_neighbours(self, drive_world):
        """No intersection with the base, j1_link or j1_cap; only contact-level overlap with the
        j1_coupler yoke it sits in (motor plate ~18 + ring gear body ~100 mm^3 measured)."""
        for key, limit in (("base#1", 1.0), ("j1_link#1", 1.0), ("j1_cap#1", 1.0), ("j1_coupler#1", 150.0)):
            part = P.OCCURRENCES[key]["part"]
            vol = interference(drive_world, place_world(part, key))
            assert vol <= limit, f"drive x {key}: {vol:.1f} mm^3 (limit {limit})"

    def test_hub_face_coplanar_with_j1_link_mount(self):
        """The hub's arm-mount face (module z=65) lies on j1_link's big mounting face."""
        world = P.location(DRIVE_KEY, "world")
        hub_centre = (world * Location((0, 0, stack_positions(CFG)["hub_top"]))).position
        axis = (world * Location((0, 0, 1))).position - world.position      # the drive axis in world
        link = place_world("j1_link", "j1_link#1")
        faces = [f for f in link.faces().filter_by(GeomType.PLANE) if abs(f.normal_at().dot(axis)) > 0.99 and f.area > 10000]
        assert faces, "j1_link has no large planar face perpendicular to the drive axis"
        face = max(faces, key=lambda f: f.area)
        distance = abs((hub_centre - face.center()).dot(face.normal_at()))
        assert distance < 0.1, f"hub face is {distance:.3f} mm off j1_link's mounting plane"
        assert axis.dot(Vector(*F.N)) < -0.99, "the drive axis should point along -N (toward j1_link)"
        joint = F.JOINT_BY_NAME["shoulder_pitch"]                       # the drive IS this joint
        off_axis = (Vector(*joint.origin_w) - world.position).cross(axis).length
        assert off_axis < 1e-3, f"shoulder_pitch origin is {off_axis:.4f} mm off the drive axis"
        assert Vector(*joint.axis_w).dot(axis) < -0.99, "shoulder_pitch turns about N = the drive's -Z"
