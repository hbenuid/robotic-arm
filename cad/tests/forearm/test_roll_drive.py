"""The forearm roll drive (assemblies/forearm_roll_drive.py): the joint's axis through the wrist centre, the module
frame, the stack, the parts' fits inside the module, and the module's clearances in the arm (at the capture pose
and with the elbow folded to its limits)."""
import math

import pytest
from build123d import Axis, Location, Vector

from assemblies import forearm_roll_drive as M
from lib import placements as P
from lib import params as PARAMS
from lib import reference as R
from lib.belts import GT2_PULLEY_20T_TEETH, centre_distance
from lib.datum import to_location
from lib.forearm import DEFAULT, module_frame_in_host, stack_positions
from lib.models import raw
from robot import frames as F
from tests.forearm.helpers import in_host, interference, is_inside

import parts

S = stack_positions(DEFAULT)
D = DEFAULT.drive


def _offset_from_line(point, origin, axis) -> float:
    d = [p - o for p, o in zip(point, origin)]
    t = sum(x * y for x, y in zip(d, axis))
    return math.sqrt(sum((x - t * y) ** 2 for x, y in zip(d, axis)))


def test_roll_axis_through_the_wrist_centre_and_the_wrist_axes_concurrent():
    assert _offset_from_line(F.WRIST_CENTRE, F.FOREARM_ROLL_ORIGIN, F.FOREARM_ROLL_AXIS) < 0.05
    assert _offset_from_line(F.WRIST_CENTRE, F.WRIST_PITCH_ORIGIN, F.N) < 0.05
    assert _offset_from_line(F.WRIST_CENTRE, F.WRIST_ROLL_ORIGIN, F.F) < 0.05
    assert abs(sum(x * y for x, y in zip(F.FOREARM_ROLL_AXIS, F.N))) < 1e-6           # x_hint perpendicular to the axis
    assert abs(math.sqrt(sum(v * v for v in F.FOREARM_ROLL_AXIS)) - 1.0) < 1e-9
    j = F.JOINT_BY_NAME["forearm_roll"]
    assert (j.parent, j.child, j.type) == ("elbow_link", "forearm_link", "revolute")
    assert [jj.name for jj in F.JOINTS][:5] == ["base_yaw", "shoulder_pitch", "elbow_pitch", "forearm_roll", "wrist_pitch"]
    assert F.LINK_ORDER.index("elbow_link") == F.LINK_ORDER.index("forearm_link") - 1


def test_module_frame_and_record():
    loc = to_location(module_frame_in_host(DEFAULT))
    x = (loc * Location((1.0, 0.0, 0.0))).position - loc.position
    z = (loc * Location((0.0, 0.0, 1.0))).position - loc.position
    assert (x - Vector(0, 0, 1)).length < 1e-9 and (z - Vector(-1, 0, 0)).length < 1e-9   # module +X = host +Z, +Z = host -X
    assert loc.position.Z == DEFAULT.roll_end.axis_z == PARAMS.FOREARM_ROLL_AXIS_Z
    world = P.location("forearm_roll_drive#1", "world")
    zw = (world * Location((0.0, 0.0, 1.0))).position - world.position
    assert abs(zw.dot(Vector(*F.FOREARM_ROLL_AXIS)) - 1.0) < 1e-6                       # +Z toward the wrist
    assert _offset_from_line(tuple(world.position), F.FOREARM_ROLL_ORIGIN, F.FOREARM_ROLL_AXIS) < 0.01


def test_stack():
    assert S["z_seat"] - S["z_end"] == D.end_wall + D.lip
    assert S["z_bearing_2"] == S["z_bearing_1"] + D.bearing_width + D.bearing_gap
    assert S["z_face"] == S["z_bearing_2"] + D.bearing_width == S["z_retainer"]
    assert math.isclose(S["z_ring_mid"] - S["z_motor_face"], D.t20) and S["z_20t"] - S["z_pad_top"] == D.pulley_lift
    assert S["z_wall"] == -DEFAULT.roll_end.wall_x[1] and S["z_flange"] + D.flange_t == S["z_wall"]
    assert S["z_spigot_end"] - S["z_wall"] == D.spigot_len == DEFAULT.roll_end.flange_recess_depth
    assert S["z_end"] >= 35.0 + DEFAULT.disc.nut_af / math.cos(math.radians(30)) / 2.0 + 0.5   # past the disc's (-35, 0) nut pocket
    assert D.shoulder_od < D.inner_race_od < D.bearing_od
    assert D.tube_od / 2.0 - (D.bearing_od + D.seat_add) / 2.0 >= 3.0                       # the seat's wall
    assert D.tube_od / 2.0 - DEFAULT.roll_end.axis_z <= 5.5 + 3.0                           # the tube bottom stays 3 mm above the upper arm's slab (host z -8.5)
    assert math.isclose(centre_distance(D.roll_belt, D.ring_teeth, GT2_PULLEY_20T_TEETH), D.motor_offset, abs_tol=0.1)
    assert D.flange_dia == DEFAULT.roll_end.flange_dia and D.bore == DEFAULT.roll_end.cable_bore
    assert D.stop_deg == PARAMS.FOREARM_ROLL_LIMIT_DEG
    assert D.stop_r > D.ring_flange_dia / 2.0 + 2.0 and D.retainer_id > D.shoulder_od


@pytest.fixture(scope="module")
def module():
    return raw(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


@pytest.mark.slow
def test_totals_and_bodies(module):
    got = M.totals()
    assert (got["leaves"], got["solids"]) == (M.EXPECTED["leaves"], M.EXPECTED["solids"])
    assert abs(got["solid_volume"] - M.EXPECTED["solid_volume"]) < 0.5
    for body in M.BODIES:
        got = M.totals(body)
        assert (got["leaves"], got["solids"]) == (M.EXPECTED["bodies"][body]["leaves"], M.EXPECTED["bodies"][body]["solids"])
        assert abs(got["solid_volume"] - M.EXPECTED["bodies"][body]["solid_volume"]) < 0.5
    assert module.is_valid


@pytest.mark.slow
def test_bearings_press_on_the_journals_and_slip_in_the_seat(module):
    block, shaft = _leaf(module, "forearm_roll_block"), _leaf(module, "forearm_roll_shaft")
    press = math.pi / 4.0 * ((D.bearing_bore + D.journal_add) ** 2 - D.bearing_bore ** 2) * D.bearing_width   # ~132 mm^3
    for label in ("bearing_6808:1", "bearing_6808:2"):
        b = _leaf(module, label)
        assert interference(b, block) < 1.0, label                         # the seat is the bearing OD + seat_add
        assert 0.9 * press <= interference(b, shaft) <= press + 0.5, label   # the journal's interference in the inner race
    bb = _leaf(module, "bearing_6808:1").bounding_box()
    assert math.isclose(bb.min.Z, S["z_bearing_1"], abs_tol=1e-6) and math.isclose(bb.max.Z, S["z_bearing_1"] + D.bearing_width, abs_tol=1e-6)


@pytest.mark.slow
def test_everything_else_in_the_module_is_clean(module):
    names = [c.label for c in module.children]
    press_pairs = {("bearing_6808:1", "forearm_roll_shaft"), ("bearing_6808:2", "forearm_roll_shaft")}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if (a, b) in press_pairs or (b, a) in press_pairs:
                continue
            if {a, b} == {"nema17_40mm", "mks_servo42d"}:
                continue   # the kit's screws run through the motor by design
            vol = interference(_leaf(module, a), _leaf(module, b))
            assert vol < 1.0, f"{a} x {b}: {vol:.1f} mm^3"


@pytest.mark.slow
def test_block_and_shaft_features(module):
    block, shaft = _leaf(module, "forearm_roll_block"), _leaf(module, "forearm_roll_shaft")
    r_seat = (D.bearing_od + D.seat_add) / 2.0
    assert not is_inside(block, r_seat - 0.5, 0, S["z_seat"] + 3) and is_inside(block, r_seat + 0.5, 0, S["z_seat"] + 3)    # the seat
    assert is_inside(block, -(D.lip_id / 2.0 + 0.5), 0, S["z_lip"] + 1.5) and not is_inside(block, -(D.lip_id / 2.0 - 0.5), 0, S["z_lip"] + 1.5)   # the lip (-X: away from the cable window)
    assert is_inside(block, -5, 0, S["z_end"] + 1.5) and not is_inside(block, 5, 0, S["z_end"] + 1.5)   # the end wall, its cable window (+X side)
    assert is_inside(block, 0, 20, S["z_end"] + 1.5)
    assert not is_inside(block, 0, 0, 0) and is_inside(block, -15, 0, 30)                                # the disc's bore, the disc
    assert is_inside(block, S["x_motor"] + 10, 10, S["z_motor_face"] + 1.5)                              # the pad plate (above the mounting face)
    assert not is_inside(block, S["x_motor"], 0, S["z_motor_face"] + 1.5)                                # its pilot slot
    assert not is_inside(block, S["x_motor"] + 10, 10, S["z_motor_face"] - 1.5)                          # the motor body's space below
    assert not is_inside(shaft, 0, 0, S["z_ring_mid"]) and is_inside(shaft, D.bore / 2.0 + 1, 0, S["z_ring_mid"])   # the bore, the core
    assert is_inside(shaft, 27.0, 0, S["z_ring_mid"]) and not is_inside(shaft, 28.3, 0, S["z_ring_mid"])   # a tooth land vs a groove at angle 0? land at 0 deg:
    assert is_inside(shaft, 0, 27.5, S["z_ring_mid"])                                                       # the ring exists all round
    assert not is_inside(shaft, 0, 23, S["z_flange"] + 1) and is_inside(shaft, 10, 10, S["z_flange"] + 1)   # a flange bolt hole, the flange
    assert not is_inside(shaft, 0, 23, S["z_flange"] + 4.5)                                                 # ... through the spigot
    bb = shaft.bounding_box()
    assert math.isclose(bb.max.Z, S["z_spigot_end"], abs_tol=1e-6) and math.isclose(bb.min.Z, S["z_shaft_end"], abs_tol=1e-6)


def _placed_module():
    return raw(M.forearm_roll_drive).moved(to_location(module_frame_in_host(DEFAULT)))


@pytest.mark.slow
def test_module_clears_its_neighbours_in_the_arm():
    """In j2_link's frame: the module against the forearm parts and the elbow's SolidWorks parts at the capture
    pose (the block's disc mates with j3_coupler#1, the shaft's spigot sits in j2_link's recess: contact, no overlap)."""
    module = _placed_module()
    link = parts.build("j2_link")
    assert interference(module, link) < 1.0
    for key in ("gt2_pulley_90t#1", "j3_coupler#1", "j1_link#1", "j1_cap#1", "nema17_40mm#2", "mks_servo42d#2",
                "nema17_40mm#3", "mks_servo42d#3", "j2_cap_1#1", "j2_cap_2#1"):
        vol = interference(module, in_host(key))
        assert vol < 1.0, f"module x {key}: {vol:.1f} mm^3"
    # the rotor's spigot is IN the wall's recess (the flange face on the wall): the shaft reaches the wall's elbow face
    # plus the spigot (a child read on its own has no parent placement - place the leaf itself)
    shaft = next(c for c in raw(M.forearm_roll_drive).children if c.label == "forearm_roll_shaft").moved(to_location(module_frame_in_host(DEFAULT)))
    assert math.isclose(shaft.bounding_box().min.X, -S["z_spigot_end"], abs_tol=1e-6)
    assert interference(shaft, link) < 1.0


@pytest.mark.slow
@pytest.mark.parametrize("deg", [-PARAMS.ELBOW_PITCH_LIMIT_DEG, -60.0, 60.0, PARAMS.ELBOW_PITCH_LIMIT_DEG])
def test_module_clears_the_folded_upper_arm(deg):
    """The upper arm (j1_link, j1_cap, its motor + board) swung about the elbow axis (host z through the origin) by
    the elbow's limits never runs into the block, its motor tower or the retainer post."""
    module = _placed_module()
    axis = Axis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    for key in ("j1_link#1", "j1_cap#1", "nema17_40mm#2", "mks_servo42d#2"):
        vol = interference(module, in_host(key).rotate(axis, deg))
        assert vol < 1.0, f"elbow {deg:+.0f} deg: module x {key}: {vol:.1f} mm^3"
