"""The forearm roll drive (assemblies/forearm_roll_drive.py): the joint's axis through the wrist centre, the module
frame, the stack round the elbow axis, the parts' fits inside the module, and the module's clearances in the arm (at
the capture pose, with the elbow folded to its limits and with the forearm rolled to its limits)."""
import math

import pytest
from build123d import Axis, Location, Vector

import parts
from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import placements as P
from lib.belts import GT2_PULLEY_20T_TEETH, centre_distance
from lib.datum import to_location
from lib.forearm import DEFAULT, belt_window, cap_bolt_points, module_frame_in_host, pulley_bolt_points, stack_positions
from lib.models import raw
from robot import frames as F
from tests.forearm.helpers import in_host
from tests.helpers import interference, is_inside

S = stack_positions(DEFAULT)
D = DEFAULT.drive
UPPER_ARM_FACE_Z = -8.5          # host z of j1_link's +N face (its slab lies below it)
UPPER_ARM_RECESS = (40.0, -14.5)  # j1_link's recess round the elbow axis: radius, floor (host z)
UPPER_ARM_BORE_R = 21.0          # j1_link's bore the coupler's stub / journal turn in
UPPER_ARM_END_R = 45.0           # j1_link's round end about the elbow axis
STUB_END_Z = -22.0               # host z of the block's stub end (the SolidWorks coupler's)
ELBOW_PULLEY_FACE_Z = STUB_END_Z - PARAMS.PULLEY_SEAT_SHIFT   # the elbow 90T's hub end: on j1_link's lip, under the stub's end


def _offset_from_line(point, origin, axis) -> float:
    d = [p - o for p, o in zip(point, origin, strict=True)]
    t = sum(x * y for x, y in zip(d, axis, strict=True))
    return math.sqrt(sum((x - t * y) ** 2 for x, y in zip(d, axis, strict=True)))


def test_roll_axis_through_the_wrist_centre_and_the_wrist_axes_concurrent():
    assert _offset_from_line(F.WRIST_CENTRE, F.FOREARM_ROLL_ORIGIN, F.FOREARM_ROLL_AXIS) < 0.05
    assert _offset_from_line(F.WRIST_CENTRE, F.WRIST_PITCH_ORIGIN, F.N) < 0.05
    assert _offset_from_line(F.WRIST_CENTRE, F.WRIST_ROLL_ORIGIN, F.F) < 0.05
    assert abs(sum(x * y for x, y in zip(F.FOREARM_ROLL_AXIS, F.N, strict=True))) < 1e-6           # x_hint perpendicular to the axis
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


def test_the_elbow_coupler_is_retired_for_the_block():
    """j3_coupler#1's record stays in placements.json but nothing claims it; the wrist's j3_coupler#2 is untouched."""
    assert P.RETIRED[0] == "j3_coupler#1" and "j3_coupler#1" in P.OCCURRENCES
    assert "j3_coupler#1" not in P.keys() and "j3_coupler#1" in P.keys(retired=True) and "j3_coupler#2" in P.keys()
    assert F.LINKS["elbow_link"] == ["gt2_pulley_90t#3", "forearm_roll_drive#1:stator"]   # the re-seated elbow 90T (lib/mounts.py)


def test_stack():
    w, z_axis = DEFAULT.roll_end, DEFAULT.roll_end.axis_z
    # the block round the elbow axis: its underside 0.5 above the upper arm's slab, the coupler features below it in
    # j1_link's recess and bore where the SolidWorks coupler's were, an inner-ring shoulder between the journal and the
    # stub, the elbow pulley PULLEY_SEAT_SHIFT below the stub's end (the lip between them)
    assert z_axis + D.block_x[0] >= UPPER_ARM_FACE_Z + 0.5
    assert D.lip_dia / 2.0 <= UPPER_ARM_RECESS[0] - 1.0 and D.boss_dia / 2.0 <= UPPER_ARM_RECESS[0] - 2.0
    assert D.lip_x[1] == D.block_x[0] and D.boss_x[1] == D.lip_x[0] and D.journal_x[1] == D.boss_x[0]
    assert D.step_x[1] == D.journal_x[0] and D.stub_x[1] == D.step_x[0] and D.stub_dia < D.step_dia < D.journal_dia
    assert z_axis + D.boss_x[0] >= UPPER_ARM_RECESS[1] + 0.5 and D.journal_dia / 2.0 < UPPER_ARM_BORE_R and D.stub_dia / 2.0 < UPPER_ARM_BORE_R
    assert z_axis + D.stub_x[0] == STUB_END_Z
    # the stations, rear end wall -> the forearm wall
    assert S["z_end"] == D.block_z[0] and S["z_lip"] == S["z_end"] + D.end_wall and S["z_seat"] == S["z_lip"] + D.lip == S["z_bearing_1"]
    assert S["z_bore"] == S["z_seat"] + D.bearing_width and S["z_bore"] < S["z_cavity"] == D.cavity_z0 < S["z_ring_flange_1"]
    assert S["z_ring"] == D.ring_z0 and S["z_ring_end"] + D.belt_window_margin < S["z_face"] == D.block_z[1] == S["z_cap"] == S["z_bearing_2"]
    assert S["z_neck"] == S["z_face"] + D.bearing_width and S["z_cap_outer"] == S["z_neck"] + D.cap_lip == S["z_stop_post"] == S["z_stop_lug"]
    assert S["z_stop_post"] + D.stop_t + 0.5 <= S["z_wall"] == -w.wall_x[1] and S["z_spigot_end"] - S["z_wall"] == w.flange_recess_depth
    assert S["z_shaft_end"] == S["z_lip"] + D.shaft_end_clear < S["z_seat"]
    assert math.isclose(S["z_ring_mid"] - S["z_motor_face"], D.t20) and S["z_20t"] - S["z_pad_top"] == D.pulley_lift
    # the shaft CROSSES the elbow axis: the bearings straddle it inside the block
    assert S["z_end"] < S["z_bearing_1"] and S["z_bearing_1"] + D.bearing_width < 0 < S["z_bearing_2"] <= S["z_face"]
    # the forearm wall: the rolling +/-45 wall (its corners r 57 about the roll axis) clears j1_link's round end
    assert S["z_wall"] >= UPPER_ARM_END_R + 1.5
    # walls: 2 mm under the cavity and the seat, over the inserts; the inserts 2 mm from the cavity's rear wall
    assert -D.block_x[0] - D.cavity_dia / 2.0 >= 2.0 and -D.block_x[0] - (D.bearing_od + D.seat_add) / 2.0 >= 2.0
    assert D.cavity_z0 - (D.pulley_bolt_r + D.insert_dia / 2.0) >= 2.0
    assert -D.insert_x[1] - D.core_bore_dia / 2.0 >= 2.0 and D.pin_bore_x[1] <= -D.core_bore_dia / 2.0 - 2.0
    assert D.insert_x[0] == D.boss_x[0] and D.pulley_bolt_r + D.insert_dia / 2.0 <= D.boss_dia / 2.0 - 2.0
    assert D.pulley_bolt_r + D.pulley_bolt_dia / 2.0 <= D.stub_dia / 2.0 - 1.5   # the SolidWorks coupler's own wall round them
    # bearing 2 slides on from the wrist end: everything beyond journal 2 is smaller than its bore; the ring passes the open cavity
    assert D.shoulder_od < D.inner_race_od < D.bearing_od and D.neck_od < D.bearing_bore and w.flange_dia < D.bearing_bore
    assert D.cavity_dia >= D.ring_flange_dia + 2.0 and D.lip_id > D.bearing_bore + D.journal_add
    assert D.bearing_od + 0.3 <= D.core_bore_dia < D.cavity_dia      # bearing 1 rides through the core bore to its seat from the front
    # the motor: on the block's top, centred on the roll axis in X, the belt setting its height, its body clear of the
    # top at the slot's low end, the plate past its bolt square, the cheeks clear of the board
    assert S["x_motor"] == 0.0 and math.isclose(S["y_motor"], D.centre_distance)
    assert math.isclose(D.centre_distance, centre_distance(D.roll_belt, D.ring_teeth, GT2_PULLEY_20T_TEETH), abs_tol=1e-9)
    assert S["y_motor"] - D.pad_slot_len / 2.0 - D.motor.body_width / 2.0 >= D.block_y[1] + 0.5
    assert S["y_plate_top"] >= S["y_motor"] + 15.5 + D.pad_bolt_dia / 2.0 + 2.0 and D.plate_w >= D.motor.body_width + 2.0
    assert S["z_cheek"] > S["z_motor_board"] and D.cheek_gap > 0.0
    # the belt window: the ring's width + the margins, from inside the cavity out through the top wall
    z0, z1, half_x, y0 = belt_window(DEFAULT)
    assert z0 == S["z_ring_flange_1"] - D.belt_window_margin and z1 == S["z_ring_end"] + D.belt_window_margin and z1 < S["z_face"]
    assert half_x < D.ring_flange_dia / 2.0 and y0 < D.cavity_dia / 2.0 < D.block_y[1]      # a slot for the runs, not the ring
    assert all(abs(x) - D.cap_tap_dia / 2.0 >= half_x + 2.0 for x, _ in cap_bolt_points(DEFAULT))   # the cap's taps beside the window
    assert D.motor_spin_deg == 90.0 and D.cheek_h + D.block_y[1] < S["y_motor"] - 8.0            # the connector (16 wide) clears the cheeks
    # the wall bolts into the shaft's end wall: 2 mm of PETG round each tap hole, between the bore and the spigot
    assert D.bore == w.cable_bore and D.cable_exit_dia > D.bore
    assert w.bolt_circle_dia / 2.0 - D.end_bolt_tap_dia / 2.0 >= D.bore / 2.0 + 2.0
    assert w.bolt_circle_dia / 2.0 + D.end_bolt_tap_dia / 2.0 <= w.flange_dia / 2.0 - 2.0
    assert D.stop_deg == PARAMS.FOREARM_ROLL_LIMIT_DEG == 180.0 - D.stop_deg_width
    assert D.stop_lug_r[1] > D.stop_post_r[0] and D.stop_lug_r[0] < D.neck_od / 2.0 < D.stop_post_r[0]
    # the cap's four bolts: inside the outline's rounded corners, outside the cavity
    for x, y in cap_bolt_points(DEFAULT):
        assert math.hypot(x, y) > D.cavity_dia / 2.0 + D.cap_tap_dia and abs(x) < D.block_x[1] and D.block_y[0] < y < D.block_y[1]
    assert len(pulley_bolt_points(DEFAULT)) == 4 and all(math.isclose(math.hypot(*yz), D.pulley_bolt_r) for yz in pulley_bolt_points(DEFAULT))


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
    block, shaft, cap = _leaf(module, "forearm_roll_block"), _leaf(module, "forearm_roll_shaft"), _leaf(module, "forearm_roll_retainer")
    press = math.pi / 4.0 * ((D.bearing_bore + D.journal_add) ** 2 - D.bearing_bore ** 2) * D.bearing_width   # ~132 mm^3
    for label, seat_part in (("bearing_6808:1", block), ("bearing_6808:2", cap)):
        b = _leaf(module, label)
        assert interference(b, seat_part) < 1.0, label                     # the seat is the bearing OD + seat_add
        assert 0.9 * press <= interference(b, shaft) <= press + 0.5, label   # the journal's interference in the inner race
    bb = _leaf(module, "bearing_6808:1").bounding_box()
    assert math.isclose(bb.min.Z, S["z_bearing_1"], abs_tol=1e-6) and math.isclose(bb.max.Z, S["z_bearing_1"] + D.bearing_width, abs_tol=1e-6)
    bb = _leaf(module, "bearing_6808:2").bounding_box()
    assert math.isclose(bb.min.Z, S["z_bearing_2"], abs_tol=1e-6) and math.isclose(bb.max.Z, S["z_neck"], abs_tol=1e-6)


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
    block, shaft, cap = _leaf(module, "forearm_roll_block"), _leaf(module, "forearm_roll_shaft"), _leaf(module, "forearm_roll_retainer")
    r_seat, r_lip, r_bore, r_cav = (D.bearing_od + D.seat_add) / 2.0, D.lip_id / 2.0, D.core_bore_dia / 2.0, D.cavity_dia / 2.0
    zs, zm, zc = S["z_seat"] + 3.0, S["z_ring_mid"], S["z_cap"] + 3.0
    # the housing bore: the seat, the lip, the end wall with its cable exit, the clearance bore, the cavity and its 2 mm floor
    assert not is_inside(block, 0, -(r_seat - 0.5), zs) and is_inside(block, 0, -(r_seat + 0.5), zs)
    assert is_inside(block, 0, -(r_lip + 0.5), S["z_lip"] + 1.0) and not is_inside(block, 0, -(r_lip - 0.5), S["z_lip"] + 1.0)
    assert not is_inside(block, 0, 0, S["z_end"] + 1.5) and is_inside(block, 0, 20, S["z_end"] + 1.5) and is_inside(block, -20, 0, S["z_end"] + 1.5)
    assert not is_inside(block, 0, r_bore - 0.5, 0) and is_inside(block, 0, r_bore + 1.0, 0)
    assert not is_inside(block, -(r_cav - 1.0), 0, zm) and is_inside(block, -(r_cav + 1.0), 0, zm) and not is_inside(block, 0, -(r_cav - 1.0), zm)
    _z0, z1, _half_x, _y0 = belt_window(DEFAULT)
    assert not is_inside(block, 0, r_cav + 1.5, zm) and is_inside(block, 0, r_cav + 1.5, z1 + 2.0)          # the belt window, the top wall past it
    # the front face is open (the cavity reaches it), the cap's tap holes in it
    assert not is_inside(block, 0, r_cav - 1.0, S["z_face"] - 0.5)
    for x, y in cap_bolt_points(DEFAULT):
        assert not is_inside(block, x, y, S["z_face"] - 4.0) and is_inside(block, x, y, S["z_face"] - D.cap_tap_depth - 2.0)
    # the coupler side: the lip inside its radius only, the boss, the stub, the pin bore, the pulley bolts + inserts
    assert is_inside(block, D.lip_x[0] + 1.0, 0, 20.0) and not is_inside(block, D.lip_x[0] + 1.0, 0, D.lip_dia / 2.0 + 2.0)   # (0, 0) is the pin bore
    assert is_inside(block, D.boss_x[0] + 1.0, 0, D.boss_dia / 2.0 - 2.0) and not is_inside(block, D.boss_x[0] + 1.0, 0, D.boss_dia / 2.0 + 1.0)
    assert is_inside(block, D.stub_x[0] + 2.0, 0, D.stub_dia / 2.0 - 1.0) and not is_inside(block, D.stub_x[0] + 2.0, 0, D.stub_dia / 2.0 + 1.0)
    assert not is_inside(block, D.stub_x[0] + 2.0, 0, 0) and not is_inside(block, D.pin_bore_x[1] - 1.0, 0, 0) and is_inside(block, D.pin_bore_x[1] + 1.0, 0, 0)
    for y, z in pulley_bolt_points(DEFAULT):
        assert not is_inside(block, D.stub_x[0] + 2.0, y, z) and not is_inside(block, (D.insert_x[0] + D.insert_x[1]) / 2.0, y, z)
        assert is_inside(block, D.insert_x[1] + 1.5, y, z)
    # the motor plate (with its pilot slot), the motor's space behind it, a cheek
    assert is_inside(block, 10, S["y_motor"] + 10, S["z_motor_face"] + 1.5) and not is_inside(block, 0, S["y_motor"], S["z_motor_face"] + 1.5)
    assert not is_inside(block, 10, S["y_motor"] + 10, S["z_motor_face"] - 1.5) and not is_inside(block, 0, D.block_y[1] + 5.0, -10.0)
    x_cheek = D.motor.body_width / 2.0 + D.cheek_gap + D.cheek_t / 2.0
    assert is_inside(block, x_cheek, D.block_y[1] + 5.0, -10.0) and is_inside(block, -x_cheek, D.block_y[1] + 5.0, -10.0)
    assert not is_inside(block, x_cheek, D.block_y[1] + 5.0, S["z_motor_board"] - 1.0)
    # the shaft: the bore, the core, journal 1, the ring (a land, a groove, all round), the neck, the lug, the spigot, a tap
    assert not is_inside(shaft, 0, 0, zm) and is_inside(shaft, D.bore / 2.0 + 1.0, 0, zm)
    assert is_inside(shaft, D.bearing_bore / 2.0 - 0.5, 0, S["z_seat"] + 3.0) and not is_inside(shaft, D.bearing_bore / 2.0 + 1.0, 0, S["z_seat"] + 3.0)
    assert is_inside(shaft, 27.0, 0, zm) and not is_inside(shaft, 28.3, 0, zm) and is_inside(shaft, 0, 27.5, zm)
    assert is_inside(shaft, 0, D.neck_od / 2.0 - 1.0, S["z_stop_lug"] + 1.0) and not is_inside(shaft, 0, D.neck_od / 2.0 + 1.0, S["z_stop_lug"] + 1.0)
    assert is_inside(shaft, (D.stop_lug_r[0] + D.stop_lug_r[1]) / 2.0, 0, S["z_stop_lug"] + 1.0)
    assert is_inside(shaft, DEFAULT.roll_end.flange_dia / 2.0 - 0.5, 0, S["z_wall"] + 1.0)
    assert not is_inside(shaft, DEFAULT.roll_end.bolt_circle_dia / 2.0, 0, S["z_wall"] + 1.0)
    bb = shaft.bounding_box()
    assert math.isclose(bb.max.Z, S["z_spigot_end"], abs_tol=1e-6) and math.isclose(bb.min.Z, S["z_shaft_end"], abs_tol=1e-6)
    # the cap: seat 2, the lip, the stop post (-X), the rounded corner, a bolt hole
    assert not is_inside(cap, 0, -(r_seat - 0.5), zc) and is_inside(cap, 0, -(r_seat + 0.5), zc)
    assert is_inside(cap, 0, -(r_lip + 0.5), S["z_neck"] + 1.0) and not is_inside(cap, 0, -(r_lip - 0.5), S["z_neck"] + 1.0)
    assert is_inside(cap, -(D.stop_post_r[0] + D.stop_post_r[1]) / 2.0, 0, S["z_stop_post"] + 1.0)
    assert not is_inside(cap, D.block_x[1] - 1.0, D.block_y[0] + 1.0, zc) and is_inside(cap, D.block_x[1] - 3.0, D.block_y[0] + 6.0, zc)
    for x, y in cap_bolt_points(DEFAULT):
        assert not is_inside(cap, x, y, zc)


def _placed_module():
    return raw(M.forearm_roll_drive).moved(to_location(module_frame_in_host(DEFAULT)))


@pytest.mark.slow
def test_module_clears_its_neighbours_in_the_arm():
    """In j2_link's frame: the module against the forearm, the elbow pulley (PULLEY_SEAT_SHIFT below the block's stub
    end), the elbow bearings (the stub in the upper one, its shoulder on it: contact, no overlap) and the upper arm at
    the capture pose; the shaft's spigot in j2_link's recess likewise."""
    module = _placed_module()
    link = parts.build("j2_link")
    assert interference(module, link) < 1.0
    for key in ("gt2_pulley_90t#3", "bearing_6806#3", "bearing_6806#4", "j1_link#1", "nema17_40mm#2", "mks_servo42d#2",
                "nema17_40mm#3", "mks_servo42d#3"):
        vol = interference(module, in_host(key))
        assert vol < 1.0, f"module x {key}: {vol:.1f} mm^3"
    block = next(c for c in raw(M.forearm_roll_drive).children if c.label == "forearm_roll_block").moved(to_location(module_frame_in_host(DEFAULT)))
    assert math.isclose(block.bounding_box().min.Z, STUB_END_Z, abs_tol=1e-6)                            # the stub's end ...
    assert math.isclose(in_host("gt2_pulley_90t#3").bounding_box().max.Z, ELBOW_PULLEY_FACE_Z, abs_tol=0.05)   # ... the pulley below it
    shaft = next(c for c in raw(M.forearm_roll_drive).children if c.label == "forearm_roll_shaft").moved(to_location(module_frame_in_host(DEFAULT)))
    assert math.isclose(shaft.bounding_box().min.X, -S["z_spigot_end"], abs_tol=1e-6)
    assert interference(shaft, link) < 1.0


@pytest.mark.slow
@pytest.mark.parametrize("deg", [-PARAMS.ELBOW_PITCH_LIMIT_DEG, -60.0, 60.0, PARAMS.ELBOW_PITCH_LIMIT_DEG])
def test_module_clears_the_folded_upper_arm(deg):
    """The upper arm (j1_link, its motor + board) swung about the elbow axis (host z through the origin) by the
    elbow's limits never runs into the block, its motor, the board or the end cap."""
    module = _placed_module()
    axis = Axis((0.0, 0.0, 0.0), (0.0, 0.0, 1.0))
    for key in ("j1_link#1", "nema17_40mm#2", "mks_servo42d#2"):
        vol = interference(module, in_host(key).rotate(axis, deg))
        assert vol < 1.0, f"elbow {deg:+.0f} deg: module x {key}: {vol:.1f} mm^3"


@pytest.mark.slow
@pytest.mark.parametrize("deg", [-PARAMS.FOREARM_ROLL_LIMIT_DEG, -90.0, 90.0, PARAMS.FOREARM_ROLL_LIMIT_DEG])
def test_forearm_clears_the_elbow_while_rolling(deg):
    """j2_link rolled about the roll axis (host: through (0, 0, axis_z) along -X) to its limits never runs into the
    upper arm's round end, the elbow pulley or the stator's parts (the block, the end cap, the motor, the board)."""
    axis = Axis((0.0, 0.0, DEFAULT.roll_end.axis_z), (-1.0, 0.0, 0.0))
    link = parts.build("j2_link").rotate(axis, deg)
    for key in ("j1_link#1", "gt2_pulley_90t#3"):
        vol = interference(link, in_host(key))
        assert vol < 1.0, f"roll {deg:+.0f} deg: j2_link x {key}: {vol:.1f} mm^3"
    module = _placed_module()
    for label in ("forearm_roll_block", "forearm_roll_retainer", "nema17_40mm:forearm_roll", "mks_servo42d:forearm_roll"):
        vol = interference(link, next(c for c in module.children if c.label == label))
        assert vol < 1.0, f"roll {deg:+.0f} deg: j2_link x {label}: {vol:.1f} mm^3"
