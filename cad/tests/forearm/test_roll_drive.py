"""The forearm roll drive (assemblies/forearm_roll_drive.py): the joint's axis through the wrist centre and
ELBOW_ROLL_OFFSET across the elbow axis, the module frame, the frame's stack (the housing round the roll axis, the web
down to the elbow flange, the motor on the elbow axis), the parts' fits inside the module, and the module's clearances
in the arm (at the capture pose, with the elbow folded to its limits and with the forearm rolled through its range)."""
import functools
import math

import pytest
from build123d import Axis, Location, Vector
from cadgen.geometry import closest_points

from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import placements as P
from lib.belts import GT2_PULLEY_20T_TEETH, centre_distance
from lib.datum import to_location
from lib.forearm import (
    DEFAULT,
    belt_window,
    cap_bolt_points,
    end_nut_pocket,
    flange_bolt_points_module,
    module_frame_in_host,
    nut_channel_end,
    pulley_bolt_points,
    stack_positions,
)
from robot import frames as F
from tests import built
from tests.forearm.helpers import in_host
from tests.helpers import interference, is_inside, module_tints

S = stack_positions(DEFAULT)
D = DEFAULT.drive
UPPER_ARM_FACE_Z = -11.0         # host z of j1_link's +N face under the block: its flat top, the relief's floor (1 under the lip's root) ...
UPPER_ARM_RECESS = (40.0, -16.0)  # j1_link's recess round the elbow axis: radius, floor (host z)
FACE_GAP = 2.0                   # the least axial gap between a face the elbow turns and j1_link's (printed PETG; the
#                                  forearm's moment tilts the block on its 6806 pair)
UPPER_ARM_BORE_R = 21.0          # j1_link's bore the coupler's stub / journal turn in
UPPER_ARM_END_R = 45.0           # j1_link's round end about the elbow axis
ELBOW_PULLEY_FACE_Z = -22.0 - PARAMS.PULLEY_SEAT_SHIFT   # host z of the elbow 90T's mating face: the SolidWorks coupler's stub end
#                                                          (-22), both moved on to the re-seated pulley on j1_link's lip
ELBOW_AXIS_HOST = (0.0, -DEFAULT.elbow_offset)            # the elbow axis (along host Z) in j2_link's frame: the forearm sits ELBOW_ROLL_OFFSET across it


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
    # the elbow offset: the roll axis the roll belt's centre distance from the elbow axis - the motor sits on the elbow axis
    assert DEFAULT.elbow_offset == PARAMS.ELBOW_ROLL_OFFSET == D.centre_distance == -D.elbow_y == -S["y_elbow"] > 0.0
    assert _offset_from_line(F.ELBOW_ORIGIN, tuple(world.position), F.N) == pytest.approx(DEFAULT.elbow_offset, abs=0.05)


def test_the_elbow_coupler_is_retired_for_the_block():
    """j3_coupler#1's record stays in placements.json but nothing claims it; the wrist's j3_coupler#2 is untouched."""
    assert P.RETIRED[0] == "j3_coupler#1" and "j3_coupler#1" in P.OCCURRENCES
    assert "j3_coupler#1" not in P.keys() and "j3_coupler#1" in P.keys(retired=True) and "j3_coupler#2" in P.keys()
    assert F.LINKS["elbow_link"] == ["gt2_pulley_90t#3", "elbow_pulley_screws#1", "elbow_pulley_nuts#1",
                                     "forearm_roll_drive#1:stator"]   # the re-seated elbow 90T + its bolts (lib/mounts.py)


def test_stack():
    w, z_axis = DEFAULT.roll_end, DEFAULT.roll_end.axis_z
    # the frame: its underside (the housing's and the web's, one plane) and the cap's FACE_GAP clear of the upper arm's
    # flat top (j1_link's slab, level end to end: tests/upper_arm), the coupler features below the web about the elbow
    # axis in j1_link's recess and bore where the SolidWorks coupler's were, an inner-ring shoulder between the journal
    # and the stub, the stub on through the lip to the elbow pulley's face (the Ø12.5 bore open at its end)
    assert z_axis + D.block_x[0] >= UPPER_ARM_FACE_Z + FACE_GAP and S["x_cradle"] - D.web_t == D.block_x[0]
    assert D.lip_dia / 2.0 <= UPPER_ARM_RECESS[0] - 1.0 and D.boss_dia / 2.0 <= UPPER_ARM_RECESS[0] - 2.0
    assert D.lip_x[1] == D.block_x[0] and D.boss_x[1] == D.lip_x[0] and D.journal_x[1] == D.boss_x[0]
    assert D.step_x[1] == D.journal_x[0] and D.stub_x[1] == D.step_x[0] and D.stub_dia < D.step_dia < D.journal_dia
    assert z_axis + D.boss_x[0] >= UPPER_ARM_RECESS[1] + FACE_GAP and D.journal_dia / 2.0 < UPPER_ARM_BORE_R and D.stub_dia / 2.0 < UPPER_ARM_BORE_R
    assert z_axis + D.stub_x[0] == ELBOW_PULLEY_FACE_Z and D.pin_bore_x[0] == D.stub_x[0]
    # the stations, rear end wall -> the forearm wall
    assert S["z_end"] == D.block_z[0] and S["z_lip"] == S["z_end"] + D.end_wall and S["z_seat"] == S["z_lip"] + D.lip == S["z_bearing_1"]
    assert S["z_bore"] == S["z_seat"] + D.bearing_width and S["z_bore"] < S["z_cavity"] == D.cavity_z0 < S["z_ring_flange_1"]
    assert S["z_ring"] == D.ring_z0 and S["z_ring_end"] + D.belt_window_margin < S["z_face"] == D.block_z[1] == S["z_cap"] == S["z_bearing_2"]
    assert S["z_neck"] == S["z_face"] + D.bearing_width and S["z_cap_outer"] == S["z_neck"] + D.cap_lip == S["z_stop_post"] == S["z_stop_lug"]
    assert S["z_stop_post"] + D.stop_t + 0.5 <= S["z_wall"] == -w.wall_x[1] and S["z_spigot_end"] - S["z_wall"] == w.flange_recess_depth
    assert S["z_shaft_end"] == S["z_lip"] + D.shaft_end_clear < S["z_seat"]
    assert math.isclose(S["z_ring_mid"] - S["z_motor_face"], D.t20) and S["z_20t"] - S["z_pad_top"] == D.pulley_lift
    # the elbow axis runs elbow_offset under the roll axis (the motor sits on it): the bearings straddle its station
    assert S["z_end"] < S["z_bearing_1"] and S["z_bearing_1"] + D.bearing_width < 0 < S["z_bearing_2"] <= S["z_face"]
    # the web: from the housing down past the lip about the elbow axis, its whole width along the roll axis
    assert D.web_z[0] <= -D.lip_dia / 2.0 and D.web_z[1] >= D.lip_dia / 2.0 and D.web_z[1] == D.block_z[1]
    assert S["y_web_low"] == S["y_elbow"] - D.lip_dia / 2.0 - D.web_margin and S["y_elbow"] + D.lip_dia / 2.0 > D.block_y[0]
    # the forearm wall: the rolling forearm, whatever its roll angle, clears j1_link's round end
    assert S["z_wall"] >= UPPER_ARM_END_R + 1.5
    # walls: 2 mm under the cavity and the seat
    assert -D.block_x[0] - D.cavity_dia / 2.0 >= 2.0 and -D.block_x[0] - (D.bearing_od + D.seat_add) / 2.0 >= 2.0
    # the elbow 90T's nuts: seated in the boss, the M4 from the pulley's counterbores 2 pitches past its nut, nut and tip
    # 2 mm under the motor's cradle (the web's top); each hex channel open up into the cradle (the nuts go in from there
    # before the motor), inside the web, and clear of the pin bore, which ends under the seats
    corner = D.nut_af / math.sqrt(3.0)
    tip = D.stub_x[0] - D.pulley_hub_len + PARAMS.GT2_PULLEY_90T_HEAD_SEAT + D.pulley_screw_len
    assert D.boss_x[0] < D.nut_seat_x < D.boss_x[1] and D.pulley_bolt_r + corner <= D.boss_dia / 2.0 - 2.0
    assert tip - (D.nut_seat_x + D.nut_t) >= 2 * 0.7 - 1e-9 and tip <= S["x_cradle"] - 2.0
    assert D.nut_seat_x + D.nut_t <= S["x_cradle"] - 2.0 and D.pin_bore_x[1] == D.nut_seat_x - 1.0
    assert nut_channel_end(DEFAULT) == S["x_cradle"] + D.nut_channel_past
    assert all(D.web_z[0] < z - corner and z + corner < D.web_z[1] for _, z in pulley_bolt_points(DEFAULT))
    assert D.pulley_bolt_r + D.pulley_bolt_dia / 2.0 <= D.stub_dia / 2.0 - 1.5   # the SolidWorks coupler's own wall round them
    # bearing 2 slides on from the wrist end: everything beyond journal 2 is smaller than its bore; the ring passes the open cavity
    assert D.shoulder_od < D.inner_race_od < D.bearing_od and D.neck_od < D.bearing_bore and w.flange_dia < D.bearing_bore
    assert D.cavity_dia >= D.ring_flange_dia + 2.0 and D.lip_id > D.bearing_bore + D.journal_add
    assert D.bearing_od + 0.3 <= D.core_bore_dia < D.cavity_dia      # bearing 1 rides through the core bore to its seat from the front
    # the motor: ON the elbow axis under the housing, centred on the roll axis in X, the belt setting the offset; its
    # body 1 mm or more under the housing's bottom at the slot's top end and 2 mm over the web (the cradle's floor); the
    # plate in front of it past its bolt square and as wide as it + 2
    assert S["x_motor"] == 0.0 and S["y_motor"] == S["y_elbow"] == D.elbow_y
    assert math.isclose(D.centre_distance, centre_distance(D.roll_belt, D.ring_teeth, GT2_PULLEY_20T_TEETH), abs_tol=1e-9)
    assert S["y_motor"] + D.pad_slot_len / 2.0 + D.motor.body_width / 2.0 <= D.block_y[0] - 1.0
    assert -D.motor.body_width / 2.0 - S["x_cradle"] >= 2.0
    assert S["y_plate_low"] <= S["y_motor"] - 15.5 - D.pad_bolt_dia / 2.0 - D.pad_slot_len / 2.0 - 2.0 and D.plate_w >= D.motor.body_width + 2.0
    # the belt window: the ring's width + the margins, from inside the cavity down out through the bottom wall
    z0, z1, half_x, y0 = belt_window(DEFAULT)
    assert z0 == S["z_ring_flange_1"] - D.belt_window_margin and z1 == S["z_ring_end"] + D.belt_window_margin and z1 < S["z_face"]
    assert half_x < D.ring_flange_dia / 2.0 and D.block_y[0] < -D.cavity_dia / 2.0 < y0 == -D.belt_window_y0   # a slot for the runs, not the ring
    assert all(abs(x) - D.cap_tap_dia / 2.0 >= half_x + 2.0 for x, _ in cap_bolt_points(DEFAULT))   # the cap's taps beside the window
    assert D.motor_spin_deg == 90.0                                                               # the connector toward +X, away from the upper arm
    # the wall's screws into nuts in the shaft: each clearance hole centred in the neck's wall (between the bore and the
    # neck's outside), the head on the wall's wrist face, the tip 2 pitches past its nut; the nut behind bearing 2 in the
    # Ø44 core, clear of the ring, 3 mm of the core outside it, its pocket open into the bore - so head and nut clamp the
    # spigot, the neck and journal 2 between them
    rc, hole_r, corner = w.bolt_circle_dia / 2.0, w.bolt_dia / 2.0, D.end_nut.af / math.sqrt(3.0)
    assert D.bore == w.cable_bore and D.cable_exit_dia > D.bore and w.bolt_dia == PARAMS.M3_CLEAR and w.screw.d == D.end_nut.d
    assert math.isclose(rc - hole_r - D.bore / 2.0, D.neck_od / 2.0 - rc - hole_r) and D.neck_od / 2.0 - rc - hole_r >= 1.5
    assert S["z_wall_back"] == -w.wall_x[0] and S["z_end_tip"] == S["z_wall_back"] - w.screw_len
    assert math.isclose(S["z_end_nut"] - S["z_end_tip"], 2 * PARAMS.M3_PITCH) and math.isclose(S["z_end_nut_seat"] - S["z_end_nut"], D.end_nut.h)
    assert S["z_ring_end"] + 2.0 <= S["z_end_nut"] - D.end_nut_fit and S["z_end_nut_seat"] <= S["z_bearing_2"] - 1.5
    r_in, r_out, z0, z1 = end_nut_pocket(DEFAULT)
    assert r_in < D.bore / 2.0 < rc - corner and r_out == rc + corner + D.end_nut_fit and D.shoulder_od / 2.0 - r_out >= 3.0
    assert (z0, z1) == (S["z_end_nut"] - D.end_nut_fit, S["z_end_nut_seat"]) and S["z_wall_back"] - S["z_end_nut_seat"] >= 20.0
    assert D.stop_deg == PARAMS.FOREARM_ROLL_LIMIT_DEG == 180.0 - D.stop_deg_width
    assert D.stop_lug_r[1] > D.stop_post_r[0] and D.stop_lug_r[0] < D.neck_od / 2.0 < D.stop_post_r[0]
    # the cap's four bolts: inside the outline's rounded corners, outside the cavity
    for x, y in cap_bolt_points(DEFAULT):
        assert math.hypot(x, y) > D.cavity_dia / 2.0 + D.cap_tap_dia and abs(x) < D.block_x[1] and D.block_y[0] < y < D.block_y[1]
    assert len(pulley_bolt_points(DEFAULT)) == 4 and all(math.isclose(math.hypot(*yz), D.pulley_bolt_r) for yz in pulley_bolt_points(DEFAULT))


@pytest.fixture(scope="module")
def module():
    return built.model(M.forearm_roll_drive)


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


@pytest.mark.slow
def test_totals_and_bodies(module):
    got = M.totals(shape=module)
    assert (got["leaves"], got["solids"]) == (M.EXPECTED["leaves"], M.EXPECTED["solids"])
    assert abs(got["solid_volume"] - M.EXPECTED["solid_volume"]) < 0.5
    for body in M.BODIES:
        got = M.totals(body, shape=module)
        assert (got["leaves"], got["solids"]) == (M.EXPECTED["bodies"][body]["leaves"], M.EXPECTED["bodies"][body]["solids"])
        assert abs(got["solid_volume"] - M.EXPECTED["bodies"][body]["solid_volume"]) < 0.5


@pytest.mark.slow
def test_module_colors(module):
    """Standalone: the purchased parts BOUGHT_TINT, the printed ones the module's TINT."""
    assert module.label == "forearm_roll_drive"
    # 2 bearings, motor, board, 20T / frame, shaft, retainer
    assert module_tints(module, M.TINT) == {True: 5, False: 3}


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
    assert not is_inside(block, 0, -(r_cav + 1.5), zm) and is_inside(block, 0, -(r_cav + 1.5), z1 + 2.0)    # the belt window, the bottom wall past it
    assert is_inside(block, 0, r_cav + 1.5, zm)                                                              # the top wall, closed
    # the front face is open (the cavity reaches it), the cap's tap holes in it
    assert not is_inside(block, 0, r_cav - 1.0, S["z_face"] - 0.5)
    for x, y in cap_bolt_points(DEFAULT):
        assert not is_inside(block, x, y, S["z_face"] - 4.0) and is_inside(block, x, y, S["z_face"] - D.cap_tap_depth - 2.0)
    # the elbow flange about the elbow axis (y_elbow): the lip inside its radius only, the boss, the stub, the pin bore,
    # the pulley bolts + nut channels up through the web into the motor's cradle
    ye = S["y_elbow"]
    assert is_inside(block, D.lip_x[0] + 1.0, ye, 20.0) and not is_inside(block, D.lip_x[0] + 1.0, ye, D.lip_dia / 2.0 + 2.0)   # (ye, 0) is the pin bore
    assert is_inside(block, D.boss_x[0] + 1.0, ye, D.boss_dia / 2.0 - 2.0) and not is_inside(block, D.boss_x[0] + 1.0, ye, D.boss_dia / 2.0 + 1.0)
    assert is_inside(block, D.stub_x[0] + 2.0, ye, D.stub_dia / 2.0 - 1.0) and not is_inside(block, D.stub_x[0] + 2.0, ye, D.stub_dia / 2.0 + 1.0)
    assert not is_inside(block, D.stub_x[0] + 2.0, ye, 0) and not is_inside(block, D.pin_bore_x[1] - 0.5, ye, 0) and is_inside(block, D.pin_bore_x[1] + 0.5, ye, 0)
    flat, corner = D.nut_af / 2.0, D.nut_af / math.sqrt(3.0)
    for y0_, z in pulley_bolt_points(DEFAULT):
        y = ye + y0_
        u = (y0_ / D.pulley_bolt_r, z / D.pulley_bolt_r)            # radial (a flat faces the axis), t tangential (a corner)
        t = (-u[1], u[0])
        assert not is_inside(block, D.stub_x[0] + 2.0, y, z) and not is_inside(block, D.nut_seat_x - 1.0, y, z)   # the clearance hole
        assert is_inside(block, D.nut_seat_x - 0.5, y + 2.8 * u[0], z + 2.8 * u[1])                            # the seat beside it
        above = D.nut_seat_x + 1.0
        assert not is_inside(block, above, y, z) and not is_inside(block, S["x_cradle"] - 0.5, y, z)          # on up through the web
        assert not is_inside(block, above, y - (flat - 0.2) * u[0], z - (flat - 0.2) * u[1])       # the hex, a flat toward the axis
        assert is_inside(block, above, y - (flat + 0.3) * u[0], z - (flat + 0.3) * u[1])
        assert not is_inside(block, above, y + (corner - 0.2) * t[0], z + (corner - 0.2) * t[1])
        assert is_inside(block, above, y + (corner + 0.3) * t[0], z + (corner + 0.3) * t[1])
    # the motor's cradle: the web under it (on the upper arm's side), the plate in front (its pilot slot, the motor's
    # space behind it), the housing's bottom over it, nothing beside it on the +N side
    ym = S["y_motor"]
    assert is_inside(block, S["x_cradle"] - 1.0, ym, -10.0) and not is_inside(block, S["x_cradle"] + 1.0, ym, -10.0)
    assert not is_inside(block, 0, ym, -10.0) and not is_inside(block, D.motor.body_width / 2.0 + 2.0, ym, -10.0)
    assert is_inside(block, 10, ym - 10, S["z_motor_face"] + 1.5) and not is_inside(block, 0, ym, S["z_motor_face"] + 1.5)
    assert not is_inside(block, 10, ym - 10, S["z_motor_face"] - 1.5) and not is_inside(block, 10, ym - 10, S["z_pad_top"] + 1.0)
    assert is_inside(block, 0, D.block_y[0] + 1.0, -10.0) and not is_inside(block, 0, D.block_y[0] - 1.0, -10.0)
    # the shaft: the bore, the core, journal 1, the ring (a land, a groove, all round), the neck, the lug, the spigot, the
    # wall screws' holes and nut pockets
    assert not is_inside(shaft, 0, 0, zm) and is_inside(shaft, D.bore / 2.0 + 1.0, 0, zm)
    assert is_inside(shaft, D.bearing_bore / 2.0 - 0.5, 0, S["z_seat"] + 3.0) and not is_inside(shaft, D.bearing_bore / 2.0 + 1.0, 0, S["z_seat"] + 3.0)
    assert is_inside(shaft, 27.0, 0, zm) and not is_inside(shaft, 28.3, 0, zm) and is_inside(shaft, 0, 27.5, zm)
    assert is_inside(shaft, 0, D.neck_od / 2.0 - 1.0, S["z_stop_lug"] + 1.0) and not is_inside(shaft, 0, D.neck_od / 2.0 + 1.0, S["z_stop_lug"] + 1.0)
    assert is_inside(shaft, (D.stop_lug_r[0] + D.stop_lug_r[1]) / 2.0, 0, S["z_stop_lug"] + 1.0)
    assert is_inside(shaft, DEFAULT.roll_end.flange_dia / 2.0 - 0.5, 0, S["z_wall"] + 1.0)
    _r_in, r_out, z0, z1 = end_nut_pocket(DEFAULT)
    zn, hole_r, flat = (z0 + z1) / 2.0, DEFAULT.roll_end.bolt_dia / 2.0, D.end_nut.af / 2.0
    for x, y in flange_bolt_points_module(DEFAULT):
        rc = math.hypot(x, y)
        u, t = (x / rc, y / rc), (-y / rc, x / rc)                                 # radial, tangential
        assert not is_inside(shaft, x, y, S["z_wall"] + 1.0) and not is_inside(shaft, x, y, S["z_neck"] + 1.0)   # the hole ...
        assert not is_inside(shaft, x, y, S["z_end_tip"] - D.end_nut_fit + 0.3)                                  # ... to the tip
        assert is_inside(shaft, x, y, S["z_end_tip"] - D.end_nut_fit - 0.3)
        assert is_inside(shaft, x + (hole_r + 0.5) * t[0], y + (hole_r + 0.5) * t[1], S["z_neck"] + 1.0)
        assert is_inside(shaft, x + (hole_r + 0.5) * t[0], y + (hole_r + 0.5) * t[1], z1 + 0.3)              # the nut's seat
        assert not is_inside(shaft, x + (flat - 0.2) * t[0], y + (flat - 0.2) * t[1], zn)                    # the pocket: a flat
        assert is_inside(shaft, x + (flat + 0.3) * t[0], y + (flat + 0.3) * t[1], zn)                        # either side, ...
        assert not is_inside(shaft, (r_out - 0.2) * u[0], (r_out - 0.2) * u[1], zn)                          # ... past the nut's corner,
        assert is_inside(shaft, (r_out + 0.3) * u[0], (r_out + 0.3) * u[1], zn)
        assert not is_inside(shaft, (D.bore / 2.0 + 0.3) * u[0], (D.bore / 2.0 + 0.3) * u[1], zn)          # ... open into the bore
    bb = shaft.bounding_box()
    assert math.isclose(bb.max.Z, S["z_spigot_end"], abs_tol=1e-6) and math.isclose(bb.min.Z, S["z_shaft_end"], abs_tol=1e-6)
    # the cap: seat 2, the lip, the stop post (-X), the rounded corner, a bolt hole
    assert not is_inside(cap, 0, -(r_seat - 0.5), zc) and is_inside(cap, 0, -(r_seat + 0.5), zc)
    assert is_inside(cap, 0, -(r_lip + 0.5), S["z_neck"] + 1.0) and not is_inside(cap, 0, -(r_lip - 0.5), S["z_neck"] + 1.0)
    assert is_inside(cap, -(D.stop_post_r[0] + D.stop_post_r[1]) / 2.0, 0, S["z_stop_post"] + 1.0)
    assert not is_inside(cap, D.block_x[1] - 1.0, D.block_y[0] + 1.0, zc) and is_inside(cap, D.block_x[1] - 3.0, D.block_y[0] + 6.0, zc)
    for x, y in cap_bolt_points(DEFAULT):
        assert not is_inside(cap, x, y, zc)


@functools.cache
def _placed_module():
    """The module in j2_link's frame (tests/built.py: shared, read-only)."""
    return built.model(M.forearm_roll_drive).moved(to_location(module_frame_in_host(DEFAULT)))


@functools.cache
def _placed_part(label: str):
    """One of the module's parts in j2_link's frame (shared, read-only). A child of _placed_module() stays in the MODULE
    frame (moving the compound moves only the compound), so each part is moved itself."""
    return built.leaf(M.forearm_roll_drive, label).moved(to_location(module_frame_in_host(DEFAULT)))


def _upper_arm_at(shape, elbow: float):
    """A part of the upper arm (in this frame at the capture pose) with the elbow at `elbow` degrees (elbow_pitch's
    sign): this frame turns with the forearm, so the upper arm turns the other way about the elbow axis (along Z,
    ELBOW_ROLL_OFFSET under the forearm: ELBOW_AXIS_HOST)."""
    return shape.rotate(Axis((*ELBOW_AXIS_HOST, 0.0), (0.0, 0.0, 1.0)), -elbow)


def _elbow_range(step: int = 10) -> list[float]:
    """The elbow's range every `step` degrees from its lower limit, and the upper limit."""
    lo, hi = PARAMS.ELBOW_PITCH_LIMITS_DEG
    return sorted({*(lo + step * i for i in range(int((hi - lo) // step) + 1)), hi})


@pytest.mark.slow
def test_module_clears_its_neighbours_in_the_arm():
    """In j2_link's frame, at the capture pose: the module against the forearm and the upper arm - the stub through its
    lip -; the block's stub end on the elbow pulley's face (contact, no overlap), the shaft's spigot in j2_link's recess.
    (The mounted motors, boards, bearings and pulleys against the whole module: tests/test_mounts.py
    test_motors_and_boards_clear_their_neighbours.) The upper arm's own motor and board - on its +N side like the
    module's, ELBOW_MOTOR_CENTRES from the elbow axis - turned about it over the elbow's range stay 1 mm off the whole
    module: the lower limit, the forearm lifted back, is set 3 deg short of where the roll housing's rear end comes that
    near (lib/params.py ELBOW_PITCH_LIMITS_DEG); the roll motor itself, on the elbow axis, never reaches it."""
    module = _placed_module()
    for key, other in (("j2_link", built.part("j2_link")), ("j1_link#1", in_host("j1_link#1"))):
        vol = interference(module, other)
        assert vol < 1.0, f"module x {key}: {vol:.1f} mm^3"
    block = _placed_part("forearm_roll_block")
    assert math.isclose(block.bounding_box().min.Z, ELBOW_PULLEY_FACE_Z, abs_tol=1e-6)                   # the stub's end on the pulley
    assert math.isclose(in_host("gt2_pulley_90t#3").bounding_box().max.Z, ELBOW_PULLEY_FACE_Z, abs_tol=0.05)
    shaft = _placed_part("forearm_roll_shaft")
    assert math.isclose(shaft.bounding_box().min.X, -S["z_spigot_end"], abs_tol=1e-6)
    for key in ("nema17_40mm#2", "mks_servo42d#2"):
        for elbow in _elbow_range(5):
            gap = _upper_arm_at(in_host(key), elbow).distance_to(module)
            assert gap > 1.0, f"{key} at elbow {elbow:+.0f} deg: {gap:.1f} mm from the module"


@pytest.mark.slow
@pytest.mark.parametrize("deg", PARAMS.ELBOW_PITCH_LIMITS_DEG)
def test_module_clears_the_folded_upper_arm(deg):
    """The upper arm (j1_link) swung about the elbow axis (along host z, ELBOW_AXIS_HOST) to the elbow's limits never
    runs into the frame, its motor, the board or the end cap - and never comes nearer what turns over it than designed:
    the end cap's underside (level with the frame's) no nearer than its designed height above j1_link's flat top, the
    frame nowhere nearer than its journal's edge to the bore's rim (a radial gap: the bearings set it). The frame and the
    cap turn over j1_link's flat top, round its bore, so the two limits stand for the angles between; the capture pose
    is test_module_clears_its_neighbours_in_the_arm's. Widen the sample if the top stops being flat under them."""
    module = _placed_module()
    upper_arm = _upper_arm_at(in_host("j1_link#1"), deg)
    vol = interference(module, upper_arm)
    assert vol < 1.0, f"elbow {deg:+.0f} deg: module x j1_link#1: {vol:.1f} mm^3"
    cap_gap = closest_points(_placed_part("forearm_roll_retainer"), upper_arm).distance
    assert cap_gap >= DEFAULT.roll_end.axis_z + D.block_x[0] - UPPER_ARM_FACE_Z - 0.01 and cap_gap >= FACE_GAP
    journal_edge = math.hypot(UPPER_ARM_BORE_R - D.journal_dia / 2.0, DEFAULT.roll_end.axis_z + D.journal_x[0] - UPPER_ARM_RECESS[1])
    block_gap = closest_points(_placed_part("forearm_roll_block"), upper_arm).distance
    assert block_gap == pytest.approx(journal_edge, abs=0.01) and block_gap >= 1.0


@pytest.mark.slow
def test_forearm_clears_the_elbow_while_rolling():
    """j2_link rolls about the roll axis (host: through (0, 0, axis_z) along -X), which keeps every point's x: the whole
    link ends at its wall's elbow face, and the upper arm's round end, the elbow pulley and the stator's parts (the
    frame, the end cap, the motor, the board) all lie beyond that plane, so no roll angle brings them together.
    Anything of j2_link past the wall toward the elbow would need a sweep over the roll angles instead."""
    plane = built.part("j2_link").bounding_box().max.X
    assert plane == pytest.approx(DEFAULT.roll_end.wall_x[1], abs=1e-6), f"j2_link reaches x {plane:.2f}, past its wall"
    others = {key: in_host(key) for key in ("j1_link#1", "gt2_pulley_90t#3")}
    others |= {label: _placed_part(label) for label in ("forearm_roll_block", "forearm_roll_retainer",
                                                        "nema17_40mm:forearm_roll", "mks_servo42d:forearm_roll")}
    for key, other in others.items():
        x = other.bounding_box().min.X
        assert x >= plane, f"{key} reaches x {x:.2f}, into the rolling forearm's side of x {plane:.2f}"


@pytest.mark.slow
@pytest.mark.parametrize("elbow", PARAMS.ELBOW_PITCH_LIMITS_DEG)
@pytest.mark.parametrize("roll", [-PARAMS.FOREARM_ROLL_LIMIT_DEG, -90.0, 0.0, 90.0, PARAMS.FOREARM_ROLL_LIMIT_DEG])
def test_forearm_clears_the_upper_arm_at_the_elbow_limits(elbow, roll):
    """The elbow folded to its limits, the forearm rolled anywhere: j2_link (its round wall 48 along the roll axis from
    the elbow axis' station and ELBOW_ROLL_OFFSET across it, its necked web) stays 2 mm off the upper arm beside it."""
    link = built.part("j2_link").rotate(Axis((0.0, 0.0, DEFAULT.roll_end.axis_z), (-1.0, 0.0, 0.0)), roll)
    upper_arm = _upper_arm_at(in_host("j1_link#1"), elbow)
    gap = closest_points(link, upper_arm).distance
    assert gap >= 2.0, f"elbow {elbow:+.0f} deg, roll {roll:+.0f} deg: j2_link {gap:.2f} mm off j1_link"
