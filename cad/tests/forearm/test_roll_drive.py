"""The forearm roll drive (assemblies/forearm_roll_drive.py): the joint's axis across the wrist_pitch axis (WEB_DROP from
the wrist centre) and
ELBOW_ROLL_OFFSET across the elbow axis, the module frame, the stack (the frame round the motor - an octagon about the elbow
axis, the tower round the roll axis, the side face closed, the pocket open through the bottom -, the 6806 pair a spacer
apart under the pulley's ring, the rotor split at bearing 1 and clamped, the motor on the elbow axis, its board's side
plugs in grooves), the parts' fits inside the module, the motor's way in from the bottom, and the module's clearances in
the arm (at the capture pose, with the elbow folded to its limits and with the forearm rolled through its range)."""
import copy
import functools
import math

import pytest
from build123d import Align, Axis, Box, Location, Vector
from cadgen.geometry import closest_points

from assemblies import forearm_roll_drive as M
from lib import params as PARAMS
from lib import placements as P
from lib.belts import GT2_PULLEY_20T_TEETH, STANDARD_2GT_LENGTHS, closed_belt_length
from lib.datum import to_location
from lib.forearm import (
    DEFAULT,
    WEB_DROP,
    clamp_nut_pocket,
    clamp_points,
    end_nut_pocket,
    flange_bolt_points_module,
    module_frame_in_host,
    motor_pocket,
    nut_channel_end,
    pad_bolt_points,
    plug_slots,
    pulley_bolt_points,
    side_headers,
    side_plug_room,
    stack_positions,
)
from lib.forearm.roll import rear_line
from robot import frames as F
from tests import built
from tests.forearm.helpers import in_drive_frame, j2_link_in_drive_frame
from tests.helpers import interference, is_inside, module_tints

S = stack_positions(DEFAULT)
D = DEFAULT.drive
UPPER_ARM_FACE_Z = -16.0         # host z of j1_link's +N face under the block: its flat top, the relief's floor - down at the
#                                  floor of the old recess round the elbow axis, level with the upper 6806's top (no recess left)
FACE_GAP = 2.0                   # the least axial gap between a face the elbow turns and j1_link's (printed PETG; the
#                                  forearm's moment tilts the block on its 6806 pair)
UPPER_ARM_BORE_R = 21.0          # j1_link's bore the coupler's stub / journal turn in
UPPER_ARM_END_R = 36.0           # j1_link's round end about the elbow axis (lib/upper_arm/params.py ArmParams.end_r)
ELBOW_PULLEY_FACE_Z = -22.0 - PARAMS.PULLEY_SEAT_SHIFT   # host z of the elbow 90T's mating face: the SolidWorks coupler's stub end
#                                                          (-22), both moved on to the re-seated pulley on j1_link's lip
ELBOW_AXIS_HOST = (0.0, -DEFAULT.elbow_offset)            # the elbow axis (along host Z) in j2_link's frame before the forearm's turn (the
#                                                           drive's mount frame, tests/forearm/helpers.py): the forearm sits ELBOW_ROLL_OFFSET across it
RUN_GAP = 1.0                    # the least gap between the turning rotor and the frame (the ring over the bearing boss, the shaft in the bay)
BELT_W = 6.0                     # the roll belt's width, centred on the ring's teeth


def _eq(*values) -> bool:
    """The stations are sums of the params: equal to float noise."""
    return all(math.isclose(v, values[0], abs_tol=1e-9) for v in values)


def _offset_from_line(point, origin, axis) -> float:
    d = [p - o for p, o in zip(point, origin, strict=True)]
    t = sum(x * y for x, y in zip(d, axis, strict=True))
    return math.sqrt(sum((x - t * y) ** 2 for x, y in zip(d, axis, strict=True)))


def _line_gap(o1, a1, o2, a2) -> float:
    """The distance between two lines (point, unit direction): 0 where they cross."""
    n = (a1[1] * a2[2] - a1[2] * a2[1], a1[2] * a2[0] - a1[0] * a2[2], a1[0] * a2[1] - a1[1] * a2[0])
    return abs(sum((p - q) * c for p, q, c in zip(o2, o1, n, strict=True))) / math.sqrt(sum(c * c for c in n))


def test_roll_axis_crosses_the_pitch_axis_web_drop_from_the_wrist_centre():
    """The forearm roll crosses the wrist_pitch axis; the wrist dropped along that axis with the web (WEB_DROP, toward the
    belt side), so it crosses it that far from the wrist centre (the wrist's own station, where the SolidWorks wrist_roll
    axis met it). The wrist_roll axis meets neither - the wrist roll drive puts it WRIST_ROLL_OFFSET across
    (tests/wrist/test_roll_drive.py), so the wrist is not spherical."""
    assert _line_gap(F.FOREARM_ROLL_ORIGIN, F.FOREARM_ROLL_AXIS, F.WRIST_PITCH_ORIGIN, F.WRIST_PITCH_AXIS) < 0.05
    assert _offset_from_line(F.WRIST_CENTRE, F.FOREARM_ROLL_ORIGIN, F.FOREARM_ROLL_AXIS) == pytest.approx(WEB_DROP, abs=0.05)
    assert _offset_from_line(F.WRIST_CENTRE, F.WRIST_PITCH_ORIGIN, F.WRIST_PITCH_AXIS) < 0.05
    assert _offset_from_line(F.WRIST_CENTRE, F.WRIST_ROLL_ORIGIN, F.WRIST_ROLL_AXIS) > 40.0
    # the forearm turned half a turn about the roll axis (lib/placements.py TURNS): the wrist_pitch axis is the capture's N
    # turned with it - reversed -, the roll axis still crossing it
    assert PARAMS.FOREARM_ROLL_TURN_DEG == 180.0 and sum(x * y for x, y in zip(F.WRIST_PITCH_AXIS, F.N, strict=True)) < -1.0 + 1e-9
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
    # the elbow offset: the roll axis the drive's centre distance from the elbow axis - the motor sits on the elbow axis
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
    # the frame: its underside (one plane) FACE_GAP clear of the upper arm's flat top (j1_link's slab, level end to end:
    # tests/upper_arm), an octagon about the elbow axis, its flats at j1_link's old round end (the plate's), the
    # slimmed arm's narrower end inside them; the coupler features below it about the elbow axis over j1_link's flat
    # top and in its bore where the SolidWorks coupler's were - the lip and the boss FACE_GAP or more over that top -,
    # an inner-ring shoulder between the journal and the stub, the stub on through the lip to the elbow pulley's face
    # (the Ø12.5 bore open at its end)
    assert z_axis + D.block_x[0] >= UPPER_ARM_FACE_Z + FACE_GAP and S["x_cradle"] - D.web_t == D.block_x[0]
    assert D.end_r >= UPPER_ARM_END_R and D.lip_dia / 2.0 <= D.end_r - 5.0
    assert D.lip_x[1] == D.block_x[0] and D.boss_x[1] == D.lip_x[0] and D.journal_x[1] == D.boss_x[0]
    assert D.step_x[1] == D.journal_x[0] and D.stub_x[1] == D.step_x[0] and D.stub_dia < D.step_dia < D.journal_dia
    assert z_axis + D.boss_x[0] >= UPPER_ARM_FACE_Z + FACE_GAP and D.journal_dia / 2.0 < UPPER_ARM_BORE_R and D.stub_dia / 2.0 < UPPER_ARM_BORE_R
    assert z_axis + D.stub_x[0] == ELBOW_PULLEY_FACE_Z and D.pin_bore_x[0] == D.stub_x[0]
    # the stations, the forearm wall back: the ring's front face on the wall, the ring, bearing 2 RIGHT UNDER it (its
    # front face the bearing boss's top, run_gap under the ring's flange), the lip - the spacer between the outer rings,
    # the pair 19 apart centre to centre -, bearing 1; the pulley's neck meets the shaft's journal at bearing 1's front face
    assert _eq(S["z_ring_end"], S["z_wall"], -w.wall_x[1]) and _eq(S["z_spigot_end"] - S["z_wall"], w.flange_recess_depth)
    assert _eq(S["z_ring"], S["z_wall"] - D.ring_flange_t - D.ring_width) and _eq(S["z_ring_flange_1"], S["z_ring"] - D.ring_flange_t)
    assert _eq(S["z_boss"], S["z_ring_flange_1"] - D.run_gap, S["z_bearing_2"] + D.bearing_width)
    assert _eq(S["z_lip"], S["z_bearing_2"] - D.lip) and _eq(S["z_bearing_1"], S["z_lip"] - D.bearing_width)
    assert _eq(S["z_bearing_2"] - S["z_bearing_1"], D.bearing_width + D.lip, 19.0)
    assert _eq(S["z_meet"], S["z_lip"], S["z_bearing_1"] + D.bearing_width) and _eq(S["z_shoulder_1"], S["z_bearing_1"] - D.run_gap)
    assert S["z_wall"] >= UPPER_ARM_END_R + 1.5                 # the rolling forearm, whatever its roll angle, clears j1_link's round end
    # the bearing boss: bearing 2's seat in it, 4 mm of wall or more; its top bearing 2's front face, run_gap under the
    # ring's rear flange, which reaches past its edge (no cup round the flange), the belt's edge clear above it; centred
    # on the frame - the underside's plane, the side face and the tower's top all ROLL_FRAME_R off the roll axis, the
    # forearm wall's radius: the frame flush with the round parts that turn about the roll axis - and on the flat of the
    # front face, 4 inside the three sides (the tower's corners are sharp); the front face runs through the tower (the
    # pair straddles it)
    r_boss = D.bearing_boss_dia / 2.0
    assert r_boss - (D.bearing_od + D.seat_add) / 2.0 >= 4.0 and r_boss < D.ring_flange_dia / 2.0
    assert S["z_boss"] < S["z_ring_flange_1"] < S["z_ring_mid"] - BELT_W / 2.0
    assert _eq(-D.block_x[0], D.block_x[1], D.tower_y, w.wall_od / 2.0) and r_boss <= D.tower_y - 4.0
    assert S["z_bearing_1"] < S["z_front"] < S["z_boss"]
    # the 6806 pair: the lip stops the outer rings and clears the pulley's neck and the shoulders, which bear on the inner
    # rings only; the neck under the bearing's bore - bearing 2 slides over it onto its journal; the tower round the seats
    r_seat, j_r = (D.bearing_od + D.seat_add) / 2.0, (D.bearing_bore + D.journal_add) / 2.0
    assert D.bearing_bore == PARAMS.BEARING_6806_BORE and D.bearing_od == PARAMS.BEARING_6806_OD and D.shoulder_dia == PARAMS.BEARING_6806_SHOULDER_DIA
    assert j_r < D.shoulder_dia / 2.0 < D.lip_id / 2.0 - 1.0 and D.lip_id < D.bearing_od - 3.0
    assert D.neck_dia < D.bearing_bore < 2.0 * j_r
    assert D.tower_y - r_seat >= 4.0 and D.block_x[1] - r_seat >= 4.0 and r_seat + 4.0 <= -D.block_x[0]
    # the motor's face where its 20T meets the ring, the 20T's hub pulley_seat over the motor's pilot boss - down in the
    # plate's pilot hole, under the front face - and the 20T on the shaft to its far end; the plate in front of the motor
    # the block's front face, pad_t thick: the motor screws' heads flush in it (tests/forearm/test_roll_motor_screws.py).
    # The shaft's tip passes the forearm wall's plane: the rolling forearm keeps clear of it round the roll axis
    # (test_forearm_clears_the_elbow_while_rolling)
    assert _eq(S["z_ring_mid"] - S["z_motor_face"], D.t20) and _eq(S["z_front"], S["z_motor_face"] + D.pad_t)
    assert _eq(S["z_20t"], S["z_motor_face"] + D.motor.pilot_height + D.pulley_seat) and S["z_20t"] < S["z_front"]
    assert S["z_20t"] + D.t20_len <= S["z_motor_face"] + D.motor.shaft_length
    # the rotor clamp: its heads sunk in the pulley's front face, its tips at the shaft's rear face, the nuts 2 pitches
    # short of them in the flange, the flange 3 mm in front of them; its holes in the hubs' walls between the bore and the
    # journals - 1 mm of the pulley's neck outside them; the clamp between the wall's screws
    assert _eq(S["z_clamp_head"], S["z_spigot_end"] - D.clamp_screw.head_h - D.clamp_head_clear)
    assert _eq(S["z_shaft_end"], S["z_clamp_head"] - D.clamp_screw_len) and math.isclose(S["z_clamp_nut"] - S["z_shaft_end"], 2 * PARAMS.M3_PITCH)
    assert math.isclose(S["z_clamp_nut_seat"] - S["z_clamp_nut"], D.clamp_nut.h) and S["z_shoulder_1"] - S["z_clamp_nut_seat"] >= 3.0
    clamp_r, hole_r = D.clamp_circle_dia / 2.0, w.bolt_dia / 2.0
    assert clamp_r - hole_r - D.bore / 2.0 >= 1.5 and j_r - clamp_r - hole_r >= 1.2 - 1e-9 and D.neck_dia / 2.0 - clamp_r - hole_r >= 1.0 - 1e-9
    r_in, r_out, z0, z1 = clamp_nut_pocket(DEFAULT)
    assert r_in < D.bore / 2.0 and D.collar_od / 2.0 - r_out >= 3.0 and (z0, z1) == (S["z_shaft_end"], S["z_clamp_nut_seat"])
    head_r = D.clamp_screw.head_dia / 2.0 + D.clamp_head_clear
    assert min(math.dist(a, b) for a in clamp_points(DEFAULT) for b in flange_bolt_points_module(DEFAULT)) - head_r - hole_r >= 3.0
    # the forearm wall's screws through the wall and the spigot into nuts in the pulley's core: the tip 2 pitches past
    # its nut, the nut over the ring's rear flange and 2 mm short of the clamp's counterbores, 3 mm of the core outside
    # it, its pocket open into the bore
    assert S["z_wall_back"] == -w.wall_x[0] and S["z_end_tip"] == S["z_wall_back"] - w.screw_len and w.bolt_dia == PARAMS.M3_CLEAR
    assert math.isclose(S["z_end_nut"] - S["z_end_tip"], 2 * PARAMS.M3_PITCH) and math.isclose(S["z_end_nut_seat"] - S["z_end_nut"], D.end_nut.h)
    r_in, r_out, z0, z1 = end_nut_pocket(DEFAULT)
    assert r_in < D.bore / 2.0 and D.ring_core_dia / 2.0 - r_out >= 3.0 and (z0, z1) == (S["z_end_nut"] - D.end_nut_fit, S["z_end_nut_seat"])
    assert z0 >= S["z_ring_flange_1"] + 2.0 and z1 <= S["z_clamp_head"] - 2.0 and w.cable_bore >= D.bore
    # the bay behind bearing 1, bored on through the frame's back - the shaft, bearing 1 on it, goes in along the axis from
    # behind -: round the lug, wide enough for bearing 1; the tower's rear in the profile its own (tower_rear: the rear
    # line's anchor). The shaft's rear face inside the rear line, its lug 2 mm or more at any roll angle (it sweeps the
    # top: FOREARM_ROLL_LIMIT_DEG) - the elbow motor's board passes the rear line at the elbow's lower limit
    assert S["z_tower_rear"] == D.tower_rear
    assert D.bay_r >= D.stop_lug_r[1] + 1.0 and D.bay_r > r_seat and D.tower_y - D.bay_r >= 3.0
    (nu, nv), dist = rear_line(DEFAULT)
    assert PARAMS.FOREARM_ROLL_LIMIT_DEG > 90.0 and dist - (nu * (D.stop_lug_r[1] - S["y_elbow"]) + nv * S["z_shaft_end"]) >= 2.0
    # the stop: the lug on the shaft's flange, the post on the tower's rear face, 2.5 of each beside the other, each clear
    # of the other's part; where they meet: tests/test_sweeps.py
    lug_top = S["z_stop_lug"] + D.stop_lug_t
    assert S["z_stop_lug"] == S["z_shaft_end"] and S["z_stop_post"] == S["z_bearing_1"] - D.stop_post_t
    assert lug_top - S["z_stop_post"] >= 2.5 - 1e-9 and S["z_bearing_1"] - lug_top >= 2.0
    assert D.stop_lug_r[0] < D.collar_od / 2.0 < D.stop_post_r[0] - 1.0 and D.bearing_od / 2.0 + 0.5 <= D.stop_post_r[0] < D.stop_lug_r[1]
    assert D.stop_post_r[1] > D.bay_r
    # the motor: ON the elbow axis, centred on the roll axis in X, the drive's centre distance (a design value: the stock
    # roll belt is longer than the open loop, its idler takes up the rest - tests/forearm/test_roll_idler.py) the offset;
    # 2 mm over the pocket's floor, as far inside the side wall (as thick as the floor); its pocket round it and its
    # board (no slide: the idler tensions the belt), its top inside the round end, under the bay; turned so its
    # connector points down into the opening
    assert S["x_motor"] == 0.0 and S["y_motor"] == S["y_elbow"] == D.elbow_y and D.motor_spin_deg == 0.0
    assert D.roll_belt in STANDARD_2GT_LENGTHS and closed_belt_length(D.centre_distance, D.ring_teeth, GT2_PULLEY_20T_TEETH) < D.roll_belt
    assert -D.motor.body_width / 2.0 - S["x_cradle"] >= 2.0 and -D.board_w / 2.0 - S["x_cradle"] >= 1.5
    assert D.side_t == D.web_t and _eq(S["x_side"], D.block_x[1] - D.side_t, -S["x_cradle"])
    y0, y1, z0, z1 = motor_pocket(DEFAULT)
    half = max(D.motor.body_width, D.board_w) / 2.0 + D.pocket_clear
    assert (y0, y1) == (S["y_motor"] - half, S["y_motor"] + half) and z1 == S["z_motor_face"]
    assert z0 == S["z_motor_board"] - D.board_stack - D.pocket_rear
    assert max(math.hypot(y1 - S["y_elbow"], z) for z in (z0, z1)) <= D.end_r - 5.0
    assert -D.bay_r - y1 >= 5.0
    # the motor slides up from the bottom: the pocket deep enough for it to slide motor_drop back (its pilot boss under
    # the plate), its shaft in the slit - wider than the shaft, the screws' counterbores whole either side of it
    play = (z1 - z0) - (D.motor.body_length + D.board_stack)
    assert D.motor.pilot_height < D.motor_drop <= play - 0.25 + 1e-9
    assert D.motor.shaft_dia + 1.0 <= D.shaft_slit < D.motor_holes.pilot_dia
    hole_in = min(abs(x - S["x_motor"]) for x, _y in pad_bolt_points(DEFAULT)) - D.motor_holes.pocket_dia / 2.0
    assert hole_in - D.shaft_slit / 2.0 >= 5.0
    # the board's side plugs face the walls (the side wall, the floor): a groove inside each, plug_clear round the
    # header, from the plug down out through the bottom and motor_drop back behind it (the slide), plug_skin of wall left
    # outside - each cable bent straight down at its plug: the straight room it leaves (8.5) short of MKS_PLUG_ROOM
    rooms = side_plug_room(DEFAULT)
    assert set(rooms) == {"side_neg_x", "side_pos_x"} and all(r == pytest.approx(8.5) for r in rooms.values())
    assert all(r < PARAMS.MKS_PLUG_ROOM[n] for n, r in rooms.items())
    for ((hx0, hx1, _hy0, hy1, hz0, hz1), way), (gx0, gx1, gy0, gy1, gz0, gz1) in zip(side_headers(DEFAULT).values(), plug_slots(DEFAULT), strict=True):
        assert (gx1 == pytest.approx(D.block_x[1] - D.plug_skin) and gx0 < S["x_side"] and hx1 < S["x_side"]) if way > 0 else \
               (gx0 == pytest.approx(D.block_x[0] + D.plug_skin) and gx1 > S["x_cradle"] and hx0 > S["x_cradle"])
        assert gy0 < S["y_elbow"] - D.end_r and gy1 == pytest.approx(hy1 + D.plug_clear)
        assert gz0 == pytest.approx(hz0 - D.plug_clear - D.motor_drop) and gz1 == pytest.approx(hz1 + D.plug_clear)
    # the elbow 90T's nuts: seated in the boss, the M4 from the pulley's counterbores 2 pitches past its nut, nut and tip
    # 2 mm under the pocket's floor; each hex channel open up into the pocket (the nuts go in from there before the
    # motor), and clear of the pin bore, which ends under the seats
    corner = D.nut_af / math.sqrt(3.0)
    tip = D.stub_x[0] - D.pulley_hub_len + PARAMS.GT2_PULLEY_90T_HEAD_SEAT + D.pulley_screw_len
    assert D.boss_x[0] < D.nut_seat_x < D.boss_x[1] and D.pulley_bolt_r + corner <= D.boss_dia / 2.0 - 2.0
    assert tip - (D.nut_seat_x + D.nut_t) >= 2 * 0.7 - 1e-9 and tip <= S["x_cradle"] - 2.0
    assert D.nut_seat_x + D.nut_t <= S["x_cradle"] - 2.0 and D.pin_bore_x[1] == D.nut_seat_x - 1.0
    assert nut_channel_end(DEFAULT) == S["x_cradle"] + D.nut_channel_past
    assert all(y0 < S["y_elbow"] + y - corner and S["y_elbow"] + y + corner < y1 and z0 < z - corner and z + corner < z1
               for y, z in pulley_bolt_points(DEFAULT))
    assert D.pulley_bolt_r + D.pulley_bolt_dia / 2.0 <= D.stub_dia / 2.0 - 1.5   # the SolidWorks coupler's own wall round them
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
    # 2 bearings, the rotor clamp's screws and nuts, motor, board, 20T, the motor's screws, belt, the tensioner's idler,
    # screw and nut, the back panel's magnets / frame, shaft, pulley, back panel
    assert module_tints(module, M.TINT) == {True: 13, False: 4}


@pytest.mark.slow
def test_bearings_press_on_the_journals_and_slip_in_the_seat(module):
    """Bearing 1 on the shaft's hub, bearing 2 on the pulley's, both in the frame's tower."""
    block = _leaf(module, "forearm_roll_block")
    press = math.pi / 4.0 * ((D.bearing_bore + D.journal_add) ** 2 - D.bearing_bore ** 2) * D.bearing_width   # ~99 mm^3
    for label, hub in (("bearing_6806:1", "forearm_roll_shaft"), ("bearing_6806:2", "forearm_roll_pulley")):
        b = _leaf(module, label)
        assert interference(b, block) < 1.0, label                                        # the seat is the bearing OD + seat_add
        assert 0.9 * press <= interference(b, _leaf(module, hub)) <= press + 0.5, label   # the journal's interference in the inner ring
    for label, z in (("bearing_6806:1", S["z_bearing_1"]), ("bearing_6806:2", S["z_bearing_2"])):
        bb = _leaf(module, label).bounding_box()
        assert math.isclose(bb.min.Z, z, abs_tol=1e-6) and math.isclose(bb.max.Z, z + D.bearing_width, abs_tol=1e-6)


@pytest.mark.slow
def test_everything_else_in_the_module_is_clean(module):
    names = [c.label for c in module.children]
    press_pairs = {("bearing_6806:1", "forearm_roll_shaft"), ("bearing_6806:2", "forearm_roll_pulley")}
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if (a, b) in press_pairs or (b, a) in press_pairs:
                continue
            if {a, b} == {"nema17_40mm:forearm_roll", "mks_servo42d:forearm_roll"}:
                continue   # the kit's screws run through the motor by design
            vol = interference(_leaf(module, a), _leaf(module, b))
            assert vol < 1.0, f"{a} x {b}: {vol:.1f} mm^3"


@pytest.mark.slow
def test_the_rotor_turns_clear_of_the_frame(module):
    """The pulley and the shaft keep run_gap off the frame: the ring's flange over the bearing boss, the pulley's neck in
    the lip, the shaft's flange and lug in the bay; the pulley's neck meets the shaft's journal face to face at bearing 1's
    front face."""
    block = _leaf(module, "forearm_roll_block")
    for label in ("forearm_roll_pulley", "forearm_roll_shaft"):
        gap = closest_points(_leaf(module, label), block).distance
        assert gap >= RUN_GAP - 0.01, f"{label}: {gap:.2f} mm off the frame"
    pulley, shaft = _leaf(module, "forearm_roll_pulley"), _leaf(module, "forearm_roll_shaft")
    assert math.isclose(pulley.bounding_box().min.Z, S["z_meet"], abs_tol=1e-6) and math.isclose(shaft.bounding_box().max.Z, S["z_meet"], abs_tol=1e-6)
    assert closest_points(pulley, shaft).distance < 1e-6 and interference(pulley, shaft) < 1e-3


@pytest.mark.slow
def test_the_motor_slides_up_from_the_bottom(module):
    """The roll motor and its board slide up into the pocket from the octagon's bottom motor_drop back - the pilot boss
    under the plate, the shaft along the slit - and go forward into the pilot hole: clear of the frame from below it to
    the seat, in 2 mm steps, and through the last move."""
    block = _leaf(module, "forearm_roll_block")
    kit = [_leaf(module, label) for label in ("nema17_40mm:forearm_roll", "mks_servo42d:forearm_roll")]
    out = S["y_motor"] + D.board_w / 2.0 - (S["y_elbow"] - D.end_r)   # the board's top below the bottom flat
    for k in range(int(out // 2.0) + 2):
        for part in kit:
            vol = interference(part.moved(Location((0.0, -2.0 * k, -D.motor_drop))), block)
            assert vol < 0.5, f"{part.label} {2.0 * k:.0f} mm down, {D.motor_drop} back: {vol:.1f} mm^3 in the frame"
    for k in range(11):
        for part in kit:
            vol = interference(part.moved(Location((0.0, 0.0, -D.motor_drop * k / 10.0))), block)
            assert vol < 0.5, f"{part.label} going forward, {D.motor_drop * k / 10.0:.2f} back: {vol:.1f} mm^3 in the frame"


def _round_trip(part, r: float, a_deg: float, z: float) -> bool:
    return is_inside(part, r * math.cos(math.radians(a_deg)), r * math.sin(math.radians(a_deg)), z)


@pytest.mark.slow
def test_frame_features(module):
    block = _leaf(module, "forearm_roll_block")
    r_seat, r_lip, r_boss = (D.bearing_od + D.seat_add) / 2.0, D.lip_id / 2.0, D.bearing_boss_dia / 2.0
    # the tower: the two seats, the lip between them; the bearing boss round bearing 2's seat, nothing above its top
    for z in (S["z_bearing_1"] + 3.0, S["z_bearing_2"] + 3.0):
        assert not is_inside(block, 0, -(r_seat - 0.5), z) and is_inside(block, 0, -(r_seat + 0.5), z)
    assert is_inside(block, 0, -(r_lip + 0.5), S["z_lip"] + 1.0) and not is_inside(block, 0, -(r_lip - 0.5), S["z_lip"] + 1.0)
    zb = (S["z_front"] + S["z_boss"]) / 2.0
    for x, y in ((0.0, 1.0), (1.0, 0.0), (0.0, -1.0), (-1.0, 0.0)):
        assert is_inside(block, x * (r_boss - 0.5), y * (r_boss - 0.5), zb) and not is_inside(block, x * (r_boss + 0.5), y * (r_boss + 0.5), zb)
    assert not is_inside(block, 0, r_seat + 1.0, S["z_boss"] + 0.5)
    # one flat front face: material just behind it, none in front, beside the boss
    assert is_inside(block, -28.0, -30.0, S["z_front"] - 0.5) and not is_inside(block, -28.0, -30.0, S["z_front"] + 0.5)
    # the octagonal end about the elbow axis: a flat on each axis end_r off it (on j1_link's round end) behind the motor's
    # pocket, nothing past it; the end's two corners between them, end_r / cos(180 / end_sides) out; nothing behind the
    # rear line (roll.py rear_line, the elbow motor's board's nearest at the elbow's lower limit: tests below) - probed in
    # the side wall's skin, beside the pocket's opening through the bottom and outside the plug's groove
    ye, xw = S["y_elbow"], D.block_x[1] - 0.5
    assert is_inside(block, xw, ye, -(D.end_r - 2.0)) and not is_inside(block, xw, ye, -(D.end_r + 1.0))
    assert is_inside(block, xw, ye - (D.end_r - 2.0), 0.0) and not is_inside(block, xw, ye - (D.end_r + 1.0), 0.0)
    corner = D.end_r / math.cos(math.pi / D.end_sides)
    for deg in (180.0 - 180.0 / D.end_sides, 180.0 + 180.0 / D.end_sides):
        u, v = math.cos(math.radians(deg)), math.sin(math.radians(deg))
        assert is_inside(block, xw, ye + (corner - 1.0) * u, (corner - 1.0) * v)
        assert not is_inside(block, xw, ye + (corner + 0.5) * u, (corner + 0.5) * v)
    (nu, nv), dist = rear_line(DEFAULT)
    assert is_inside(block, xw, ye + (dist - 1.0) * nu, (dist - 1.0) * nv)
    assert not is_inside(block, xw, ye + (dist + 0.5) * nu, (dist + 0.5) * nv)
    bb = block.bounding_box()
    assert math.isclose(bb.min.Y, ye - D.end_r, abs_tol=0.01) and math.isclose(bb.min.Z, -D.end_r, abs_tol=0.01)
    # the motor's pocket: the floor and the side wall either side of it - the side face closed -, the space, open down
    # through the octagon's bottom between the walls; its top and back round it
    _y0, y1, z0, _z1 = motor_pocket(DEFAULT)
    assert is_inside(block, S["x_cradle"] - 1.0, ye, 0.0) and not is_inside(block, S["x_cradle"] + 1.0, ye, 0.0)
    assert is_inside(block, S["x_side"] + 1.0, ye, 0.0) and not is_inside(block, S["x_side"] - 1.0, ye, 0.0)
    assert is_inside(block, D.block_x[1] - 0.5, ye, 0.0)
    assert is_inside(block, 10.0, y1 + 1.0, 0.0) and not is_inside(block, 10.0, y1 - 1.0, 0.0)
    assert not is_inside(block, 10.0, ye - D.end_r + 1.0, 0.0) and is_inside(block, S["x_side"] + 3.0, ye - D.end_r + 1.0, 0.0)
    assert is_inside(block, 10.0, ye, z0 - 1.0) and not is_inside(block, 10.0, ye, z0 + 1.0)
    # the side plugs' grooves inside the walls: open from the pocket out to the skin, from the plug down out through the
    # bottom; the skin outside, the wall over each
    for (gx0, gx1, _gy0, gy1, gz0, gz1), (_h, way) in zip(plug_slots(DEFAULT), side_headers(DEFAULT).values(), strict=True):
        zg, xg = (gz0 + gz1) / 2.0, (gx0 + gx1) / 2.0
        outer = (D.block_x[1] - 0.5) if way > 0 else (D.block_x[0] + 0.5)
        for yg in (gy1 - 1.0, ye - 35.0):                                       # (the octagon's lower facet cuts it off further down)
            assert not is_inside(block, xg, yg, zg) and is_inside(block, outer, yg, zg)
        assert is_inside(block, xg, gy1 + 1.0, zg)
    # the plate, the pocket's front wall: material beside the pilot hole, the hole, the pocket behind, the 20T's room in front
    assert is_inside(block, 10, ye - 10, S["z_motor_face"] + 1.5) and not is_inside(block, 0, ye, S["z_motor_face"] + 1.5)
    assert not is_inside(block, 10, ye - 10, S["z_motor_face"] - 1.5) and not is_inside(block, 10, ye - 10, S["z_front"] + 1.0)
    # the shaft's slit: open through the plate from the pilot hole down out through the bottom, the plate either side of it
    zp = S["z_motor_face"] + D.pad_t / 2.0
    for y in (ye - 15.0, ye - 30.0):
        assert not is_inside(block, 0, y, zp) and is_inside(block, D.shaft_slit / 2.0 + 1.0, y, zp)
        assert is_inside(block, -D.shaft_slit / 2.0 - 1.0, y, zp)
    # the shaft's bay behind bearing 1: bored on through the frame's back (past the tower's rear, the outline's), the
    # tower round it, the side face closed beside it; the stop post at -X only
    zb = S["z_bearing_1"] - 3.0
    assert not is_inside(block, 0, 0, zb) and not is_inside(block, 0, 0, S["z_tower_rear"] - 1.0)
    assert is_inside(block, D.block_x[1] - 0.5, 0, zb) and is_inside(block, 0, D.bay_r + 1.5, S["z_bearing_1"] - 2.0)
    zp, rp = S["z_stop_post"] + 1.0, (D.stop_post_r[0] + D.stop_post_r[1]) / 2.0
    assert is_inside(block, -rp, 0, zp) and not is_inside(block, rp, 0, zp)
    # the elbow flange about the elbow axis (y_elbow): the lip inside its radius only, the boss, the stub, the pin bore,
    # the pulley bolts + nut channels up through the floor into the motor's pocket
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
        assert not is_inside(block, above, y, z) and not is_inside(block, S["x_cradle"] - 0.5, y, z)          # on up through the floor
        assert not is_inside(block, above, y - (flat - 0.2) * u[0], z - (flat - 0.2) * u[1])       # the hex, a flat toward the axis
        assert is_inside(block, above, y - (flat + 0.3) * u[0], z - (flat + 0.3) * u[1])
        assert not is_inside(block, above, y + (corner - 0.2) * t[0], z + (corner - 0.2) * t[1])
        assert is_inside(block, above, y + (corner + 0.3) * t[0], z + (corner + 0.3) * t[1])


def _nut_pocket_probes(part, points, pocket, af: float):
    """A nut's pocket along each screw's angle: a flat either side, past the nut's corner, open into the bore."""
    _r_in, r_out, z0, z1 = pocket
    zn, flat = (z0 + z1) / 2.0, af / 2.0
    for x, y in points:
        rc = math.hypot(x, y)
        u, t = (x / rc, y / rc), (-y / rc, x / rc)                                 # radial, tangential
        assert not is_inside(part, x + (flat - 0.2) * t[0], y + (flat - 0.2) * t[1], zn)
        assert is_inside(part, x + (flat + 0.3) * t[0], y + (flat + 0.3) * t[1], zn)
        assert not is_inside(part, (r_out - 0.2) * u[0], (r_out - 0.2) * u[1], zn)
        assert is_inside(part, (r_out + 0.3) * u[0], (r_out + 0.3) * u[1], zn)
        assert not is_inside(part, (D.bore / 2.0 + 0.3) * u[0], (D.bore / 2.0 + 0.3) * u[1], zn)


@pytest.mark.slow
def test_pulley_and_shaft_features(module):
    pulley, shaft = _leaf(module, "forearm_roll_pulley"), _leaf(module, "forearm_roll_shaft")
    w, j_r, a = DEFAULT.roll_end, (D.bearing_bore + D.journal_add) / 2.0, 22.5   # a: an angle clear of both screw patterns
    hole_r, head_r = w.bolt_dia / 2.0, D.clamp_screw.head_dia / 2.0 + D.clamp_head_clear
    # the pulley: the bore, the ring (a land, a groove, all round), its rear flange, the core, the shoulder, the hub, the spigot
    zm = S["z_ring_mid"]
    assert not is_inside(pulley, 0, 0, zm) and _round_trip(pulley, D.bore / 2.0 + 0.5, a, zm)
    assert is_inside(pulley, 27.0, 0, zm) and not is_inside(pulley, 28.3, 0, zm) and is_inside(pulley, 0, 27.5, zm)
    assert _round_trip(pulley, D.ring_flange_dia / 2.0 - 0.5, a, S["z_ring_flange_1"] + 0.6)
    assert _round_trip(pulley, D.shoulder_dia / 2.0 - 0.3, a, S["z_boss"] + 0.5) and not _round_trip(pulley, D.shoulder_dia / 2.0 + 0.3, a, S["z_boss"] + 0.5)
    assert _round_trip(pulley, j_r - 0.3, a, S["z_bearing_2"] + 3.0) and not _round_trip(pulley, j_r + 0.3, a, S["z_bearing_2"] + 3.0)
    zn = (S["z_meet"] + S["z_bearing_2"]) / 2.0                                 # the neck, on through the lip to bearing 1
    assert _round_trip(pulley, D.neck_dia / 2.0 - 0.2, a, zn) and not _round_trip(pulley, D.neck_dia / 2.0 + 0.2, a, zn)
    assert not _round_trip(pulley, D.bore / 2.0 - 0.5, a, zn)
    assert _round_trip(pulley, w.flange_dia / 2.0 - 0.5, a, S["z_wall"] + 1.0) and not _round_trip(pulley, w.flange_dia / 2.0 + 0.5, a, S["z_wall"] + 1.0)
    # the forearm wall's 4x M3: the hole from the front face on past each nut to the tip, the nut's pocket in the core
    for x, y in flange_bolt_points_module(DEFAULT):
        assert not is_inside(pulley, x, y, S["z_wall"] + 1.0) and not is_inside(pulley, x, y, S["z_end_tip"] - D.end_nut_fit + 0.3)
        assert is_inside(pulley, x, y, S["z_end_tip"] - D.end_nut_fit - 0.3)
    _nut_pocket_probes(pulley, flange_bolt_points_module(DEFAULT), end_nut_pocket(DEFAULT), D.end_nut.af)
    # the rotor clamp: each hole through the hub, its head's counterbore in the front face
    for x, y in clamp_points(DEFAULT):
        rc = math.hypot(x, y)
        u = (x / rc, y / rc)
        assert not is_inside(pulley, x, y, S["z_bearing_2"] + 3.0) and is_inside(pulley, x + (hole_r + 0.5) * u[0], y + (hole_r + 0.5) * u[1], S["z_bearing_2"] + 3.0)
        assert not is_inside(pulley, x, y, zn) and is_inside(pulley, x + (hole_r + 0.5) * u[0], y + (hole_r + 0.5) * u[1], zn)
        assert not is_inside(pulley, x + (head_r - 0.3) * u[0], y + (head_r - 0.3) * u[1], S["z_spigot_end"] - 1.0)
        assert is_inside(pulley, x + (head_r - 0.3) * u[0], y + (head_r - 0.3) * u[1], S["z_clamp_head"] - 0.5)
    bb = pulley.bounding_box()
    assert math.isclose(bb.max.Z, S["z_spigot_end"], abs_tol=1e-6) and math.isclose(bb.min.Z, S["z_meet"], abs_tol=1e-6)
    assert math.isclose(bb.max.X, D.ring_flange_dia / 2.0, abs_tol=0.05)
    # the shaft: the bore, the hub, the shoulder, the flange, the lug at +X, the clamp's holes and its nuts' pockets
    zh = S["z_bearing_1"] + 3.0
    assert not is_inside(shaft, 0, 0, zh) and _round_trip(shaft, D.bore / 2.0 + 0.5, a, zh)
    assert _round_trip(shaft, j_r - 0.3, a, zh) and not _round_trip(shaft, j_r + 0.3, a, zh)
    assert _round_trip(shaft, D.shoulder_dia / 2.0 - 0.3, a, S["z_shoulder_1"] + 0.5) and not _round_trip(shaft, D.shoulder_dia / 2.0 + 0.3, a, S["z_shoulder_1"] + 0.5)
    assert _round_trip(shaft, D.collar_od / 2.0 - 0.5, 90.0, S["z_shaft_end"] + 1.0) and not _round_trip(shaft, D.collar_od / 2.0 + 0.5, 90.0, S["z_shaft_end"] + 1.0)
    assert is_inside(shaft, (D.stop_lug_r[0] + D.stop_lug_r[1]) / 2.0, 0, S["z_stop_lug"] + 1.0)
    assert not is_inside(shaft, -(D.stop_lug_r[0] + D.stop_lug_r[1]) / 2.0, 0, S["z_stop_lug"] + 1.0)
    for x, y in clamp_points(DEFAULT):
        assert not is_inside(shaft, x, y, zh) and not is_inside(shaft, x, y, S["z_shaft_end"] + 0.2)   # the hole; the pocket open at the rear face
    _nut_pocket_probes(shaft, clamp_points(DEFAULT), clamp_nut_pocket(DEFAULT), D.clamp_nut.af)
    bb = shaft.bounding_box()
    assert math.isclose(bb.min.Z, S["z_shaft_end"], abs_tol=1e-6) and math.isclose(bb.max.Z, S["z_meet"], abs_tol=1e-6)


@functools.cache
def _placed_module():
    """The module in its mount frame - j2_link's before the forearm's turn (tests/forearm/helpers.py; tests/built.py:
    shared, read-only)."""
    return built.model(M.forearm_roll_drive).moved(to_location(module_frame_in_host(DEFAULT)))


@functools.cache
def _placed_part(label: str):
    """One of the module's parts in its mount frame (shared, read-only). A child of _placed_module() stays in the MODULE
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
    """In the module's mount frame (j2_link's before the forearm's turn), at the capture pose: the module against the
    turned forearm and the upper arm - the stub through its lip -; the frame's stub end on the elbow pulley's face
    (contact, no overlap), the pulley's spigot in j2_link's recess.
    (The mounted motors, boards, bearings and pulleys against the whole module: tests/test_mounts.py
    test_motors_and_boards_clear_their_neighbours.) The upper arm's own motor and board - on its +N side like the
    module's, ELBOW_MOTOR_CENTRES from the elbow axis - turned about it over the elbow's range stay 1 mm off the whole
    module (lib/params.py ELBOW_PITCH_LIMITS_DEG: the lower limit keeps room to spare)."""
    module = _placed_module()
    for key, other in (("j2_link", j2_link_in_drive_frame()), ("j1_link#1", in_drive_frame("j1_link#1"))):
        vol = interference(module, other)
        assert vol < 1.0, f"module x {key}: {vol:.1f} mm^3"
    block = _placed_part("forearm_roll_block")
    assert math.isclose(block.bounding_box().min.Z, ELBOW_PULLEY_FACE_Z, abs_tol=1e-6)                   # the stub's end on the pulley
    assert math.isclose(in_drive_frame("gt2_pulley_90t#3").bounding_box().max.Z, ELBOW_PULLEY_FACE_Z, abs_tol=0.05)
    pulley = _placed_part("forearm_roll_pulley")
    assert math.isclose(pulley.bounding_box().min.X, -S["z_spigot_end"], abs_tol=1e-6)
    for key in ("nema17_40mm#2", "mks_servo42d#2"):
        for elbow in _elbow_range(5):
            gap = _upper_arm_at(in_drive_frame(key), elbow).distance_to(module)
            assert gap > 1.0, f"{key} at elbow {elbow:+.0f} deg: {gap:.1f} mm from the module"


@pytest.mark.slow
@pytest.mark.parametrize("deg", PARAMS.ELBOW_PITCH_LIMITS_DEG)
def test_module_clears_the_folded_upper_arm(deg):
    """The upper arm (j1_link) swung about the elbow axis (along host z, ELBOW_AXIS_HOST) to the elbow's limits never
    runs into the frame, its motor or the board - and never comes nearer the frame than designed: its journal's edge to
    the bore's rim (a radial gap: the bearings set it; the frame's flat underside rides FACE_GAP or more over j1_link's
    flat top, round its bore, so the two limits stand for the angles between - the capture pose is
    test_module_clears_its_neighbours_in_the_arm's). Widen the sample if the top stops being flat under it."""
    module = _placed_module()
    upper_arm = _upper_arm_at(in_drive_frame("j1_link#1"), deg)
    vol = interference(module, upper_arm)
    assert vol < 1.0, f"elbow {deg:+.0f} deg: module x j1_link#1: {vol:.1f} mm^3"
    journal_edge = math.hypot(UPPER_ARM_BORE_R - D.journal_dia / 2.0, DEFAULT.roll_end.axis_z + D.journal_x[0] - UPPER_ARM_FACE_Z)
    block_gap = closest_points(_placed_part("forearm_roll_block"), upper_arm).distance
    assert block_gap == pytest.approx(journal_edge, abs=0.01) and block_gap >= 1.0


def _slab(shape, x0: float, x1: float):
    """A copy of `shape` cut to host x0 .. x1 (shared shapes stay untouched), or None."""
    box = Location((x0, -500.0, -500.0)) * Box(x1 - x0, 1000.0, 1000.0, align=(Align.MIN, Align.MIN, Align.MIN))
    cut = copy.deepcopy(shape) & box
    return cut if cut is not None and cut.solids() else None


def _radii(shape, axis_z: float) -> list[float]:
    """Distances from the roll axis (host: y 0, z axis_z, along X) of the shape's vertices and points along its edges."""
    pts = [v for v in shape.vertices()] + [e.position_at(k / 40.0) for e in shape.edges() for k in range(41)]
    return [math.hypot(p.Y, p.Z - axis_z) for p in pts]


@pytest.mark.slow
def test_forearm_clears_the_elbow_while_rolling():
    """j2_link rolls about the roll axis (host: through (0, 0, axis_z) along -X), which keeps every point's x and its
    distance from the axis: the whole link ends at its wall's elbow face, and the upper arm's round end, the elbow pulley
    and the stator's parts (the frame, the board, the 20T, the belt, the tensioner's idler and nut) lie beyond that plane,
    so no roll angle brings them together. The roll motor's shaft and the tensioner's screw head pass the plane: in the
    slab they share with the link, the link stays inside a radius about the roll axis that they stay 2 mm outside."""
    link = j2_link_in_drive_frame()
    plane, axis_z = link.bounding_box().max.X, DEFAULT.roll_end.axis_z
    assert plane == pytest.approx(DEFAULT.roll_end.wall_x[1], abs=1e-6), f"j2_link reaches x {plane:.2f}, past its wall"
    others = {key: in_drive_frame(key) for key in ("j1_link#1", "gt2_pulley_90t#3")}
    others |= {label: _placed_part(label) for label in ("forearm_roll_block", "mks_servo42d:forearm_roll", "gt2_pulley_20t:forearm_roll",
                                                        "forearm_roll_belt:forearm_roll", "gt2_idler_20t:forearm_roll", "roll_idler_nut")}
    for key, other in others.items():
        x = other.bounding_box().min.X
        assert x >= plane, f"{key} reaches x {x:.2f}, into the rolling forearm's side of x {plane:.2f}"
    past = {label: _placed_part(label) for label in ("nema17_40mm:forearm_roll", "roll_idler_screw")}
    x0 = min(part.bounding_box().min.X for part in past.values())
    assert x0 < plane
    reach = max(_radii(_slab(link, x0, plane + 0.01), axis_z))
    for label, part in past.items():
        inner = min(_radii(_slab(part, x0, plane), axis_z))
        assert inner - reach >= 2.0, f"{label}: {inner:.2f} from the roll axis past the wall's plane, the forearm reaches {reach:.2f}"


@pytest.mark.slow
@pytest.mark.parametrize("elbow", PARAMS.ELBOW_PITCH_LIMITS_DEG)
@pytest.mark.parametrize("roll", [-PARAMS.FOREARM_ROLL_LIMIT_DEG, -90.0, 0.0, 90.0, PARAMS.FOREARM_ROLL_LIMIT_DEG])
def test_forearm_clears_the_upper_arm_at_the_elbow_limits(elbow, roll):
    """The elbow folded to its limits, the forearm rolled anywhere: j2_link (its round wall 48 along the roll axis from
    the elbow axis' station and ELBOW_ROLL_OFFSET across it, its straight web; turned with the forearm) stays 2 mm off the
    upper arm beside it."""
    link = j2_link_in_drive_frame().rotate(Axis((0.0, 0.0, DEFAULT.roll_end.axis_z), (-1.0, 0.0, 0.0)), roll)
    upper_arm = _upper_arm_at(in_drive_frame("j1_link#1"), elbow)
    gap = closest_points(link, upper_arm).distance
    assert gap >= 2.0, f"elbow {elbow:+.0f} deg, roll {roll:+.0f} deg: j2_link {gap:.2f} mm off j1_link"
