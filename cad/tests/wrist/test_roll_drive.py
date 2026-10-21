"""The wrist roll drive (assemblies/wrist_roll_drive.py): the joint's axis R' WRIST_ROLL_OFFSET across the wrist_pitch axis
(a wrist offset) and parallel to the SolidWorks roll axis, the module frame on wrist_link (its frame), the stack (the
pancake centred on the pitch axis, its 20T's band the 40T's, the 6806 pair under the band, the rotor split between them
and clamped), the belt a stock length, the parts' fits inside the module and against wrist_link, the pancake's way in,
the rotor's walls, and what the drive replaced (the pancake's and its 20T's records retired, the bracket dropped).
The joint limits it sets: tests/test_sweeps.py."""
import math

import pytest
from build123d import Location, Vector
from cadgen.geometry import closest_points

from assemblies import wrist_roll_drive as M
from lib import params as PARAMS
from lib import placements as P
from lib.belts import STANDARD_2GT_LENGTHS, belt_length, pulley_od
from lib.datum import to_location
from lib.wrist import DEFAULT
from lib.wrist.layout import (
    board_window,
    clamp_nut_pocket,
    clamp_points,
    connector_bolt_points_module,
    face_nut_pocket,
    module_frame_in_host,
    motor_pocket,
    roll_belt_loop,
    stack_positions,
)
from robot import frames as F
from tests import built
from tests.helpers import interference, module_tints

S = stack_positions(DEFAULT)
D = DEFAULT.roll
RUN_GAP = 1.0      # the least gap between the turning rotor and the frame
# The board's 4x M3 x 30 (the kit's, 19.6 into the motor) run through the pancake's own 4x M3 in the same holes - on the
# motor the board's replace them (their length on a pancake: docs/open_issues.md): 4x 79.4 mm^3 of shank in the model
KIT_OVERLAP = 317.7


def _offset_from_line(point, origin, axis) -> float:
    d = [p - o for p, o in zip(point, origin, strict=True)]
    t = sum(x * y for x, y in zip(d, axis, strict=True))
    return math.sqrt(sum((x - t * y) ** 2 for x, y in zip(d, axis, strict=True)))


def test_module_frame_and_record():
    """On wrist_link#1 (turned with it: beyond the forearm's turn), +Z along the wrist_roll axis toward the gripper, the
    origin on it; the pitch axis WRIST_ROLL_OFFSET across - the pancake sits on it."""
    loc = to_location(module_frame_in_host(DEFAULT))
    x = (loc * Location((1.0, 0.0, 0.0))).position - loc.position
    z = (loc * Location((0.0, 0.0, 1.0))).position - loc.position
    assert (x - Vector(0, 0, -1)).length < 1e-9 and (z - Vector(1, 0, 0)).length < 1e-9   # module +X = host -Z, +Z = host +X
    world = P.location("wrist_roll_drive#1", "world")
    zw = (world * Location((0.0, 0.0, 1.0))).position - world.position
    assert abs(zw.dot(Vector(*F.WRIST_ROLL_AXIS)) - 1.0) < 1e-6
    assert _offset_from_line(tuple(world.position), F.WRIST_ROLL_ORIGIN, F.WRIST_ROLL_AXIS) < 0.01
    # the wrist offset: R' the belt's centre distance from the pitch axis (perpendicular to both) - the pancake on it
    assert PARAMS.WRIST_ROLL_OFFSET == D.centre_distance == -S["y_motor"] > 40.0
    assert _offset_from_line(tuple(world.position), F.WRIST_PITCH_ORIGIN, F.WRIST_PITCH_AXIS) == pytest.approx(D.centre_distance, abs=0.01)
    motor = (world * Location((S["x_motor"], S["y_motor"], 0.0))).position
    assert _offset_from_line(tuple(motor), F.WRIST_PITCH_ORIGIN, F.WRIST_PITCH_AXIS) < 0.01
    # and the SolidWorks roll axis through the wrist centre, parallel: off it across the plate and along its normal
    assert _offset_from_line(F.WRIST_CENTRE, F.WRIST_ROLL_ORIGIN, F.WRIST_ROLL_AXIS) == pytest.approx(
        math.hypot(D.centre_distance, DEFAULT.end_face.axis_z - (DEFAULT.plate.thickness - D.axis_h)), abs=0.05)


def test_what_the_drive_replaced():
    """The pancake's and its 20T's SolidWorks records are retired (the module carries both), gripper_clamp_bracket is
    dropped (its product skipped); the wrist's links carry the module's bodies."""
    assert {"nema17_pancake#1", "gt2_pulley_20t#1"} <= set(P.RETIRED) and "gripper_clamp_bracket#1" not in P.OCCURRENCES
    assert "brack_for_hand_cmap" in {s["label"] for s in P.DATA["skipped"]}
    assert F.LINKS["wrist_pitch_link"][-1] == "wrist_roll_drive#1:stator" and F.LINKS["wrist_roll_link"][0] == "wrist_roll_drive#1:rotor"
    assert M.BODIES["rotor"] == {"wrist_roll_shaft", "wrist_roll_pulley"}
    assert M.BODIES["stator"] == {"bearing_6806", "nema17_pancake", "mks_servo42d", "gt2_pulley_20t", "wrist_roll_belt"}


def test_stack():
    p = D.pancake
    # the pancake centred on the pitch axis' station, 1+ mm over the plate's free face (its pocket's floor); its 20T's
    # band level with the 40T's
    assert S["z_motor_face"] + S["z_motor_rear"] == 0.0 and S["z_motor_face"] - S["z_motor_rear"] == p.body_length
    assert -p.body_w / 2.0 - S["x_free"] >= 1.0 and S["x_free"] - S["x_coupler"] == DEFAULT.plate.thickness
    assert S["z_20t"] + D.t20_hub == S["z_ring_mid"] == S["z_motor_face"] + D.t20
    # the stations, the band back: its rear flange, run_gap, bearing 2 (the boss's top), the lip, bearing 1; the hubs
    # meet in the lip's middle; the face in front of the band
    assert S["z_ring_flange_1"] == pytest.approx(S["z_ring"] - D.ring_flange_t)
    assert S["z_boss"] == pytest.approx(S["z_ring_flange_1"] - D.run_gap) == pytest.approx(S["z_bearing_2"] + D.bearing_width)
    assert S["z_lip"] == pytest.approx(S["z_bearing_2"] - D.lip) and S["z_bearing_1"] == pytest.approx(S["z_lip"] - D.bearing_width)
    assert S["z_meet"] == pytest.approx(S["z_lip"] + D.lip / 2.0) and S["z_face"] == pytest.approx(S["z_ring_end"] + D.face_t)
    assert S["z_front"] < S["z_boss"]                                   # the bearing boss stands on the plate's face
    # the bay: room to slide the shaft in, the plate's coupler face 4+ mm under it, bearing 1 through it with the shaft
    assert S["z_bay"] == pytest.approx(S["z_shaft_end"] - (S["z_meet"] - S["z_bearing_1"]) - D.bay_clear)
    assert -D.bay_r - S["x_coupler"] >= 4.0 and D.bay_r > D.bearing_od / 2.0 + 1.0 and D.bay_r >= D.stop_lug_r[1] + 1.0
    # the pocket: round the body and the tension travel, clear of the tower's seats
    _, y1, z0, _ = motor_pocket(DEFAULT)
    assert y1 - S["y_motor"] == p.body_w / 2.0 + D.pad_slot_len / 2.0 + D.pocket_clear
    assert -y1 >= (D.bearing_od + D.seat_add) / 2.0 + 2.0 and -y1 >= D.bay_r + 1.5
    # behind it no wall: the MKS board's window - its housing + the tension travel + board_clear, from in front of the
    # pancake's rear face (where the housing starts) out through the frame's rear - 1.5+ off the shaft's bay too; the
    # board's back behind the frame
    w0, w1, wz0, wz1 = board_window(DEFAULT)
    assert (D.board_w, D.board_stack) == (PARAMS.MKS_SERVO42D_W, PARAMS.MKS_SERVO42D_STACK) and D.board_w > p.body_w
    half = D.board_w / 2.0 + D.pad_slot_len / 2.0 + D.board_clear
    assert w1 - S["y_motor"] == pytest.approx(half) and S["y_motor"] - w0 == pytest.approx(half)
    assert -w1 >= D.bay_r + 1.5 and w1 > y1
    assert wz0 == S["z_rear"] < z0 < S["z_motor_rear"] < wz1 and S["z_board_back"] < S["z_rear"]


def test_the_belt_is_a_stock_length_at_the_slots_middle():
    """2 : 1, the stock length's centre distance with the pancake at its slots' middle (no idler yet:
    docs/open_issues.md); the 40T's band in front of the boss."""
    assert D.belt in STANDARD_2GT_LENGTHS and PARAMS.WRIST_ROLL_BELT_LENGTH == D.belt
    assert belt_length(roll_belt_loop(DEFAULT)) == pytest.approx(D.belt, abs=0.01)
    assert PARAMS.WRIST_ROLL_RATIO == D.ring_teeth / 20 == 2.0
    assert D.ring_core_dia < pulley_od(D.ring_teeth) - 2 * 0.75   # the band's annulus round its core (the root)


def test_the_rotor_walls():
    """Inside the band's core the clamp's M3 between the cable bore and the core (1+ mm each side); in the face the
    connector's nut pockets clear of the clamp's counterbores; the clamp's nut pockets inside the shaft's flange."""
    r_core, r_bore, r_bolt = D.ring_core_dia / 2.0, D.bore / 2.0, D.bolt_dia / 2.0
    for x, y in clamp_points(DEFAULT):
        r = math.hypot(x, y)
        assert r + r_bolt <= r_core - 1.0 and r - r_bolt >= r_bore + 1.0
    head = D.clamp_screw.head_dia / 2.0 + D.clamp_head_clear
    r_in, r_out, z0, z1 = face_nut_pocket(DEFAULT)
    assert r_out > D.face_dia / 2.0 and S["z_ring_end"] < z0 < z1 < S["z_face"]   # open to the rim, a floor and a roof
    half = D.nut.af / 2.0 + D.nut_fit
    for bx, by in connector_bolt_points_module(DEFAULT):
        a = math.atan2(by, bx)
        for cx, cy in clamp_points(DEFAULT):
            # the clamp head's counterbore off the nut pocket's slot - a rectangle from r_in outward, half wide - in the
            # slot's frame (u along it, v across)
            u, v = cx * math.cos(a) + cy * math.sin(a), -cx * math.sin(a) + cy * math.cos(a)
            d = math.hypot(max(r_in - u, 0.0), max(abs(v) - half, 0.0))
            assert d - head >= 1.0, (bx, by, cx, cy)
    r_in, r_out, _, _ = clamp_nut_pocket(DEFAULT)
    assert r_out <= D.collar_od / 2.0 - 1.0


def test_the_connector_turns_with_the_gripper():
    """connector_turn_deg: the gripper's connector's plate x axis in the module's x-y plane, as the placements put it."""
    host = P.location("wrist_link#1", "world")
    conn = host.inverse() * P.location("gripper_j3_connector#1", "world")
    m = to_location(module_frame_in_host(DEFAULT)).inverse() * conn
    x = (m * Location((1.0, 0.0, 0.0))).position - m.position
    y = (m * Location((0.0, 1.0, 0.0))).position - m.position
    assert math.degrees(math.atan2(x.Y, x.X)) == pytest.approx(D.connector_turn_deg, abs=1e-3) and abs(x.Z) < 1e-6
    assert (y - Vector(0, 0, -1)).length < 1e-6                     # its hub-side +Y back along R', toward the pulley
    assert (m.position.X, m.position.Y) == pytest.approx((0.0, 0.0), abs=1e-3)
    assert m.position.Z == pytest.approx(S["z_connector"] + DEFAULT.connector.plate_t, abs=1e-3)   # its plate on the hub


@pytest.fixture(scope="module")
def module():
    return built.model(M.wrist_roll_drive)


@pytest.fixture(scope="module")
def frame():
    """wrist_link (the drive's frame) in the module frame."""
    return built.part("wrist_link").moved(to_location(module_frame_in_host(DEFAULT)).inverse())


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
    assert module.label == "wrist_roll_drive"
    assert module_tints(module, M.TINT) == {True: 6, False: 2}   # 2 bearings, the pancake, its board, its 20T, the belt / shaft, pulley


@pytest.mark.slow
def test_bearings_press_on_the_journals_and_slip_in_the_seats(module, frame):
    press = math.pi / 4.0 * ((D.bearing_bore + D.journal_add) ** 2 - D.bearing_bore ** 2) * D.bearing_width   # ~99 mm^3
    for label, hub, z in (("bearing_6806:wrist_roll_1", "wrist_roll_shaft", S["z_bearing_1"]),
                          ("bearing_6806:wrist_roll_2", "wrist_roll_pulley", S["z_bearing_2"])):
        b = _leaf(module, label)
        assert interference(b, frame) < 1.0, label
        assert 0.9 * press <= interference(b, _leaf(module, hub)) <= press + 0.5, label
        bb = b.bounding_box()
        assert math.isclose(bb.min.Z, z, abs_tol=1e-6) and math.isclose(bb.max.Z, z + D.bearing_width, abs_tol=1e-6)


@pytest.mark.slow
def test_everything_else_is_clean(module, frame):
    """Every pair in the module and every row against the frame below 1 mm^3, but the bearings' presses and the board's
    screws in the pancake (KIT_OVERLAP)."""
    press = {("bearing_6806:wrist_roll_1", "wrist_roll_shaft"), ("bearing_6806:wrist_roll_2", "wrist_roll_pulley")}
    kit = {"mks_servo42d:wrist_roll", "nema17_pancake:wrist_roll"}
    names = [c.label for c in module.children]
    for i, a in enumerate(names):
        assert interference(_leaf(module, a), frame) < 1.0, f"{a} x wrist_link"
        for b in names[i + 1:]:
            if (a, b) in press or (b, a) in press:
                continue
            vol = interference(_leaf(module, a), _leaf(module, b))
            if {a, b} == kit:
                assert 0.9 * KIT_OVERLAP <= vol <= KIT_OVERLAP, f"{a} x {b}: {vol:.1f} mm^3 (designed ~{KIT_OVERLAP})"
                continue
            assert vol < 1.0, f"{a} x {b}: {vol:.1f} mm^3"


@pytest.mark.slow
def test_the_rotor_turns_clear_of_the_frame(module, frame):
    """The pulley and the shaft keep run_gap off the frame (the lug off the post at the zero pose); the two hubs meet face
    to face in the lip."""
    for label in ("wrist_roll_pulley", "wrist_roll_shaft"):
        gap = closest_points(_leaf(module, label), frame).distance
        assert gap >= RUN_GAP - 0.01, f"{label}: {gap:.2f} mm off the frame"
    pulley, shaft = _leaf(module, "wrist_roll_pulley"), _leaf(module, "wrist_roll_shaft")
    assert math.isclose(pulley.bounding_box().min.Z, S["z_meet"], abs_tol=1e-6) and math.isclose(shaft.bounding_box().max.Z, S["z_meet"], abs_tol=1e-6)
    assert closest_points(pulley, shaft).distance < 1e-6 and interference(pulley, shaft) < 1e-3


@pytest.mark.slow
def test_the_pancake_and_its_board_slide_in_from_the_open_face(module, frame):
    """The pancake and its board go in from the frame's open +X face motor_drop back - its pilot behind the plate, its
    shaft along the slit, the board through the window - and forward into the pilot slot: clear of the frame from
    outside it to the seat, in 2 mm steps, and forward."""
    kit = [_leaf(module, label) for label in ("nema17_pancake:wrist_roll", "mks_servo42d:wrist_roll")]
    out = D.frame_r - S["x_free"] + 2.0
    for k in range(int(out // 2.0) + 1):
        for part in kit:
            vol = interference(part.moved(Location((2.0 * k, 0.0, -D.motor_drop))), frame)
            assert vol < 0.5, f"{part.label} {2.0 * k:.0f} mm out, {D.motor_drop} back: {vol:.1f} mm^3 in the frame"
    for k in range(11):
        for part in kit:
            vol = interference(part.moved(Location((0.0, 0.0, -D.motor_drop * k / 10.0))), frame)
            assert vol < 0.5, f"{part.label} {D.motor_drop * k / 10.0:.2f} back: {vol:.1f} mm^3 in the frame"
    # and the belt's tension travel: the pancake clear of the frame at both ends of its slots, the board's housing
    # board_clear off the window's sides
    pancake, board = kit
    for dy in (-D.pad_slot_len / 2.0, D.pad_slot_len / 2.0):
        assert interference(pancake.moved(Location((0.0, dy, 0.0))), frame) < 0.5
        gap = closest_points(board.moved(Location((0.0, dy, 0.0))), frame).distance
        assert gap >= D.board_clear - 0.01, f"the board {gap:.2f} mm off the frame at {dy:+.1f}"


@pytest.mark.slow
def test_the_board_sits_on_the_pancake(module, frame):
    """The MKS board on the pancake's rear face, centred on its axis, turned with it (its motor plug, the board frame's -Y,
    toward the open +X face, as the pancake's); its back board_stack behind that face, the frame's rear in front of it;
    off the plate's free face."""
    board = _leaf(module, "mks_servo42d:wrist_roll")
    bb = board.bounding_box()
    assert math.isclose(bb.min.Z, S["z_board_back"], abs_tol=1e-6) and S["z_board_back"] < S["z_rear"]
    assert math.isclose((bb.min.Y + bb.max.Y) / 2.0, S["y_motor"], abs_tol=1e-6)
    assert math.isclose(bb.min.X, S["x_motor"] - D.board_w / 2.0, abs_tol=1e-6) and bb.min.X > S["x_free"]
    a = math.radians(D.motor_spin_deg)
    (x0, y0, _, x1, y1, _), _ = PARAMS.MKS_SERVO42D_HEADERS["motor"]
    cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
    want = (S["x_motor"] + cx * math.cos(a) - cy * math.sin(a), S["y_motor"] + cx * math.sin(a) + cy * math.cos(a))
    assert want[0] > S["x_motor"] + D.board_w / 4.0                    # toward the open face
    assert any(math.dist(want, ((b.min.X + b.max.X) / 2.0, (b.min.Y + b.max.Y) / 2.0)) < 1e-6
               for b in (s.bounding_box() for s in board.solids()))
