"""The wrist roll drive's fasteners (assemblies/wrist_roll_drive.py): the pancake's 4x M3 into wrist_link's plate
(wrist_roll_motor_screws), the rotor clamp's 4x M3 + nuts (wrist_roll_clamp_screws / _nuts: the forearm roll's,
tests/forearm/test_roll_clamp.py, one joint further out) and gripper_j3_connector's 4x M3 + nuts on the pulley's face
(wrist_roll_connector_screws / _nuts). Purchased parts with no reference (lib/reference.py NO_REFERENCE), which keep no
reference file - THIS file locks their numbers (the pieces, the modelled volume and the bounding box, measured on the
build and written in) and what they are, where they sit, and the counterbores the pancake's heads sit in.

The pancake's heads: wrist_link's plate (pad_t) carries a counterbore round each tension slot in its front face
(WristRollParams pad_cbore_*), the heads on their floors and flush with the face - the plate thickened forward to take
them, where bosses round the slots stood proud of the face the frame prints on. Moving the pancake back to thicken the
plate instead was built and dropped (twice): the plate then stood in the way of the wrist 90T's two front M4 nuts,
which go into their channels from the pancake's pocket - the last test holds that way open."""
import math

import pytest
from build123d import Axis, Location
from cad.build123d import closest_points

import parts
from assemblies import gripper as G
from assemblies import wrist_roll_drive as M
from assemblies._occurrences import row_location
from lib import params as PARAMS
from lib import reference as R
from lib.datum import to_location
from lib.fasteners import nut_volume, shcs_volume
from lib.forearm import DEFAULT as FOREARM
from lib.geom import hex_prism
from lib.wrist import DEFAULT
from lib.wrist.layout import (
    clamp_points,
    connector_bolt_points_module,
    gripper_frame_in_module,
    module_frame_in_host,
    pad_bolt_points,
    pulley_bolt_points,
    stack_positions,
)
from tests import built
from tests.helpers import interference, is_inside

D, C, S = DEFAULT.roll, DEFAULT.connector, stack_positions(DEFAULT)
MOTOR, CLAMP, CLAMP_NUTS = "wrist_roll_motor_screws", "wrist_roll_clamp_screws", "wrist_roll_clamp_nuts"
CONNECTOR, CONNECTOR_NUTS = "wrist_roll_connector_screws", "wrist_roll_connector_nuts"
SCREWS, NUTS = (MOTOR, CLAMP, CONNECTOR), (CLAMP_NUTS, CONNECTOR_NUTS)
# name -> (solids, volume mm^3, bbox min, bbox size): the build's own numbers
LOCKS = {
    MOTOR: (4, 483.148, (-18.25, -18.25, -3.0), (36.5, 36.5, 11.0)),
    CLAMP: (4, 1246.555, (-8.97254, -8.97254, -3.0), (17.945079, 17.945079, 38.0)),
    CLAMP_NUTS: (4, 183.635, (-9.289766, -9.289766, 0.0), (18.579532, 18.579532, 2.4)),
    CONNECTOR: (4, 539.697, (-18.813994, -12.601406, -3.0), (37.627989, 25.202811, 13.0)),
    CONNECTOR_NUTS: (4, 183.635, (-18.897885, -13.021937, 0.0), (37.79577, 26.043873, 2.4)),
}
# name -> (screw size, length under the head, order line's size, mass constant)
SCREW_SPECS = {
    MOTOR: (D.pad_screw, D.pad_screw_len, "M3 x 8", PARAMS.WRIST_ROLL_MOTOR_SCREWS_MASS_G),
    CLAMP: (D.clamp_screw, D.clamp_screw_len, "M3 x 35", PARAMS.WRIST_ROLL_CLAMP_SCREWS_MASS_G),
    CONNECTOR: (C.screw, C.bolt_len, "M3 x 10", PARAMS.WRIST_ROLL_CONNECTOR_SCREWS_MASS_G),
}
NUT_MASSES = {CLAMP_NUTS: PARAMS.WRIST_ROLL_CLAMP_NUTS_MASS_G, CONNECTOR_NUTS: PARAMS.WRIST_ROLL_CONNECTOR_NUTS_MASS_G}


@pytest.fixture(scope="module")
def module():
    return built.model(M.wrist_roll_drive)


@pytest.fixture(scope="module")
def frame():
    """wrist_link (the drive's frame) in the module frame."""
    return built.part("wrist_link").moved(to_location(module_frame_in_host(DEFAULT)).inverse())


@pytest.fixture(scope="module")
def connector():
    """gripper_j3_connector in the drive's module frame, where the gripper module puts it."""
    (row,) = [placement for part, _, placement in G.OCCURRENCES if part == "gripper_j3_connector"]
    return built.part("gripper_j3_connector").moved(to_location(gripper_frame_in_module(DEFAULT)) * row_location(row))


def _leaf(module, label):
    return next(c for c in module.children if c.label == label)


def test_they_are_purchased_parts_with_no_reference_in_the_drive():
    for name in SCREWS + NUTS:
        mod = parts.load(name)
        assert parts.GROUPS[name] == "wrist" and parts.bought(name) and mod.PURCHASE_QTY == LOCKS[name][0] == 4, name
        assert R.NO_REFERENCE[name] == f"parts/wrist/{name}.py:_envelope()" and name not in parts.unplaced(), name
    # the pancake's on the stator (wrist_pitch_link), the rotor's - the clamp's and the connector's - turn with it
    assert MOTOR in M.BODIES["stator"] and {CLAMP, CLAMP_NUTS, CONNECTOR, CONNECTOR_NUTS} <= M.BODIES["rotor"]
    # one row each: the screws turned over (the heads up R') on their seats, the nuts at their far faces' station
    rows = {part: (role, pos) for part, role, pos in M.OCCURRENCES if part in SCREWS + NUTS}
    assert rows == {
        MOTOR: (None, ((S["x_motor"], S["y_motor"], S["z_pad_seat"]), (180.0, 0.0, 0.0))),
        CLAMP: (None, ((0.0, 0.0, S["z_clamp_head"]), (180.0, 0.0, 0.0))),
        CLAMP_NUTS: (None, (0.0, 0.0, S["z_clamp_nut"])),
        CONNECTOR: (None, ((0.0, 0.0, S["z_connector_head"]), (180.0, 0.0, 0.0))),
        CONNECTOR_NUTS: (None, (0.0, 0.0, S["z_face_nut"])),
    }


def test_the_screws_and_nuts():
    """ISO 4762 M3 at their lengths, ISO 4032 M3; each pattern on its holes (the screws' mirrored in y: their rows turn them
    over about X); the masses the modelled steel."""
    for name, (size, length, spec, mass) in SCREW_SPECS.items():
        mod = parts.load(name)
        assert mod.SCREW == size == PARAMS.M3_SHCS and mod.LENGTH == length, name
        assert mod.PURCHASE_SPEC == f"{spec} socket head cap screw (ISO 4762)", name
        assert mod.MASS_G == mass == pytest.approx(PARAMS.STEEL_DENSITY * 4 * shcs_volume(size, length)), name
    for name, mass in NUT_MASSES.items():
        mod = parts.load(name)
        assert mod.NUT == D.nut == PARAMS.M3_NUT and mod.PURCHASE_SPEC == "M3 hex nut (ISO 4032)", name
        assert mod.MASS_G == mass == pytest.approx(PARAMS.STEEL_DENSITY * 4 * nut_volume(D.nut)), name
    assert (D.pad_screw_len, D.clamp_screw_len, C.bolt_len) == (8.0, 35.0, 10.0)
    assert sorted(parts.load(MOTOR).POINTS) == sorted((x - S["x_motor"], y - S["y_motor"]) for x, y in pad_bolt_points(DEFAULT))
    assert sorted((x, -y) for x, y in parts.load(CLAMP).POINTS) == sorted(clamp_points(DEFAULT)) == sorted(parts.load(CLAMP_NUTS).POINTS)
    assert (sorted((x, -y) for x, y in parts.load(CONNECTOR).POINTS) == sorted(connector_bolt_points_module(DEFAULT))
            == sorted(parts.load(CONNECTOR_NUTS).POINTS))


def test_the_stations():
    """The pancake's: the heads on their counterbores' floor, flush with the plate's front face, as long as the plate
    under them and the other motors' 4 of thread in the pancake (M3 x 8), the counterbores the other motors'
    (pad_cbore_dia). The clamp's:
    the heads on the counterbores' floor, clamp_head_clear under the pulley's face, the tips at the shaft's rear face,
    each nut 2 pitches short of its tip. The connector's: the heads on its flange's plate side, under its plate by the
    head room's clearance, the tips at the end of their holes in the pulley (z_ring_end), the nuts face_nut_depth under
    the face."""
    head = PARAMS.M3_SHCS.head_h
    assert S["z_front"] - S["z_pad_seat"] == D.pad_cbore_depth == head
    assert S["z_pad_seat"] - D.pad_screw_len == pytest.approx(S["z_motor_face"] - D.pad_screw_thread)
    assert D.pad_screw_thread == PARAMS.J1_MOTOR_SCREW_THREAD == 4.0
    assert D.pad_screw_len == D.pad_t - D.pad_cbore_depth + 4.0 and D.pad_t - D.pad_cbore_depth == 4.0
    assert D.pad_cbore_dia == FOREARM.drive.motor_holes.pocket_dia == pytest.approx(5.7)
    assert D.pad_cbore_dia > D.pad_screw.head_dia
    assert math.isclose(S["z_face"] - (S["z_clamp_head"] + head), D.clamp_head_clear)
    assert math.isclose(S["z_clamp_head"] - D.clamp_screw_len, S["z_shaft_end"])
    assert math.isclose(S["z_clamp_nut"] - S["z_shaft_end"], 2 * PARAMS.M3_PITCH)
    assert math.isclose(S["z_clamp_nut"] + D.nut.h, S["z_clamp_nut_seat"])
    assert math.isclose(S["z_connector_head"], S["z_face"] + C.hub_flange_t)
    assert math.isclose(S["z_connector"] - (S["z_connector_head"] + head), C.hub_head_room - head)   # 0.25 under its plate
    assert math.isclose(S["z_connector_head"] - C.bolt_len, S["z_ring_end"])
    assert math.isclose(S["z_face"] - S["z_face_nut_seat"], D.face_nut_depth) and S["z_ring_end"] < S["z_face_nut"]


@pytest.mark.slow
@pytest.mark.parametrize("name", list(LOCKS))
def test_the_build_matches_its_numbers(name):
    solids, volume, lo, size = LOCKS[name]
    shape = built.part(name)
    assert len(shape.solids()) == solids
    assert R.solid_volume(shape) == pytest.approx(volume, abs=0.01)
    assert R.bbox_min(shape) == pytest.approx(lo, abs=1e-3) and R.bbox_size(shape) == pytest.approx(size, abs=1e-3)
    one = shcs_volume(*SCREW_SPECS[name][:2]) if name in SCREW_SPECS else nut_volume(D.nut)
    assert R.solid_volume(shape) == pytest.approx(solids * one, rel=1e-6)


@pytest.mark.slow
def test_in_place_each_sits_on_its_seat(module, frame, connector):
    """The placed patterns between their stations; the pancake's heads bear on their counterbores' floor, the clamp's on the
    pulley and its nuts on the shaft, the connector's heads on its flange and its nuts on the pulley; none runs into
    anything (the board's kit screws in the pancake's thread: tests/wrist/test_roll_drive.py FRONT_OVERLAP)."""
    motor, clamp, clamp_nuts = _leaf(module, MOTOR), _leaf(module, CLAMP), _leaf(module, CLAMP_NUTS)
    conn, conn_nuts = _leaf(module, CONNECTOR), _leaf(module, CONNECTOR_NUTS)
    pulley, shaft = _leaf(module, "wrist_roll_pulley"), _leaf(module, "wrist_roll_shaft")
    spans = {motor: (S["z_pad_seat"] - D.pad_screw_len, S["z_front"]), clamp: (S["z_shaft_end"], S["z_clamp_head"] + 3.0),
             clamp_nuts: (S["z_clamp_nut"], S["z_clamp_nut_seat"]), conn: (S["z_ring_end"], S["z_connector_head"] + 3.0),
             conn_nuts: (S["z_face_nut"], S["z_face_nut_seat"])}
    for part, (z0, z1) in spans.items():
        bb = part.bounding_box()
        assert (bb.min.Z, bb.max.Z) == pytest.approx((z0, z1), abs=1e-6), part.label
    for part, seat in ((motor, frame), (clamp, pulley), (clamp_nuts, shaft), (conn, connector), (conn_nuts, pulley)):
        assert closest_points(part, seat).distance < 0.01, part.label
    for part in (motor, clamp, clamp_nuts, conn, conn_nuts):
        assert interference(part, frame) < 0.01 and interference(part, connector) < 0.01, part.label
        for other in module.children:
            if other is not part and {part.label, other.label} != {MOTOR, "mks_servo42d:wrist_roll"}:
                assert interference(part, other) < 0.01, (part.label, other.label)


@pytest.mark.slow
def test_the_pancakes_heads_travel_with_it_in_the_counterbores(module, frame):
    """The heads move with the pancake along its slots (the belt's tension): at both ends of the travel the screws clear
    the frame - the counterbores slots as the holes are."""
    motor = _leaf(module, MOTOR)
    for dy in (-D.pad_slot_len / 2.0, D.pad_slot_len / 2.0):
        assert interference(motor.moved(Location((0.0, dy, 0.0))), frame) < 0.01, dy


@pytest.mark.slow
def test_the_counterbores_in_the_front_face(frame):
    """Each counterbore round a slot, from the front face down to the heads' seat (z_pad_seat): pad_cbore_dia wide along
    the slot (the head travels in it), the plate round it; nothing stands proud of the face (the frame prints flat on it);
    under the head the plate whole beyond the slot's M3 clearance - the head bears on it."""
    r, half, z = D.pad_cbore_dia / 2.0, D.pad_slot_len / 2.0, S["z_pad_seat"] + D.pad_cbore_depth / 2.0
    for x, y in pad_bolt_points(DEFAULT):
        for dy in (-half, 0.0, half):
            assert not is_inside(frame, x, y + dy, z) and not is_inside(frame, x + r - 0.05, y + dy, z), (x, y, dy)
            assert is_inside(frame, x + r + 0.5, y + dy, z) and is_inside(frame, x - r - 0.5, y + dy, z), (x, y, dy)
            assert not is_inside(frame, x + r + 0.5, y + dy, S["z_front"] + 0.05), (x, y, dy)
        assert is_inside(frame, x, y + half + r + 0.5, z) and not is_inside(frame, x, y + half + r - 0.05, z)
        assert is_inside(frame, x + D.pad_bolt_dia / 2.0 + 0.5, y, S["z_pad_seat"] - 0.05)       # the plate under the head
        assert not is_inside(frame, x + D.pad_bolt_dia / 2.0 - 0.05, y, S["z_pad_seat"] - 0.05)   # the slot through it
    assert frame.bounding_box().max.Z == pytest.approx(S["z_front"], abs=1e-6)


@pytest.mark.slow
def test_the_wrist_90t_nuts_still_go_in_from_the_pocket():
    """The wrist 90T's 4 M4 nuts go into their hex channels in the plate from the pancake's pocket, before the pancake:
    each nut's hex on past its channel's end, a nut's height and a millimetre, keeps out of wrist_link but for the graze
    the two in front of the pitch axis have always had at the pocket's front wall (0.27 of their hex past it). Moving the
    pancake back to thicken that wall for counterbores put 101 mm^3 of it in each one's way."""
    f = DEFAULT.flange
    link = built.part("wrist_link")   # its part frame: the plate z 0..thickness, the pocket under z 0
    corner = math.radians(f.pulley_bolt_deg + f.nut_corner_deg)
    run = PARAMS.M4_NUT.h + 1.0
    for x, y in pulley_bolt_points(DEFAULT):
        path = Location((x, y, -f.channel_past - run)) * hex_prism(f.nut_af, corner, run)
        hit = link.intersect(path)
        vol = sum(s.volume for s in hit.solids()) if hit else 0.0
        assert vol < 1.0, f"the nut at ({x:.2f}, {y:.2f}): {vol:.2f} mm^3 of wrist_link in its way in"


@pytest.mark.slow
@pytest.mark.parametrize("name, host", [(CLAMP_NUTS, "wrist_roll_shaft"), (CONNECTOR_NUTS, "wrist_roll_pulley")])
def test_the_pockets_hold_the_nuts_from_turning(module, name, host):
    """Each clamp nut in its closed hex pocket, a flat toward the axis as the pocket's; each connector nut a corner radial
    in its pocket's slot, its flats along the slot's sides: 20 degrees about its own axis runs its corners into the part
    (the clamp's pockets are a press, the connector's nut_fit wider either side)."""
    nuts, part = _leaf(module, name), _leaf(module, host)
    points = parts.load(name).POINTS
    for solid, (x, y) in zip(sorted(nuts.solids(), key=lambda s: math.atan2(s.center().Y, s.center().X)),
                             sorted(points, key=lambda p: math.atan2(p[1], p[0])), strict=True):
        assert math.isclose(solid.center().X, x, abs_tol=1e-6) and math.isclose(solid.center().Y, y, abs_tol=1e-6)
        turned = solid.rotate(Axis((x, y, 0.0), (0.0, 0.0, 1.0)), 20.0)
        assert interference(turned, part) > 0.1, (name, x, y)
