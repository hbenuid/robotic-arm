"""The belt joints' motor mounts (lib/mounts.py, materialised into reference/placements.json by
tools/reference/mount_placements.py): each NEMA 17 x 40 sits on the pad its SolidWorks host carries with
its shaft on the joint axis, its MKS board on its rear face, and nothing runs into the neighbours."""
import math

import pytest
from build123d import GeomType, Location, Vector

from assemblies import cycloidal_drive
from assemblies._occurrences import place_world
from lib import mounts
from lib.cycloidal import DEFAULT_CONFIG
from lib import placements as P
from lib import params as PARAMS
from lib.datum import BASE_BOTTOM_Y, to_location
from lib.models import raw
from robot import frames as F
from tests.cycloidal.helpers import interference

MOTORS = [m for m in mounts.MOUNTS if m.part in mounts.MOTORS]
BOARDS = {m.host: m for m in mounts.MOUNTS if m.part == mounts.BOARD}


def _world(key: str) -> Location:
    return P.location(key, "world")


def _dir(loc: Location, local=(0.0, 0.0, 1.0)) -> Vector:
    return (loc * Location(local)).position - loc.position


def test_every_motor_has_its_board_and_they_sit_in_the_same_link():
    assert len(MOTORS) == 3 and set(BOARDS) == {m.key for m in MOTORS}
    for m in MOTORS:
        b = BOARDS[m.key]
        assert (b.link, b.joint) == (m.link, m.joint)
        body = PARAMS.CYCLOIDAL_MOTOR_BODY_LEN if m.part == mounts.MOTOR_48 else PARAMS.NEMA17_40_BODY_LEN
        assert b.frame == mounts.board_frame(body) == ((0.0, 0.0, -body), (0.0, 0.0, 0.0))
        assert m.key in F.LINKS[m.link] and b.key in F.LINKS[m.link], m.link
        assert P.OCCURRENCES[m.host]["kind"] == "part" and "mount" not in P.OCCURRENCES[m.host]   # a SolidWorks host


@pytest.mark.parametrize("m", MOTORS, ids=[m.joint for m in MOTORS])
def test_motor_shaft_is_on_its_joint_axis(m):
    """+Z of the motor frame (the shaft) parallel to the joint axis, the mounting face on the host's pad face."""
    joint = F.JOINT_BY_NAME[m.joint]
    z = _dir(_world(m.key)).normalized()
    assert abs(abs(z.dot(Vector(*joint.axis_w))) - 1.0) < 1e-6, (m.key, tuple(z))
    # the board is the motor frame shifted to the rear face
    board = _world(BOARDS[m.key].key)
    want = _world(m.key) * to_location(BOARDS[m.key].frame)
    assert (board.position - want.position).length < 1e-4 and (_dir(board) - _dir(want)).length < 1e-6


@pytest.mark.slow
@pytest.mark.parametrize("m", MOTORS, ids=[m.joint for m in MOTORS])
def test_mounting_face_lies_on_the_host_pad(m):
    """The motor's mounting face (its local z=0 plane) coincides with a planar face of the host at the pattern
    centre: the base plate's underside, j1_link's pad, j2_link's web."""
    world = _world(m.key)
    origin, z = world.position, _dir(world).normalized()
    host = place_world(P.OCCURRENCES[m.host]["part"], m.host)
    faces = [f for f in host.faces().filter_by(GeomType.PLANE)
             if abs(abs(f.normal_at().dot(z)) - 1.0) < 1e-6 and abs((f.center() - origin).dot(z)) < 1e-3]
    assert faces, f"{m.key}: no planar face of {m.host} through the mounting plane"
    # ... and the shaft points away from the host's material: the pad face's outward normal is -Z of the motor
    assert any(f.normal_at().dot(z) < -0.99 for f in faces), f"{m.key}: the shaft points into {m.host}"


@pytest.mark.slow
def test_motors_and_boards_clear_their_neighbours():
    """Interference budget (mm^3) against the host, the caps, the pulleys and the placed drive: zero everywhere
    (the Ø22 pilot boss used to stand in j2_link's Ø20 central slot - the parametric forearm's slot is 22.3 wide)."""
    neighbours = ["base#1", "j1_coupler#1", "j1_link#1", "j1_cap#1", "j2_link#1", "j2_cap_1#1", "j2_cap_2#1",
                  "gt2_pulley_90t#1", "gt2_pulley_90t#2", "j3_coupler#1", "j3_coupler#2", "wrist_link#1"]
    shapes = {k: place_world(P.OCCURRENCES[k]["part"], k) for k in neighbours}
    shapes["cycloidal_drive#1"] = raw(cycloidal_drive.cycloidal_drive).moved(_world("cycloidal_drive#1"))
    budget = {}
    for m in mounts.MOUNTS:
        part = place_world(m.part, m.key)
        for key, other in shapes.items():
            vol = interference(part, other)
            assert vol <= budget.get((m.key, key), 1.0), f"{m.key} x {key}: {vol:.1f} mm^3"
    # the three motors and boards do not touch each other either
    placed = [(m.key, place_world(m.part, m.key)) for m in mounts.MOUNTS]
    for i, (ka, a) in enumerate(placed):
        for kb, b in placed[i + 1:]:
            if mounts.BY_KEY[kb].host == ka or mounts.BY_KEY[ka].host == kb:
                continue   # a board on its motor: the kit's screws run through the motor by design
            assert interference(a, b) < 1.0, f"{ka} x {kb}"


@pytest.mark.slow
def test_base_motor_stack_hangs_below_the_base_by_the_documented_amount():
    """Under the base plate the 48 mm motor + board stack (62.1) reaches BASE_MOTOR_STACK_PROUD below the base's
    mounting face (56 mm of depth) - a known, documented protrusion, not a silent one."""
    lowest = min(place_world(m.part, m.key).bounding_box().min.Y for m in mounts.MOUNTS if m.joint == "base_yaw")
    assert math.isclose(PARAMS.BASE_MOTOR_PATTERN_CENTRE[1] - lowest, PARAMS.CYCLOIDAL_MOTOR_BODY_LEN + PARAMS.MKS_SERVO42D_STACK, abs_tol=0.05)
    assert math.isclose(BASE_BOTTOM_Y - lowest, PARAMS.BASE_MOTOR_STACK_PROUD, abs_tol=0.05), BASE_BOTTOM_Y - lowest


@pytest.mark.slow
def test_belt_pulley_planes_are_reachable():
    """Each belt motor's shaft (22 mm past its mounting face) reaches the plane of the 90T it drives: the 90T's
    bore-axis station lies between the mounting face and the shaft tip along the motor axis."""
    for m, pulley_key in (("nema17_40mm#2", "gt2_pulley_90t#1"), ("nema17_40mm#3", "gt2_pulley_90t#2")):
        world = _world(m)
        z = _dir(world).normalized()
        pulley = place_world("gt2_pulley_90t", pulley_key)
        bb = pulley.bounding_box()
        centre = Vector((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2)
        station = (centre - world.position).dot(z)
        assert 0.0 < station < DEFAULT_CONFIG.motor.shaft_length, f"{m}: the 90T's mid-plane is {station:.1f} mm along the shaft"


def test_wrist_pitch_slide_position_is_inside_the_slots_and_the_cap_window():
    lo, hi = PARAMS.J2_MOTOR_SLIDE_RANGE
    assert lo < PARAMS.J2_MOTOR_SLIDE_X < hi
    m = mounts.BY_KEY["nema17_40mm#3"]
    assert m.frame[0][0] == PARAMS.J2_MOTOR_SLIDE_X and m.frame[0][2] == PARAMS.J2_MOTOR_WEB_FACE_Z


def test_the_base_takes_the_48mm_motor_and_the_links_the_40mm_one():
    by_joint = {m.joint: m.part for m in mounts.MOUNTS if m.part in mounts.MOTORS}
    assert by_joint == {"base_yaw": "nema17_48mm", "elbow_pitch": "nema17_40mm", "wrist_pitch": "nema17_40mm"}
