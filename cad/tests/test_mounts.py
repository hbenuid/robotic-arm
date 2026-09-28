"""The belt joints' mounts (lib/mounts.py, materialised into reference/placements.json by
tools/reference/mount_placements.py): each NEMA 17 sits on the pad its printed host carries (a SolidWorks link, or
the base's bolt-on motor mount, itself bolted to the base by 4x M4 into nuts pressed into the base) with its shaft
parallel to the joint axis, its MKS board on its rear face; each joint's 6806 pair stands on the lip of its bore
with the coupler's shoulder on the upper inner ring and the re-seated 90T's ring under the lower one; each 90T clamped
by its 4x M4 screws + nuts; j1_coupler stands on the base_yaw thrust bearing in the base's groove; and nothing runs
into the neighbours but the nuts' designed press in their pockets."""
import math

import pytest
from build123d import Align, Cylinder, GeomType, Location, Pos, Vector

import parts
from assemblies import cycloidal_drive, forearm_roll_drive
from assemblies._occurrences import place_world
from lib import mounts
from lib import params as PARAMS
from lib import placements as P
from lib.base import DEFAULT as BASE
from lib.base.params import RING_CLEAR
from lib.bearings import (
    BEARING_6806_SHOULDER_DIA,
    BEARING_6806_WIDTH,
    THRUST_BORE,
    THRUST_CAGE_WIDTH,
    THRUST_OD,
    THRUST_WASHER_WIDTH,
)
from lib.coupler import DEFAULT as COUPLER
from lib.cycloidal import DEFAULT_CONFIG
from lib.datum import BASE_BOTTOM_Y, to_location
from lib.fasteners import hex_area
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import pulley_bolt_points
from lib.models import raw
from lib.yaw_coupler.params import RIM_CLEAR, THRUST_CLEAR
from robot import frames as F
from tests.helpers import interference, is_inside

MOTORS = [m for m in mounts.MOUNTS if m.part in mounts.MOTORS]
BOARDS = {m.host: m for m in mounts.MOUNTS if m.part == mounts.BOARD}
ON_AXIS = [m for m in mounts.MOUNTS if m.part in (mounts.BEARING, mounts.THRUST_CAGE, mounts.THRUST_WASHER, mounts.PULLEY,
                                                  *mounts.PULLEY_BOLTS)]
THRUST = ("washer_as6590#1", "bearing_axk6590#1", "washer_as6590#2")   # up the base_yaw axis
# per belt joint: (upper bearing, lower bearing, the housing, what bears on the upper inner ring, the re-seated 90T)
STACKS = {
    "base_yaw": ("bearing_6806#1", "bearing_6806#2", "base#1", None, None),
    "elbow_pitch": ("bearing_6806#3", "bearing_6806#4", "j1_link#1", "forearm_roll_block", "gt2_pulley_90t#3"),
    "wrist_pitch": ("bearing_6806#5", "bearing_6806#6", "j2_link#1", "j3_coupler#2", "gt2_pulley_90t#4"),
}
# per bolted 90T: (its screw set, its nut set, the pulley, what the nuts sit in, the nut_af of that pocket, how much of
# each nut is inside it) - the elbow's nuts wholly in the block's channels, the wrist's 2.8 of 3.2 in j3_coupler's pockets
PULLEY_BOLTS = {
    "elbow_pitch": ("elbow_pulley_screws#1", "elbow_pulley_nuts#1", "gt2_pulley_90t#3", "forearm_roll_drive#1",
                    FOREARM.drive.nut_af, FOREARM.drive.nut_t),
    "wrist_pitch": ("wrist_pulley_screws#1", "wrist_pulley_nuts#1", "gt2_pulley_90t#4", "j3_coupler#2",
                    COUPLER.nut_af, COUPLER.nut_depth),
}


def _press(nut_af: float, depth: float) -> float:
    """The designed overlap of a joint's 4 nuts (ISO 4032, M4_NUT.af across flats) in pockets nut_af across flats,
    `depth` of each nut inside (docs/open_issues.md: the PETG press) - mm^3."""
    return len(PARAMS.pulley_90t_bolt_points()) * (hex_area(PARAMS.M4_NUT.af) - hex_area(nut_af)) * depth


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
        host = P.OCCURRENCES[m.host]   # a printed host: a SolidWorks record, or the base's bolt-on motor mount
        assert host["kind"] == "part" and not getattr(parts.load(host["part"]), "COTS", False)
        assert "mount" not in host or mounts.BY_KEY[m.host].host == "base#1"


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
    centre: the base motor mount's plate underside, j1_link's pad, j2_link's web."""
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
    """Interference budget (mm^3) of every mount against the hosts, the pulleys and the placed drives: zero everywhere
    (the Ø22 pilot boss used to stand in j2_link's Ø20 central slot - the parametric forearm's slot is 22.3 wide) but
    the pulley nuts' designed press in their nut_af pockets (_press, test_pulley_bolts_clamp_their_joints) and the base
    motor mount's nuts' in the base's ribs (JointParams.nut_pocket_af)."""
    neighbours = ["base#1", "base_motor_mount#1", "j1_coupler#1", "j1_link#1", "j2_link#1", "gt2_pulley_90t#3", "gt2_pulley_90t#4", "j3_coupler#2",
                  "wrist_link#1"]
    shapes = {k: place_world(P.OCCURRENCES[k]["part"], k) for k in neighbours}
    shapes["cycloidal_drive#1"] = raw(cycloidal_drive.cycloidal_drive).moved(_world("cycloidal_drive#1"))
    shapes["forearm_roll_drive#1"] = raw(forearm_roll_drive.forearm_roll_drive).moved(_world("forearm_roll_drive#1"))
    budget = {(nuts, host): _press(nut_af, depth) + 0.5 for _, nuts, _, host, nut_af, depth in PULLEY_BOLTS.values()}
    joint = BASE.joint
    budget[("base_motor_mount_nuts#1", "base#1")] = _press(joint.nut_pocket_af, joint.nut.h) + 0.5
    for m in mounts.MOUNTS:
        part = place_world(m.part, m.key)
        for key, other in shapes.items():
            if key == m.key:
                continue   # a re-seated pulley is a neighbour of the others
            vol = interference(part, other)
            assert vol <= budget.get((m.key, key), 1.0), f"{m.key} x {key}: {vol:.1f} mm^3"
    # the mounts do not run into each other either (a bearing and the pulley in it only touch)
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
    """The wrist motor's shaft (22 mm past its mounting face) reaches the plane of the 90T it drives: the 90T's
    bore-axis station lies between the mounting face and the shaft tip along the motor axis. (The elbow 90T is driven
    from a second stage through j1_link's x 128 seats, not from its motor - not modelled, docs/open_issues.md.)"""
    for m, pulley_key in (("nema17_40mm#3", "gt2_pulley_90t#4"),):
        world = _world(m)
        z = _dir(world).normalized()
        pulley = place_world("gt2_pulley_90t", pulley_key)
        bb = pulley.bounding_box()
        centre = Vector((bb.min.X + bb.max.X) / 2, (bb.min.Y + bb.max.Y) / 2, (bb.min.Z + bb.max.Z) / 2)
        station = (centre - world.position).dot(z)
        assert 0.0 < station < DEFAULT_CONFIG.motor.shaft_length, f"{m}: the 90T's mid-plane is {station:.1f} mm along the shaft"


def test_wrist_pitch_slide_position_is_inside_the_slots():
    lo, hi = PARAMS.J2_MOTOR_SLIDE_RANGE
    assert lo < PARAMS.J2_MOTOR_SLIDE_X < hi
    m = mounts.BY_KEY["nema17_40mm#3"]
    assert m.frame[0][0] == PARAMS.J2_MOTOR_SLIDE_X and m.frame[0][2] == PARAMS.J2_MOTOR_WEB_FACE_Z


def test_the_base_takes_the_48mm_motor_and_the_links_the_40mm_one():
    by_joint = {m.joint: m.part for m in mounts.MOUNTS if m.part in mounts.MOTORS}
    assert by_joint == {"base_yaw": "nema17_48mm", "elbow_pitch": "nema17_40mm", "wrist_pitch": "nema17_40mm"}


def test_bearings_and_pulleys_sit_on_their_joint_axes():
    """Each bearing's +Z and each re-seated pulley's +Y (lib/mounts.py AXES) runs along its joint's axis, its origin on
    the axis; a pair per belt joint, riding with its housing's link; each pulley PULLEY_SEAT_SHIFT out from its retired
    SolidWorks pose, along its own axis - the wrist's spin kept, the elbow's turned with its block's bolt pattern; each
    pulley-bolt pattern centred on its joint's axis too."""
    for m in ON_AXIS:
        joint, (local, on_axis) = F.JOINT_BY_NAME[m.joint], mounts.AXES[m.part]
        world = _world(m.key)
        axis = Vector(*joint.axis_w)
        assert on_axis and abs(abs(_dir(world, local).normalized().dot(axis)) - 1.0) < 1e-6, m.key
        d = world.position - Vector(*joint.origin_w)
        assert (d - axis * d.dot(axis)).length < 0.01, m.key
        assert m.key in F.LINKS[m.link], m.key
    by_joint = {j: [m.key for m in ON_AXIS if m.joint == j and m.part == mounts.BEARING] for j in STACKS}
    assert by_joint == {j: list(s[:2]) for j, s in STACKS.items()}
    for j, (up, _, host, _, _) in STACKS.items():
        assert mounts.BY_KEY[up].host == host and mounts.BY_KEY[up].link == F.JOINT_BY_NAME[j].parent
    for key, spin in (("gt2_pulley_90t#3", FOREARM.drive.pulley_bolt_deg), ("gt2_pulley_90t#4", 0.0)):
        m = mounts.BY_KEY[key]
        assert m.host in P.RETIRED and m.frame == ((0.0, PARAMS.PULLEY_SEAT_SHIFT, 0.0), (0.0, spin, 0.0))
        assert m.link == F.JOINT_BY_NAME[m.joint].child


@pytest.mark.slow
def test_the_elbow_pulley_holes_line_up_with_the_block_bolts():
    """The elbow 90T turned with the block's bolt pattern: each M4 axis runs through the pulley's hole along its
    whole length and on into the stub's clearance hole, the hub solid 3 mm beside it (a pattern 45 deg off would
    put every axis in solid hub)."""
    D = FOREARM.drive
    module = _world("forearm_roll_drive#1")
    block = next(c for c in raw(forearm_roll_drive.forearm_roll_drive).children if c.label == "forearm_roll_block").moved(module)
    pulley = place_world(mounts.PULLEY, "gt2_pulley_90t#3")

    def at(x, y, z):
        p = (module * Location((x, y, z))).position
        return p.X, p.Y, p.Z

    face = D.stub_x[0]
    for y, z in pulley_bolt_points(FOREARM):
        for x in (face - 1.0, face - D.pulley_hub_len / 2.0, face - D.pulley_hub_len + 1.0):
            assert not is_inside(pulley, *at(x, y, z)), (y, z, x)
        assert not is_inside(block, *at(face + 1.0, y, z)), (y, z)
        r = D.pulley_bolt_r
        assert is_inside(pulley, *at(face - 1.0, y + 3.0 * z / r, z - 3.0 * y / r)), (y, z)


def test_the_pulley_bolts_reach_their_nuts():
    """The arithmetic of both 90T stacks (lib/mounts.py FASTENER_MOUNTS): each screw set's heads on the pulley's outer
    face, its nuts on their host's seats - the elbow block's channel seats, j3_coupler's pocket floors - and every
    screw reaching two pitches past its nut; both hosts drill the pulley's own hole circle, with M4 clearance."""
    D, C, (y0, y1) = FOREARM.drive, COUPLER, PARAMS.GT2_PULLEY_90T_FACE_Y
    for joint, length in (("elbow_pitch", D.pulley_screw_len), ("wrist_pitch", C.pulley_screw_len)):
        screws, nuts = (mounts.BY_KEY[k] for k in PULLEY_BOLTS[joint][:2])
        assert screws.host == PULLEY_BOLTS[joint][2] and nuts.host == screws.key, joint
        assert screws.frame == ((0.0, y1, 0.0), (90.0, 0.0, 0.0)), joint                      # +Z into the hub
        assert length - (nuts.frame[0][2] + PARAMS.M4_NUT.h) >= 2 * PARAMS.M4_PITCH - 1e-9, joint
        assert parts.load(screws.part).PURCHASE_SPEC.startswith(f"M4 x {length:g} "), joint
    assert math.isclose(mounts.BY_KEY["elbow_pulley_nuts#1"].frame[0][2], D.nut_seat_x - (D.stub_x[0] - (y1 - y0)))
    assert math.isclose(mounts.BY_KEY["wrist_pulley_nuts#1"].frame[0][2], C.stub_y1 + (y1 - y0) - C.nut_depth)
    assert D.pulley_hub_len == y1 - y0 and D.nut_t == PARAMS.M4_NUT.h
    assert D.pulley_bolt_r == C.pulley_bolt_r == PARAMS.GT2_PULLEY_90T_BOLT_R
    assert D.pulley_bolt_dia == PARAMS.M4_CLEAR > C.pulley_bolt_dia > PARAMS.M4_SHCS.d


@pytest.mark.slow
@pytest.mark.parametrize("joint", list(PULLEY_BOLTS))
def test_pulley_bolts_clamp_their_joints(joint):
    """Each 90T's 4x M4: the heads bear on the pulley's outer face, each shank runs through the pulley's M4_CLEAR hole
    (the SolidWorks Ø3.9 took no M4 - the part opens it), the nuts bear on their host's seats and sit in its pockets
    with exactly the designed press, and the ring the heads sweep clears the parent link through a whole turn."""
    screws_key, nuts_key, pulley_key, host_key, nut_af, depth = PULLEY_BOLTS[joint]
    ws, wn = _world(screws_key), _world(nuts_key)
    z = _dir(ws).normalized()
    pulley = place_world(mounts.PULLEY, pulley_key)
    if host_key == "forearm_roll_drive#1":
        host = next(c for c in raw(forearm_roll_drive.forearm_roll_drive).children if c.label == "forearm_roll_block")
        host = host.moved(_world(host_key))
    else:
        host = place_world(P.OCCURRENCES[host_key]["part"], host_key)

    def planar(shape, origin: Vector, sign: int) -> bool:
        return any(round(f.normal_at().dot(z)) == sign and abs((f.center() - origin).dot(z)) < 1e-3
                   for f in shape.faces().filter_by(GeomType.PLANE) if abs(abs(f.normal_at().dot(z)) - 1.0) < 1e-6)

    assert planar(pulley, ws.position, -1), joint          # the outer face, facing the heads
    assert planar(host, wn.position, 1), joint             # the seat, facing the nuts
    hub = PARAMS.GT2_PULLEY_90T_FACE_Y[1] - PARAMS.GT2_PULLEY_90T_FACE_Y[0]
    r_hole = PARAMS.M4_CLEAR / 2.0
    for x, y in PARAMS.pulley_90t_bolt_points():
        ux, uy = x / math.hypot(x, y), y / math.hypot(x, y)
        for s in (1.0, hub / 2.0, hub - 1.0):
            def at(r, s=s, x=x, y=y, ux=ux, uy=uy):
                p = (ws * Location((x + r * ux, y + r * uy, s))).position
                return p.X, p.Y, p.Z
            assert not is_inside(pulley, *at(0.0)) and not is_inside(pulley, *at(r_hole - 0.1)), (joint, x, y, s)
            assert is_inside(pulley, *at(r_hole + 0.2)), (joint, x, y, s)          # the hole, no bigger
    nuts = place_world(mounts.BY_KEY[nuts_key].part, nuts_key)
    press = _press(nut_af, depth)
    assert 0.9 * press <= interference(nuts, host) <= press + 0.5, joint
    # the heads' ring (the hole circle +/- the head's radius, the head's height off the face) against every part of
    # the joint's parent link (the modules' bodies there - the drive's rotor, the roll shaft - are far from the pulleys)
    r, head = PARAMS.GT2_PULLEY_90T_BOLT_R, PARAMS.M4_SHCS
    ring = Pos(0.0, 0.0, -head.head_h) * (Cylinder(r + head.head_dia / 2.0, head.head_h, align=(Align.CENTER, Align.CENTER, Align.MIN))
                                          - Cylinder(r - head.head_dia / 2.0, head.head_h, align=(Align.CENTER, Align.CENTER, Align.MIN)))
    ring = ring.moved(ws)
    for key in F.LINKS[F.JOINT_BY_NAME[joint].parent]:
        if ":" not in key:
            assert interference(ring, place_world(P.OCCURRENCES[key]["part"], key)) < 1.0, (joint, key)


def _faces_on(shape, origin: Vector, z: Vector) -> list[tuple[int, float, float]]:
    """The planar faces of `shape` in the plane through `origin` normal to z: (the normal's sign along z, the smallest
    and largest radius of their circular edges about the axis through origin along z)."""
    found = []
    for f in shape.faces().filter_by(GeomType.PLANE):
        n = f.normal_at()
        if abs(abs(n.dot(z)) - 1.0) > 1e-6 or abs((f.center() - origin).dot(z)) > 1e-3:
            continue
        radii = [e.radius for e in f.edges() if e.geom_type == GeomType.CIRCLE
                 and ((e.arc_center - origin) - z * (e.arc_center - origin).dot(z)).length < 1e-3]
        if radii:
            found.append((round(n.dot(z)), min(radii), max(radii)))
    return found


@pytest.mark.slow
@pytest.mark.parametrize("joint", list(STACKS))
def test_bearing_stacks(joint):
    """Each belt joint's pair: both bearings on the lip that splits the housing's bore (the lip bears on the outer
    rings only), the coupler's shoulder down on the upper inner ring, the re-seated 90T's ring under the lower inner
    ring, the coupler's stub on through the lip to the pulley, which bolts flat onto its end - a solid joint that
    clamps both inner rings - and no overlap anywhere. The base's differs: j1_coupler stands on the thrust bearing
    instead of a shoulder (test_thrust_bearing_carries_j1_coupler), and the base_yaw pulley is not modelled
    (docs/open_issues.md)."""
    up, lo, host_key, coupler_key, pulley_key = STACKS[joint]
    host = place_world(P.OCCURRENCES[host_key]["part"], host_key)
    wu, wl = _world(up), _world(lo)
    z = _dir(wu).normalized()
    width = Location((0.0, 0.0, BEARING_6806_WIDTH))
    inner_r, lip_r = BEARING_6806_SHOULDER_DIA / 2.0, math.inf
    for station, sign in ((wu.position, 1), ((wl * width).position, -1)):      # the lip's two faces
        faces = [f for f in _faces_on(host, station, z) if f[0] == sign]
        assert faces, (joint, sign)
        lip_r = min(lip_r, min(r0 for _, r0, _ in faces))
    assert lip_r > inner_r + 1.0, (joint, lip_r)                                # the lip clears the inner rings
    bearings = [place_world(mounts.BEARING, k) for k in (up, lo)]
    others = [host]
    if coupler_key is not None:
        if coupler_key == "forearm_roll_block":
            coupler = next(c for c in raw(forearm_roll_drive.forearm_roll_drive).children if c.label == coupler_key)
            coupler = coupler.moved(_world("forearm_roll_drive#1"))
        else:
            coupler = place_world(P.OCCURRENCES[coupler_key]["part"], coupler_key)
        pulley = place_world(mounts.PULLEY, pulley_key)
        shoulder = [f for f in _faces_on(coupler, (wu * width).position, z) if f[0] == -1]
        assert shoulder and max(r1 for _, _, r1 in shoulder) <= inner_r + 1e-6, (joint, shoulder)     # the inner ring only
        ring = [f for f in _faces_on(pulley, wl.position, z) if f[0] == 1]
        assert ring and max(r1 for _, _, r1 in ring) < lip_r, (joint, ring)                           # clear of the outer ring
        # the pulley bolts flat onto the stub's end, in the plane of the lip's lower face (the lower bearing's top)
        plane = (wl * width).position
        stub_end = [f for f in _faces_on(coupler, plane, z) if f[0] == -1]
        hub_end = [f for f in _faces_on(pulley, plane, z) if f[0] == 1]
        assert stub_end and hub_end, (joint, stub_end, hub_end)
        contact = (max(min(r0 for _, r0, _ in stub_end), min(r0 for _, r0, _ in hub_end)),
                   min(max(r1 for _, _, r1 in stub_end), max(r1 for _, _, r1 in hub_end)))
        assert contact[1] - contact[0] > 5.0, (joint, contact)                                        # a real annulus
        for a, b in ((pulley, coupler), (pulley, host), (coupler, host)):
            assert interference(a, b) < 1.0, joint
        others += [coupler, pulley]
    # line-to-line fits (the elbow's upper seat Ø42.0, the Ø30 stubs and hubs) leave float noise in the boolean that
    # differs per machine (0.05 mm^3 on x86_64 Linux, 0 on arm64 macOS) - the suite's 1 mm^3 'no overlap' budget;
    # a real misfit is far above it (0.1 mm across a bearing's face is ~68 mm^3)
    for b in bearings:
        for o in others + [bearings[1] if b is bearings[0] else bearings[0]]:
            assert interference(b, o) < 1.0, joint


@pytest.mark.slow
def test_thrust_bearing_carries_j1_coupler():
    """base_yaw's axial load: the lower washer on the floor of the base's groove, the needle cage on it, the upper washer
    on the cage and j1_coupler's seat down on that - face to face up the axis, each contact the washers' whole annulus -;
    the stack THRUST_CLEAR inside the coupler's recess and RING_CLEAR round the base's seat ring (inside its bore) and
    inside the groove's wall; the coupler nowhere nearer the base than its rim's RIM_CLEAR over the top face (it turns on
    the stack, not on the base's face); no overlap with the base, the coupler or the 6806 pair."""
    lower, cage, upper = (place_world(mounts.BY_KEY[k].part, k) for k in THRUST)
    base, coupler = (place_world(P.OCCURRENCES[k]["part"], k) for k in ("base#1", "j1_coupler#1"))
    w = _world(THRUST[0])
    z = _dir(w).normalized()
    assert abs(z.dot(Vector(*F.JOINT_BY_NAME["base_yaw"].axis_w)) - 1.0) < 1e-6
    heights = (0.0, THRUST_WASHER_WIDTH, THRUST_WASHER_WIDTH + THRUST_CAGE_WIDTH, 2.0 * THRUST_WASHER_WIDTH + THRUST_CAGE_WIDTH)
    for h, below, above in zip(heights, (base, lower, cage, upper), (lower, cage, upper, coupler), strict=True):
        plane = (w * Location((0.0, 0.0, h))).position
        down = [f for f in _faces_on(above, plane, z) if f[0] == -1]
        up = [f for f in _faces_on(below, plane, z) if f[0] == 1]
        assert down and up, h
        contact = (max(min(r0 for _, r0, _ in down), min(r0 for _, r0, _ in up)),
                   min(max(r1 for _, _, r1 in down), max(r1 for _, _, r1 in up)))
        assert contact[0] <= THRUST_BORE / 2.0 + 1e-6 and contact[1] >= THRUST_OD / 2.0 - 1e-6, (h, contact)
    assert base.distance_to(cage) == pytest.approx(RING_CLEAR, abs=1e-3)        # round the ring, inside the groove
    assert coupler.distance_to(cage) == pytest.approx(THRUST_CLEAR, abs=1e-3)   # inside the recess
    assert base.distance_to(coupler) == pytest.approx(RIM_CLEAR, abs=1e-3)      # the rim over the top face
    shapes = [base, coupler, lower, cage, upper, *(place_world(mounts.BEARING, k) for k in STACKS["base_yaw"][:2])]
    for i, a in enumerate(shapes):
        for b in shapes[i + 1:]:
            assert interference(a, b) < 1.0
