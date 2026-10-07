"""j1_coupler, parametric (lib/yaw_coupler/): the LEGACY configuration reproduces the SolidWorks part feature by
feature (the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a
regression names what moved), the yoke's shapes are the drive's housing's - its bore, its od, its pillars' sides,
its bolt circle and nut pockets (lib/cycloidal/params.py HousingParams; LEGACY's the port's housing, LEGACY_CONFIG) -,
and DEFAULT - what the part builds - changes its underside, its hub and the yoke: the seat on the thrust bearing, the
rim clear of the base (tests/test_mounts.py checks the stack in place), the stub on to the lip's lower face and drilled
for the base_yaw pulley, and the fork (ForkParams) round the drive whose shell turns (DEFAULT_CONFIG's ShellParams): two
legs past the shell's ends, mirrored about the middle of the discs, the disc drafted round to its rim and its side
carried up their outer faces to under the hub's bolts - the held hub bolts to one, its disc the hub's own diameter; the
other is its own part, parts/base/j1_motor_leg (no reference: its numbers are here), a solid ring round the motor
plate's sleeve bolted to the disc -, the ring lowered under the shell (tests/cycloidal/test_assembly.py checks the drive
in place, tests/test_sweeps.py the arm's swing)."""
import math
from dataclasses import replace

import pytest

from lib import placements as P
from lib import reference as R
from lib.base import DEFAULT as BASE
from lib.bearings import BEARING_6806_BORE, THRUST_OD, THRUST_STACK
from lib.belts import GT2_PULLEY_90T_BOLT_R
from lib.coupler import DEFAULT as J3_COUPLER
from lib.cycloidal import arm_mount_points, shell_ends, sleeve_end, stack_positions
from lib.cycloidal.layout import PILLAR_OVERSHOOT
from lib.cycloidal.params import DEFAULT_CONFIG as DRIVE
from lib.cycloidal.params import LEGACY_CONFIG as PORT
from lib.fasteners import M4_CLEAR, M4_NUT, M4_SHCS
from lib.geom import hex_circumdiameter
from lib.yaw_coupler import DEFAULT, LEGACY, hole_points, nut_centres, od_point
from lib.yaw_coupler.layout import (
    COUNTERBORE,
    NUT_SLOT,
    below_axis,
    disc_r,
    fork_flare_top,
    fork_hub_bolts,
    fork_hub_leg_x,
    fork_motor_bolt_x,
    fork_motor_bolts,
    fork_motor_leg_x,
    fork_x,
)
from lib.yaw_coupler.params import BASE_DRAFT, BASE_R
from tests import built
from tests.helpers import is_inside


def _polar(y: float, z: float, deg: float) -> tuple[float, float]:
    """(along, across) of a (y, z) point from the drive's axis, against the pillar at deg from straight down."""
    k, a = LEGACY.yoke, math.radians(deg)
    u = k.axis_y - y
    return (u * math.cos(a) + z * math.sin(a), abs(-u * math.sin(a) + z * math.cos(a)))


def _pillar_half_w(along: float, drive=DRIVE) -> float:
    """The housing pillar's half width at `along` from the drive's axis (lib/cycloidal/housing.py's trapezoid) - the
    housing the parts build, or `drive`'s."""
    h = drive.housing
    r0, r1 = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + PILLAR_OVERSHOOT
    return (h.pillar_inner_w + (h.pillar_outer_w - h.pillar_inner_w) * (along - r0) / (r1 - r0)) / 2.0


def test_the_yoke_is_the_housings_shape():
    """LEGACY's yoke (the SolidWorks part) on the port's housing (LEGACY_CONFIG)."""
    k, h = LEGACY.yoke, PORT.housing
    assert (2.0 * k.cradle_r, 2.0 * k.od_r, 2.0 * k.bolt_circle_r) == (h.bore_dia, h.od, h.bolt_circle_dia)
    assert k.nut_af == pytest.approx(h.bolt_nut_pocket_af, abs=0.01) and k.nut_depth == h.bolt_nut_depth
    near, end = k.groove_z
    c = (below_axis(LEGACY, k.cradle_r, near), near)
    d = (below_axis(LEGACY, k.od_r, end), end)
    a = (below_axis(LEGACY, k.cradle_r, k.cheek_top_z), k.cheek_top_z)
    b = od_point(LEGACY)
    # the V-groove's near side (C -> D) is the 45 degree pillar's side, no clearance; the cheek's top (B -> A) its
    # far side; D and B, C and A mirror each other about the pillar's centre line (to the census's 1e-4)
    for p in (c, d, a, b):
        along, across = _polar(*p, 45.0)
        assert across == pytest.approx(_pillar_half_w(along, PORT), abs=0.01)
    assert (k.axis_y - b[0], b[1]) == pytest.approx((d[1], k.axis_y - d[0]), abs=1e-4)
    assert (k.axis_y - a[0], a[1]) == pytest.approx((c[1], k.axis_y - c[0]), abs=1e-4)
    # the channel: its floor corners on the od, its walls the bottom pillar's sides 0.2 out
    corner_along = k.axis_y - k.channel_floor_y
    assert math.hypot(corner_along, _pillar_half_w(corner_along, PORT)) == pytest.approx(k.od_r, abs=0.01)
    assert k.channel_slope == pytest.approx(_pillar_half_w(60.0, PORT) - _pillar_half_w(61.0, PORT))
    gap = (k.channel_half_z - _pillar_half_w(corner_along, PORT)) / math.hypot(1.0, k.channel_slope)
    assert gap == pytest.approx(0.2, abs=0.005)
    # the nut pockets: the bottom housing bolt and the two at +/-45 degrees, on the bolt circle
    assert [math.hypot(k.axis_y - y, z) for y, z in nut_centres(LEGACY)] == pytest.approx([k.bolt_circle_r] * 3)


def test_default_fork_holds_the_drives_ends():
    """DEFAULT's fork on the drive's own axis (placements.json cycloidal_drive#1 in this frame, its +Z this frame's -X):
    two legs ShellParams.yoke_leg thick, end_plate_gap past the shell's two ends - the hub's face (hub_top) on the hub
    leg's inner face, the sleeve's end (sleeve_end) on the motor leg's outer face -, mirror images about the middle of the
    discs, which the shift (lib/placements.py SHIFTS) puts on the base_yaw axis; the hub's 4 bolts on the drive's
    arm-mount pattern, their ends flush with the hub's captive nuts, the hub leg's disc the hub's own diameter round
    them; the disc's draft up the legs meeting their outer faces under the hub's bolt heads, the legs' root 20 thick;
    the motor leg round the sleeve (its ring's bore 0.2 a side), its 2 M4s through its foot into nuts under its post, the
    heads sunk in the foot; the ring lowered under the turning shell's pillars (RING_DROP: 2.68 of clearance)."""
    f, sh, S = DEFAULT.fork, DRIVE.shell, stack_positions(DRIVE)
    world = P.location("j1_coupler#1", "world").inverse() * P.location("cycloidal_drive#1", "world")
    assert (world.position.X, world.position.Y, world.position.Z) == pytest.approx((f.face_x, f.axis_y, f.axis_z), abs=1e-5)
    assert fork_x(DEFAULT, 0.0) == f.face_x and fork_x(DEFAULT, 1.0) == f.face_x - 1.0
    (hx0, hx1), (mx0, mx1), (end_0, end_1) = fork_hub_leg_x(DEFAULT), fork_motor_leg_x(DEFAULT), shell_ends(DRIVE)
    assert (hx0, hx1) == pytest.approx((fork_x(DEFAULT, S["hub_top"] + sh.yoke_leg), fork_x(DEFAULT, S["hub_top"])))
    assert S["hub_top"] == pytest.approx(end_1 + sh.end_plate_gap)
    assert (mx0, mx1) == pytest.approx((fork_x(DEFAULT, end_0 - sh.end_plate_gap), fork_x(DEFAULT, sleeve_end(DRIVE))))
    middle = fork_x(DEFAULT, (S["z_disc1"] + S["z_disc2"] + DRIVE.disc.thickness) / 2.0)
    assert hx1 + mx0 == pytest.approx(2.0 * middle) and hx0 + mx1 == pytest.approx(2.0 * middle)
    assert middle == pytest.approx(0.0, abs=1e-9)                     # ... on the base_yaw axis: the legs mirror about it too
    assert hx0 < -DEFAULT.disc.flat_x < hx1 and mx0 < DEFAULT.disc.flat_x < mx1   # each straddling a flat of the disc
    assert sorted(fork_hub_bolts(DEFAULT)) == pytest.approx(sorted((f.axis_y + y, f.axis_z + x) for x, y in arm_mount_points(DRIVE)))
    head = S["hub_top"] + sh.yoke_leg - (M4_SHCS.head_h + 0.5)        # the screws' heads' seats, drive z
    assert head - f.hub_screw_len == pytest.approx(S["z_hub"] + sh.hub_nut_depth - DRIVE.housing.bolt_nut_thickness)
    # the hub leg's disc flush with the hub, its counterbores 6.45 inside the edge
    assert f.hub_plate_r == pytest.approx(DRIVE.output_hub.od / 2.0)
    reach = max(math.hypot(y - f.axis_y, z - f.axis_z) for y, z in fork_hub_bolts(DEFAULT)) + COUNTERBORE / 2.0
    assert f.hub_plate_r - reach == pytest.approx(6.45, abs=0.01)
    # the disc's draft: BASE_R at the rim, in at BASE_DRAFT to the legs' outer faces at y 63 - their root 20 thick - and
    # up them to flare_seat_flat under the hub's lowest bolt heads
    assert disc_r(DEFAULT, DEFAULT.disc.band_y1) == BASE_R and disc_r(DEFAULT, 63.0) == pytest.approx(mx1)
    assert disc_r(DEFAULT, DEFAULT.disc.y0) - mx0 == pytest.approx(19.98, abs=0.01)
    assert BASE_DRAFT == pytest.approx(math.tan(math.radians(10.8)), abs=1e-3)
    top = fork_flare_top(DEFAULT)
    assert top == pytest.approx(min(y for y, _ in fork_hub_bolts(DEFAULT)) - COUNTERBORE / 2.0 - f.flare_seat_flat)
    assert disc_r(DEFAULT, top) < mx1                                # the draft has met the face below its top
    # the motor leg's ring round the sleeve; its 2 M4s: through the foot (butting the disc at the leg's outer face) and
    # the wall into nuts in slots under the leg's post, the screws' tips 0.7 past the nuts, the heads sunk in the foot
    assert f.ring_bore_dia == pytest.approx(sh.sleeve_od + 0.4)
    seat_x, nut_x1, nut_x0 = fork_motor_bolt_x(DEFAULT)
    assert (seat_x, nut_x1) == pytest.approx((mx1 + f.motor_head_seat, mx1 - f.motor_nut_wall))
    assert nut_x1 - M4_NUT.h - (seat_x - f.motor_screw_len) == pytest.approx(0.7)
    corner = (M4_NUT.af + NUT_SLOT) / math.sqrt(3.0)
    for y, z in fork_motor_bolts(DEFAULT):
        assert mx0 < nut_x0 and abs(z - f.axis_z) + (M4_NUT.af + NUT_SLOT) / 2.0 < f.leg_half_z   # the slot under the post
        assert y - corner > DEFAULT.disc.y0 + 4.0 and y + 2.2 < DEFAULT.disc.top_y                 # ... and in the disc
        x_surface = math.sqrt(disc_r(DEFAULT, y) ** 2 - z * z)
        assert x_surface > seat_x + M4_SHCS.head_h + 0.2                                        # the head sunk under the draft
    sweep = DRIVE.housing.od / 2.0                                  # the pillars' tips, turning
    assert f.axis_y - sweep - DEFAULT.disc.ring_y1 == pytest.approx(2.68, abs=0.005)
    assert LEGACY.disc.ring_y1 - DEFAULT.disc.ring_y1 == 2.0


@pytest.fixture(scope="module")
def legacy():
    return built.legacy("j1_coupler")


@pytest.fixture(scope="module")
def coupler():
    return built.part("j1_coupler")


@pytest.mark.slow
def test_legacy_disc(legacy):
    leg = legacy
    assert leg.is_valid and len(leg.solids()) == 1
    assert is_inside(leg, 0, -0.2, 52.8) and not is_inside(leg, 0, -0.2, 53.3)       # the band ...
    assert is_inside(leg, 0, 19.8, 50.6) and not is_inside(leg, 0, 19.8, 51.1)       # ... drafted in to the top face
    assert not is_inside(leg, 0, -0.5, 50) and is_inside(leg, 0, -0.3, 50)           # the rim's underside
    assert not is_inside(leg, 0, 0.3, 44.5) and is_inside(leg, 0, 0.7, 44.5)         # the recess, 0.9 deep ...
    assert is_inside(leg, 0, 0.3, 45.5)                                              # ... Ø90.05
    assert is_inside(leg, 39.5, 12, 0) and not is_inside(leg, 40.5, 12, 0)           # the flats
    assert is_inside(leg, 47.5, 5, 0) and not is_inside(leg, 48.5, 5, 0)             # the ears ...
    assert not is_inside(leg, 45, 8.5, 0)                                            # ... up to y 8
    assert is_inside(leg, -32.5, 22, 20) and not is_inside(leg, -33.0, 22, 20)       # the ring, from the cheek's face ...
    assert is_inside(leg, 0, 22, 40.5) and not is_inside(leg, 0, 22, 41.0)           # ... Ø81.46


@pytest.mark.slow
def test_legacy_yoke(legacy):
    leg = legacy
    assert is_inside(leg, 0, 26, 43.5) and not is_inside(leg, 0, 26, 44.5)           # the flare
    assert is_inside(leg, 0, 40, 52.9) and not is_inside(leg, 0, 40, 53.4)           # the outer faces
    assert is_inside(leg, 20, 34.5, 20) and not is_inside(leg, 20, 36.5, 20)         # the cradle
    pillar = (LEGACY.yoke.axis_y - 64 * math.cos(math.radians(45)), 64 * math.sin(math.radians(45)))
    assert not is_inside(leg, 0, *pillar)                                            # the V-groove round the pillar ...
    assert is_inside(leg, 0, 38.0, 33.77)                                            # ... inside its near side
    assert is_inside(leg, -31, 45.8, 49.2)                                           # the cheek, across the pillar
    assert not is_inside(leg, 25, 21, 0) and is_inside(leg, 25, 20.0, 0)             # the channel's floor ...
    assert is_inside(leg, 25, 21, 6.0)                                               # ... its wall
    assert not is_inside(leg, 32.4, 21, 0) and is_inside(leg, 33.0, 21, 0)           # ... its +X end
    assert not is_inside(leg, 35, 23.5, 0) and is_inside(leg, 35, 22.3, 0)           # the notch
    assert not is_inside(leg, 15.0, 10, 30.8) and is_inside(leg, 15.5, 10, 0)        # the pocket ...
    assert is_inside(leg, 0, 10, 31.3)
    assert is_inside(leg, 0, 7.8, 20) and not is_inside(leg, 0, 8.2, 20)             # ... its floor
    for y, z in nut_centres(LEGACY):
        assert not is_inside(leg, -31, y, z) and not is_inside(leg, -28.7, y, z)     # the nut pocket, the hole ...
        assert is_inside(leg, -28.7, y + 2.5, z)                                     # ... 4 deep
    y, z = nut_centres(LEGACY)[1]
    assert is_inside(leg, -31, y + 3.8, z) and not is_inside(leg, -31, y + 3.4, z)   # flats across y ...
    assert not is_inside(leg, -31, y, z + 4.0) and is_inside(leg, -31, y, z + 4.3)   # ... a corner along z


@pytest.mark.slow
def test_legacy_hub(legacy):
    leg = legacy
    assert is_inside(leg, 0, -5, 14.7) and not is_inside(leg, 0, -5, 15.1)           # the stub
    assert is_inside(leg, 0, -8.1, 12) and not is_inside(leg, 0, -8.3, 12)           # its end ...
    assert not is_inside(leg, 0, -8.1, 14.8) and is_inside(leg, 0, -7.3, 14.8)       # ... the R1 round
    assert not is_inside(leg, 0, 2, 7.3) and is_inside(leg, 0, 2, 7.7)               # the bore
    for x, z in hole_points(LEGACY):
        assert not is_inside(leg, x, 2, z) and is_inside(leg, 1.2 * x, 2, 1.2 * z)   # the 4 holes on the diagonals
        assert not is_inside(leg, x, -8.0, z)                                        # ... from the stub's end


def test_default_stands_on_the_thrust_bearing():
    """The seat on the upper washer: the groove's floor (the base's frame; this part's origin at its ring_top_y) plus
    the stack; the recess round the stack's OD; the rim over the base's top face - nothing else changes (but the
    fork and the ring under it, flat to flat: test_default_fork_holds_the_drives_ends)."""
    h, c = DEFAULT.hub, BASE.cap
    assert replace(DEFAULT, hub=LEGACY.hub, disc=replace(DEFAULT.disc, y0=LEGACY.disc.y0, ring_y1=LEGACY.disc.ring_y1,
                                                         ring_x0=LEGACY.disc.ring_x0, band_r=LEGACY.disc.band_r,
                                                         top_r=LEGACY.disc.top_r), fork=None) == LEGACY
    assert DEFAULT.disc.ring_x0 == -DEFAULT.disc.flat_x
    assert h.recess_y1 == pytest.approx(c.groove_y0 - c.ring_top_y + THRUST_STACK)
    assert h.recess_dia > THRUST_OD and h.recess_dia / 2.0 > c.groove_r[1]          # the rim wholly over the top face
    assert DEFAULT.disc.y0 > c.top_y - c.ring_top_y and LEGACY.disc.y0 == pytest.approx(c.top_y - c.ring_top_y)


def test_default_hub_takes_the_base_yaw_90t():
    """The stub the bearings' bore, its end on the lip's lower face where the 90T's hub end meets it (as at the wrist;
    the SolidWorks end stopped 0.2 into the lip); the 90T's 4 bolts on its bolt circle at the diagonals (the 90T turned
    45 degrees on its own pattern), M4 clearance, their nuts flush in the pocket's floor clear of its walls and of the
    bore; the 90T's bore, 2.5 of wall to the holes (the SolidWorks Ø15 left 1.45)."""
    h, b, c = DEFAULT.hub, BASE.bore, BASE.cap
    assert h.stub_dia == BEARING_6806_BORE and (h.stub_round, h.hole_deg) == (LEGACY.hub.stub_round, LEGACY.hub.hole_deg)
    assert h.stub_y0 == pytest.approx(b.lip_y[0] - c.ring_top_y) and LEGACY.hub.stub_y0 == pytest.approx(b.lip_y[1] - c.ring_top_y - 0.2)
    assert (h.hole_dia, h.hole_r, h.bore_dia) == (M4_CLEAR, GT2_PULLEY_90T_BOLT_R, J3_COUPLER.bore_dia)
    assert sorted(round(math.degrees(math.atan2(z, x))) % 360 for x, z in hole_points(DEFAULT)) == [45, 135, 225, 315]
    assert (h.nut_af, h.nut_depth) == (J3_COUPLER.nut_af, M4_NUT.h) and LEGACY.hub.nut_af is None
    corner = hex_circumdiameter(h.nut_af) / 2.0
    (px, pz), k = DEFAULT.yoke.pocket_half, DEFAULT.yoke
    for x, z in hole_points(DEFAULT):
        assert abs(x) + corner < px - 3.0 and abs(z) + corner < pz - 3.0                 # clear of the pocket's walls
        assert math.hypot(x, z) - corner > h.bore_dia / 2.0 + 0.5                        # ... and of the bore
    assert k.pocket_y0 - h.nut_depth > h.recess_y1 + 3.0                                 # material under the nuts
    assert h.hole_r - (h.hole_dia + h.bore_dia) / 2.0 == pytest.approx(2.55)


@pytest.mark.slow
def test_default_changes_the_hub_and_the_underside(coupler, legacy):
    assert coupler.label == "j1_coupler" and coupler.is_valid and len(coupler.solids()) == 1
    assert coupler.bounding_box().min.Y == pytest.approx(DEFAULT.hub.stub_y0, abs=1e-6)       # the stub's end, lower
    assert is_inside(coupler, 0, -9.2, 12) and not is_inside(coupler, 0, -9.4, 12) and not is_inside(legacy, 0, -9.2, 12)
    assert is_inside(coupler, 0, -5, 14.95) and not is_inside(coupler, 0, -5, 15.05)          # the stub Ø30 ...
    assert not is_inside(legacy, 0, -5, 14.95)                                                # ... was Ø29.8
    assert is_inside(coupler, 0, 2, 7.0) and not is_inside(coupler, 0, 2, 6.1) and not is_inside(legacy, 0, 2, 7.0)   # the bore
    for x, z in hole_points(DEFAULT):
        assert not is_inside(coupler, x, 2, z) and not is_inside(coupler, x, -9.2, z)         # the 90T's holes, end to end ...
        edge = (x * 13.0 / 11.0, 2, z * 13.0 / 11.0)
        assert not is_inside(coupler, *edge) and is_inside(legacy, *edge)                     # ... M4 clearance
        assert not is_inside(coupler, x + 3.0, 6.0, z) and is_inside(legacy, x + 3.0, 6.0, z)   # the nut pocket: flats across x ...
        assert is_inside(coupler, x + 3.6, 6.0, z) and not is_inside(coupler, x, 6.0, z + 3.8)  # ... a corner along z ...
        assert is_inside(coupler, x + 3.0, 4.6, z)                                            # ... 3.2 deep
    y1 = DEFAULT.hub.recess_y1
    assert not is_inside(coupler, 0, y1 - 0.05, 44) and is_inside(coupler, 0, y1 + 0.05, 44)   # the seat, 1.1 higher
    assert is_inside(legacy, 0, y1 - 0.05, 44)
    assert not is_inside(coupler, 0, 1.0, 45.1) and is_inside(coupler, 0, 1.0, 45.3)          # the recess round the stack
    y0 = DEFAULT.disc.y0
    assert not is_inside(coupler, 0, y0 - 0.05, 50) and is_inside(coupler, 0, y0 + 0.05, 50)   # the rim, lifted
    assert is_inside(legacy, 0, y0 - 0.05, 50)
    r0 = disc_r(DEFAULT, y0 + 0.05)                                                           # ... round to BASE_R: the flats
    for x, z in ((-r0 + 0.3, 0), (0, r0 - 0.3), (0, -r0 + 0.3), (-35.0, 40.0)):              # and the ears filled
        assert not is_inside(coupler, x, y0 - 0.05, z) and is_inside(coupler, x, y0 + 0.05, z)
    assert not is_inside(coupler, -r0 - 0.3, y0 + 0.05, 0)
    assert is_inside(coupler, 0, y1 - 0.05, 14.5) and is_inside(coupler, 0, -5, 14.7)          # the stub up into the seat
    for probe in ((0, 2, 7.7), (0, 7.8, 20)):
        assert is_inside(coupler, *probe) == is_inside(legacy, *probe)                         # the hub, the pocket's floor
    assert not is_inside(coupler, 0, 10, 20) and not is_inside(legacy, 0, 10, 20)             # the pocket over the hub


@pytest.mark.slow
def test_default_is_the_fork(coupler, legacy):
    """The fork round the drive whose shell turns (ForkParams), turned with the disc: the disc drafted round to its rim
    under both legs (its flats and ears filled); the hub leg on -X - its disc the hub's own round the axis on a post down
    to the disc, the disc's draft carried up its outer face within the post, meeting the face in a curved edge, highest
    in the middle, under the hub's bolts -; on +X the coupler stops for the motor leg, its own part: at the leg's outer
    face, up to the disc's top face, its ring cut back off the leg, the leg's 2 M4s and their nuts' slots in the disc;
    the yoke's cheek, middle body, cradle and sockets gone; the ring lowered."""
    f, d = DEFAULT.fork, DEFAULT.disc
    (hx0, hx1), (mx0, mx1) = fork_hub_leg_x(DEFAULT), fork_motor_leg_x(DEFAULT)
    bb = coupler.bounding_box()
    r0 = disc_r(DEFAULT, d.y0)                                       # the disc at the rim, lifted
    assert (bb.min.X, bb.max.X, bb.max.Y) == pytest.approx((-r0, mx1, f.axis_y + f.hub_plate_r), abs=1e-6)
    assert (bb.min.Z, bb.max.Z) == pytest.approx((-r0, r0), abs=1e-6)
    # the hub leg: its disc round the axis on a post down to the disc, its inner and outer faces over the draft
    hm = (hx0 + hx1) / 2.0
    assert is_inside(coupler, hm, 40.0, f.axis_z + f.leg_half_z - 0.3) and not is_inside(coupler, hm, 40.0, f.axis_z + f.leg_half_z + 0.3)
    assert is_inside(coupler, hm, f.axis_y + f.hub_plate_r - 0.3, f.axis_z) and not is_inside(coupler, hm, f.axis_y + f.hub_plate_r + 0.3, f.axis_z)
    assert is_inside(coupler, hm, f.axis_y, f.axis_z + f.hub_plate_r - 0.3) and not is_inside(coupler, hm, f.axis_y, f.axis_z + f.hub_plate_r + 0.3)
    assert is_inside(coupler, hx0 + 0.3, 80.0, 0.0) and not is_inside(coupler, hx0 - 0.3, 80.0, 0.0)
    assert is_inside(coupler, hx1 - 0.3, 80.0, 0.0) and not is_inside(coupler, hx1 + 0.3, 80.0, 0.0)
    # the disc's draft up the outer face, within the post; it meets the face higher in the middle than at the sides
    for y in (25.0, 40.0, 55.0):
        face = -disc_r(DEFAULT, y)
        assert is_inside(coupler, face + 0.3, y, 0.0) and not is_inside(coupler, face - 0.3, y, 0.0)
    assert is_inside(coupler, -46.2, 25.0, f.axis_z + f.leg_half_z - 0.3) and not is_inside(coupler, -46.2, 25.0, f.axis_z + f.leg_half_z + 0.3)
    assert is_inside(coupler, hx0 - 0.3, 61.0, 0.0) and not is_inside(coupler, hx0 - 0.3, 64.0, 0.0)
    assert is_inside(coupler, hx0 - 0.3, 35.0, 20.0) and not is_inside(coupler, hx0 - 0.3, 45.0, 20.0)
    # the disc round to its rim under the leg, past the SolidWorks flat and ears
    assert is_inside(coupler, -50.0, 10.0, 20.0) and not is_inside(legacy, -50.0, 10.0, 20.0)
    assert not is_inside(coupler, -50.0, 10.0, 30.0)
    for y, z in fork_hub_bolts(DEFAULT):
        assert not is_inside(coupler, hx1 - 0.3, y, z) and not is_inside(coupler, hx0 + 0.3, y + 3.0, z)   # the hole, the counterbore
        assert is_inside(coupler, hx1 - 0.3, y + 3.0, z)                                                  # ... 4.5 deep
    # the ring lowered
    assert is_inside(coupler, -25.0, d.ring_y1 - 0.3, 25.0) and not is_inside(coupler, -25.0, d.ring_y1 + 0.3, 25.0)
    assert is_inside(legacy, -25.0, d.ring_y1 + 0.3, 25.0)
    # the motor side: the coupler stops at the leg's outer face and the disc's top face, its ring cut back off the leg
    assert is_inside(coupler, mx1 - 0.3, 5.0, 0.0) and not is_inside(coupler, mx1 + 0.3, 5.0, 0.0)
    assert is_inside(coupler, 42.0, d.top_y - 0.3, 0.0) and not is_inside(coupler, 42.0, d.top_y + 0.3, 0.0)
    assert is_inside(coupler, mx0 - f.motor_leg_gap - 0.1, 21.5, 0.0) and not is_inside(coupler, mx0 - f.motor_leg_gap + 0.1, 21.5, 0.0)
    assert not is_inside(coupler, 42.0, 40.0, 0.0) and not is_inside(coupler, 42.0, 80.0, 0.0)
    # ... the motor leg's 2 M4s through the wall into the nuts' slots from the top face, flats across z, a corner down
    _, nut_x1, nut_x0 = fork_motor_bolt_x(DEFAULT)
    slot_x, half_af, corner = (nut_x0 + nut_x1) / 2.0, (M4_NUT.af + NUT_SLOT) / 2.0, (M4_NUT.af + NUT_SLOT) / math.sqrt(3.0)
    for y, z in fork_motor_bolts(DEFAULT):
        assert not is_inside(coupler, (nut_x1 + mx1) / 2.0, y, z) and not is_inside(coupler, slot_x, y, z)   # the hole, the slot ...
        assert not is_inside(coupler, slot_x, d.top_y - 0.3, z)                                              # ... open at the top
        assert is_inside(coupler, slot_x, y - corner - 0.3, z) and not is_inside(coupler, slot_x, y - corner + 0.3, z)
        for sz in (-1.0, 1.0):
            assert not is_inside(coupler, slot_x, y, z + sz * (half_af - 0.2)) and is_inside(coupler, slot_x, y, z + sz * (half_af + 0.2))
        assert is_inside(coupler, nut_x0 - 0.3, y + 3.0, z) and is_inside(coupler, nut_x1 + 0.3, y + 3.0, z)  # its walls
    # the yoke that held the port's housing is gone
    for probe in ((0, 40, 52.9), (20, 34.5, 20), (-31, 45.8, 49.2)):
        assert not is_inside(coupler, *probe) and is_inside(legacy, *probe)


@pytest.fixture(scope="module")
def motor_leg():
    return built.part("j1_motor_leg")


@pytest.mark.slow
def test_motor_leg(motor_leg):
    """The motor leg (parts/base/j1_motor_leg, no reference - its numbers are here): one solid, its box and volume; a
    whole ring round the sleeve (no split), on a post standing on the disc's top face, the disc's draft carried up its
    outer face within the post; its foot the disc's rim past the leg's outer face, down to the underside, wider than the
    post; its 2 M4s through the foot from counterbores."""
    leg, f, d = motor_leg, DEFAULT.fork, DEFAULT.disc
    mx0, mx1 = fork_motor_leg_x(DEFAULT)
    assert leg.label == "j1_motor_leg" and leg.is_valid and len(leg.solids()) == 1
    assert R.solid_volume(leg) == pytest.approx(44356.285, abs=0.5)
    bb = leg.bounding_box()
    assert (bb.min.X, bb.min.Y, bb.min.Z) == pytest.approx((mx0, d.y0, f.axis_z - f.plate_r), abs=1e-6)
    assert (bb.max.X, bb.max.Y, bb.max.Z) == pytest.approx((disc_r(DEFAULT, d.y0), f.axis_y + f.plate_r, f.axis_z + f.plate_r), abs=1e-6)
    mm, r = (mx0 + mx1) / 2.0, f.ring_bore_dia / 2.0
    for dy, dz in ((-1.0, 0.0), (1.0, 0.0), (0.0, 1.0), (0.0, -1.0)):                   # a whole ring round the bore
        assert not is_inside(leg, mm, f.axis_y + dy * (r - 0.2), f.axis_z + dz * (r - 0.2))
        assert is_inside(leg, mm, f.axis_y + dy * (r + 0.2), f.axis_z + dz * (r + 0.2))
    # the post standing on the disc's top face; the draft up its outer face, within the post
    assert is_inside(leg, mm, d.top_y + 0.3, 0.0) and not is_inside(leg, mm, d.top_y - 0.3, 0.0)
    for y in (25.0, 40.0, 50.0):
        face = disc_r(DEFAULT, y)
        assert is_inside(leg, face - 0.3, y, 0.0) and not is_inside(leg, face + 0.3, y, 0.0)
    assert is_inside(leg, 46.2, 25.0, f.axis_z + f.leg_half_z - 0.3) and not is_inside(leg, 46.2, 25.0, f.axis_z + f.leg_half_z + 0.3)
    # the foot: the disc's rim past the leg's outer face, down to the underside, wider than the post
    assert is_inside(leg, mx1 + 0.3, d.y0 + 0.3, 0.0) and not is_inside(leg, mx1 - 0.3, d.y0 + 0.3, 0.0)
    assert is_inside(leg, disc_r(DEFAULT, 5.0) - 0.3, 5.0, 0.0) and not is_inside(leg, disc_r(DEFAULT, 5.0) + 0.3, 5.0, 0.0)
    assert is_inside(leg, mx1 + 0.3, 5.0, 30.0)
    # the 2 M4s through the foot, their heads' seats in counterbores
    seat_x = fork_motor_bolt_x(DEFAULT)[0]
    for y, z in fork_motor_bolts(DEFAULT):
        assert not is_inside(leg, mx1 + 2.0, y, z)                                                         # the hole
        assert is_inside(leg, seat_x - 0.5, y + 3.0, z) and not is_inside(leg, seat_x + 0.5, y + 3.0, z)   # the head's seat
