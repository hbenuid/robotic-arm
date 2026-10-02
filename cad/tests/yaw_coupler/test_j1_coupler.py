"""j1_coupler, parametric (lib/yaw_coupler/): the LEGACY configuration reproduces the SolidWorks part feature by
feature (the volume / bbox match is tests/test_reference_match.py's; here the features are probed by name so a
regression names what moved), the yoke's shapes are the drive's housing's - its bore, its od, its pillars' sides,
its bolt circle and nut pockets (lib/cycloidal/params.py HousingParams; LEGACY's the port's housing, LEGACY_CONFIG) -,
and DEFAULT - what the part builds - changes its underside, its hub and the yoke: the seat on the thrust bearing, the
rim clear of the base (tests/test_mounts.py checks the stack in place), the stub on to the lip's lower face and drilled
for the base_yaw pulley, and the fork (ForkParams) round the drive whose shell turns (DEFAULT_CONFIG's ShellParams): two
alike legs past the shell's ends, mirrored about the middle of the discs - the held hub bolts to one, the other clamps
the motor plate's sleeve: its lower half this part, its upper half the cap parts/base/j1_coupler_cap (no reference: its
numbers are here) -, the ring lowered under the shell (tests/cycloidal/test_assembly.py checks the drive in place,
tests/test_sweeps.py the arm's swing)."""
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
from lib.fasteners import M3_NUT, M4_CLEAR, M4_NUT, M4_SHCS
from lib.geom import hex_circumdiameter
from lib.yaw_coupler import DEFAULT, LEGACY, hole_points, nut_centres, od_point
from lib.yaw_coupler.layout import below_axis, fork_cap_bolts, fork_hub_bolts, fork_hub_leg_x, fork_motor_leg_x, fork_x
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
    discs, which the shift (lib/placements.py SHIFTS) puts on the base_yaw axis; the hub's 4 bolts on the drive's arm-mount pattern, their ends flush with the hub's captive nuts; the motor
    leg round the sleeve (its bore 0.2 a side), the cap's 2 screws in the leg's middle, their nuts' slots clear of the
    bore; the ring lowered under the turning shell's pillars (RING_DROP: 2.68 of clearance)."""
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
    assert f.clamp_bore_dia == pytest.approx(sh.sleeve_od + 0.4)
    for x, z in fork_cap_bolts(DEFAULT):
        assert x == pytest.approx((mx0 + mx1) / 2.0) and mx0 < x - (M3_NUT.af + 0.2) / 2.0 and x + (M3_NUT.af + 0.2) / 2.0 < mx1
        dy = f.cap_nut_y + M3_NUT.h + 0.2                            # the slot's floor below the split
        bore_z = math.sqrt((f.clamp_bore_dia / 2.0) ** 2 - dy ** 2)
        assert abs(z - f.axis_z) - hex_circumdiameter(M3_NUT.af + 0.2) / 2.0 - bore_z > 2.5
        assert abs(z - f.axis_z) + 1.7 < f.plate_r
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
                                                         ring_x0=LEGACY.disc.ring_x0), fork=None) == LEGACY
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
    assert not is_inside(coupler, 47.5, y0 - 0.05, 0) and is_inside(coupler, 47.5, y0 + 0.05, 0)   # the ears with it
    assert is_inside(coupler, 0, y1 - 0.05, 14.5) and is_inside(coupler, 0, -5, 14.7)          # the stub up into the seat
    for probe in ((0, 2, 7.7), (0, 7.8, 20)):
        assert is_inside(coupler, *probe) == is_inside(legacy, *probe)                         # the hub, the pocket's floor
    assert not is_inside(coupler, 0, 10, 20) and not is_inside(legacy, 0, 10, 20)             # the pocket over the hub


@pytest.mark.slow
def test_default_is_the_fork(coupler, legacy):
    """The fork round the drive whose shell turns (ForkParams): the hub leg on -X - its disc round the axis on a leg down
    to the disc's underside, straddling the disc's flat on a low bridge -, the motor leg on +X the same, bored to the
    sleeve and split at the drive's axis (the cap is its own part); along X the disc's ears the widest (the legs within
    them); the yoke's cheek, middle body, cradle and sockets gone; the ring lowered."""
    f, d = DEFAULT.fork, DEFAULT.disc
    (hx0, hx1), (mx0, mx1) = fork_hub_leg_x(DEFAULT), fork_motor_leg_x(DEFAULT)
    bb = coupler.bounding_box()
    r0 = d.band_r + (d.top_r - d.band_r) * (d.y0 - d.band_y1) / (d.top_y - d.band_y1)     # the disc at the rim, lifted
    assert (bb.min.X, bb.max.X, bb.max.Y) == pytest.approx((-d.ear_r, d.ear_r, f.axis_y + f.plate_r), abs=1e-6)
    assert -d.ear_r < hx0 and mx1 < d.ear_r
    assert (bb.min.Z, bb.max.Z) == pytest.approx((-r0, r0), abs=1e-6)
    assert hx0 < -d.flat_x < hx1 and mx0 < d.flat_x < mx1           # each leg straddling a flat
    for x0, x1 in ((hx0, hx1), (mx0, mx1)):
        m = (x0 + x1) / 2.0
        # each leg: a disc round the axis (the motor leg's upper half the cap's) on a leg down to the disc's underside
        assert is_inside(coupler, m, 30.0, f.axis_z + f.leg_half_z - 0.3) and not is_inside(coupler, m, 30.0, f.axis_z + f.leg_half_z + 0.3)
        assert is_inside(coupler, m, 20.0, 0.0)
        if x0 > d.flat_x:                                       # (over the hub, the coupler's recess for the thrust stack)
            assert is_inside(coupler, m, d.y0 + 0.3, 0.0)
        assert is_inside(coupler, m, f.axis_y - 0.3, f.axis_z + f.plate_r - 1.0) and not is_inside(coupler, m, f.axis_y - 0.3, f.axis_z + f.plate_r + 0.3)
        assert is_inside(coupler, x0 + 0.3, 40.0, 0.0) and not is_inside(coupler, x0 - 0.3, 40.0, 0.0)
        assert is_inside(coupler, x1 - 0.3, 40.0, 0.0) and not is_inside(coupler, x1 + 0.3, 40.0, 0.0)
    hm = (hx0 + hx1) / 2.0
    assert is_inside(coupler, hm, f.axis_y + f.plate_r - 0.3, f.axis_z) and not is_inside(coupler, hm, f.axis_y + f.plate_r + 0.3, f.axis_z)
    for x in (hx0 + 0.3, mx1 - 0.3):                                   # each leg down to the disc's underside, past the recess
        assert is_inside(coupler, x, d.y0 + 0.3, 0.0) and DEFAULT.hub.recess_dia / 2.0 < abs(x)
    for y, z in fork_hub_bolts(DEFAULT):
        assert not is_inside(coupler, hx1 - 0.3, y, z) and not is_inside(coupler, hx0 + 0.3, y + 3.0, z)   # the hole, the counterbore
        assert is_inside(coupler, hx1 - 0.3, y + 3.0, z)                                                  # ... 4.5 deep
    # the ring lowered
    assert is_inside(coupler, -25.0, d.ring_y1 - 0.3, 25.0) and not is_inside(coupler, -25.0, d.ring_y1 + 0.3, 25.0)
    assert is_inside(legacy, -25.0, d.ring_y1 + 0.3, 25.0)
    # the motor leg: bored to the sleeve, split at the axis
    mm = (mx0 + mx1) / 2.0
    r = f.clamp_bore_dia / 2.0
    assert not is_inside(coupler, mm, f.axis_y - r + 0.2, f.axis_z) and is_inside(coupler, mm, f.axis_y - r - 0.2, f.axis_z)
    assert not is_inside(coupler, mm, f.axis_y + 0.3, f.axis_z + f.plate_r - 1.0)              # the cap's
    for x, z in fork_cap_bolts(DEFAULT):
        out = 1.0 if z > f.axis_z else -1.0
        assert not is_inside(coupler, x, f.axis_y - 2.0, z)                                    # the screw's hole ...
        slot = f.axis_y - f.cap_nut_y - (M3_NUT.h + 0.2) / 2.0
        assert not is_inside(coupler, x, slot, z) and not is_inside(coupler, x, slot, f.axis_z + out * 42.0)   # ... its nut's slot to the side
        assert is_inside(coupler, x, f.axis_y - f.cap_nut_y + 0.3, f.axis_z + out * 42.0)
        assert is_inside(coupler, x + 3.4, slot, z)                                             # ... the nut's AF wide
    # the yoke that held the port's housing is gone
    for probe in ((0, 40, 52.9), (20, 34.5, 20), (-31, 45.8, 49.2)):
        assert not is_inside(coupler, *probe) and is_inside(legacy, *probe)


@pytest.fixture(scope="module")
def cap():
    return built.part("j1_coupler_cap")


@pytest.mark.slow
def test_cap(cap):
    """The motor leg's upper half (parts/base/j1_coupler_cap, no reference - its numbers are here): one solid, its box
    and volume, bored to the sleeve, its 2 screws through it from counterbores cap_seat above the split."""
    f = DEFAULT.fork
    mx0, mx1 = fork_motor_leg_x(DEFAULT)
    assert cap.label == "j1_coupler_cap" and cap.is_valid and len(cap.solids()) == 1
    assert R.solid_volume(cap) == pytest.approx(9182.805, abs=0.5)
    bb = cap.bounding_box()
    assert (bb.min.X, bb.min.Y, bb.min.Z) == pytest.approx((mx0, f.axis_y, f.axis_z - f.plate_r), abs=1e-6)
    assert (bb.size.X, bb.size.Y, bb.size.Z) == pytest.approx((mx1 - mx0, f.plate_r, 2.0 * f.plate_r), abs=1e-6)
    cm, r = (mx0 + mx1) / 2.0, f.clamp_bore_dia / 2.0
    assert not is_inside(cap, cm, f.axis_y + r - 0.2, f.axis_z) and is_inside(cap, cm, f.axis_y + r + 0.2, f.axis_z)
    for x, z in fork_cap_bolts(DEFAULT):
        assert not is_inside(cap, x, f.axis_y + 1.0, z)                                         # the hole through ...
        assert is_inside(cap, x + 2.4, f.axis_y + f.cap_seat - 0.5, z)                          # ... under the head ...
        assert not is_inside(cap, x + 2.4, f.axis_y + f.cap_seat + 0.5, z)                      # ... in its counterbore
