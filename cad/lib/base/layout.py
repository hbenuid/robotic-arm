"""The base's derived points and stations (pure math, no kernel): what lib/base/body.py and the tests share."""
from __future__ import annotations

import math

from lib.base.params import DEFAULT, YAW_BELT, YAW_BELT_Y, BaseConfig
from lib.belts import GT2_IDLER_WIDTH, GT2_PULLEY_20T_TEETH, GT2_PULLEY_120T_TEETH, idler_on_run, pulley_circle
from lib.motors import MKS_SERVO42D_W, NEMA17_BOLT_SP
from lib.sensors import (
    A3144_LEAD_T,
    KY003_BOARD_L,
    KY003_BOARD_W,
    KY003_HEADER_H,
    KY003_HOLE_INSET,
    KY003_LEAD_BEND_X,
    KY003_LEAD_TAIL,
    KY003_PIN_PITCH,
    KY003_PIN_TAIL,
    ky003_hall_point,
)


def yaw_belt_loop(cfg: BaseConfig = DEFAULT, *, idler: bool = True) -> list[tuple[float, float, float]]:
    """The base_yaw belt's loop (lib/belts.py) in the belt's own frame (yaw_belt_frame()): the motor's 20T on the
    origin, the 120T on the base_yaw axis on +X, the motor's centre distance from it, and - with cfg.idler - the
    tensioner on the run YawIdlerParams.run, `x` out from the axis, pushed in till the loop is YAW_BELT long (the stock
    belt is longer than the open loop)."""
    cx, cz = cfg.motor.centre
    c = math.hypot(cx, cz)
    loop = [pulley_circle(0.0, 0.0, GT2_PULLEY_20T_TEETH), pulley_circle(c, 0.0, GT2_PULLEY_120T_TEETH)]
    if idler and cfg.idler is not None:
        loop.insert(cfg.idler.run + 1, idler_on_run(loop, cfg.idler.run, c - cfg.idler.x, YAW_BELT))
    return loop


def yaw_belt_frame(cfg: BaseConfig = DEFAULT) -> tuple:
    """The base_yaw belt's frame as data in the base's: on the motor's axis at YAW_BELT_Y, its +Z up the base_yaw axis
    (the base's +Y), its +X toward the axis."""
    cx, cz = cfg.motor.centre
    return ((cx, YAW_BELT_Y, cz), (-90.0, 0.0, round(180.0 - math.degrees(math.atan2(cz, cx)), 6)))


def yaw_belt_point(bx: float, by: float, cfg: BaseConfig = DEFAULT) -> tuple[float, float]:
    """A point of the belt's frame (yaw_belt_frame(): x from the motor's axis toward the base_yaw axis, y across) as the
    base's (x, z): the frame's +X is -(cx, cz) / c, its +Y the base's +Z turned with it (the motor's small z)."""
    cx, cz = cfg.motor.centre
    c = math.hypot(cx, cz)
    ux, uz = -cx / c, -cz / c          # the frame's +X; its +Y is (uz, -ux)
    return (cx + bx * ux + by * uz, cz + bx * uz - by * ux)


def yaw_idler_xz(cfg: BaseConfig = DEFAULT) -> tuple[float, float]:
    """The base_yaw idler's axis (the base's x, z) with the stock belt: yaw_belt_loop()'s idler."""
    bx, by, _ = yaw_belt_loop(cfg)[cfg.idler.run + 1]
    return yaw_belt_point(bx, by, cfg)


def yaw_idler_slot(cfg: BaseConfig = DEFAULT) -> tuple[tuple[float, float], tuple[float, float]]:
    """The idler's slots' end centres (the base's x, z): `x` out along the belt's centre line, slot_y across it."""
    cx, cz = cfg.motor.centre
    bx = math.hypot(cx, cz) - cfg.idler.x
    return yaw_belt_point(bx, cfg.idler.slot_y[0], cfg), yaw_belt_point(bx, cfg.idler.slot_y[1], cfg)


def yaw_idler_y(cfg: BaseConfig = DEFAULT) -> float:
    """The idler's lower face along Y: level with the belt (its band centred on YAW_BELT_Y); the spacer's land on it."""
    return YAW_BELT_Y - GT2_IDLER_WIDTH / 2.0


def yaw_idler_spacer_h(cfg: BaseConfig = DEFAULT) -> float:
    """The idler's spacer: the plate's top to the idler's lower face."""
    return yaw_idler_y(cfg) - cfg.plate.y[1]


def motor_holes(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the base_yaw motor's 4 holes: the NEMA 17 square about the pattern centre."""
    (cx, cz), h = cfg.motor.centre, NEMA17_BOLT_SP / 2.0
    return [(cx + sx * h, cz + sz * h) for sx in (-1, 1) for sz in (-1, 1)]


def motor_face_y(cfg: BaseConfig = DEFAULT) -> float:
    """The base_yaw motor's mounting face: the pad's underside, MotorParams.pad_t under the plate's."""
    return round(cfg.plate.y[0] - cfg.motor.pad_t, 6)


def motor_screw_y(cfg: BaseConfig = DEFAULT) -> float:
    """The motor's screws' heads' bearing face (MotorParams.holes): the counterbores' floor, head_h under the plate's top."""
    return round(cfg.plate.y[1] - cfg.motor.holes.screw.head_h, 6)


def outer_r(cfg: BaseConfig, y: float) -> float:
    """The walls' outer radius (the round half's, the sides' |z|) at height y: r at the cap's top face, `draft` more per
    mm down."""
    s = cfg.shell
    return s.r + s.draft * (cfg.cap.top_y - y)


def inner_r(cfg: BaseConfig, y: float) -> float:
    """The walls' inner radius at height y: `wall` in from the outside."""
    return outer_r(cfg, y) - cfg.shell.wall


def side_stub_x(cfg: BaseConfig = DEFAULT) -> float:
    """Where the straight sides stop above the plate: the inside of a side (z = +/- (r - wall)) meets the outer round -
    at the plate's top face, where the leaning walls are widest above it."""
    r = outer_r(cfg, cfg.plate.y[1])
    return math.sqrt(r ** 2 - (r - cfg.shell.wall) ** 2)


def flare_points(cfg: BaseConfig = DEFAULT) -> tuple[tuple[float, float], tuple[float, float]]:
    """The foot's chamfer as (radius, y) ends: its foot on the flange's top, its top on the wall's outside."""
    s, f = cfg.shell, cfg.foot
    top_y = s.y0 + f.flare_h
    return (f.flare_r, s.y0 + f.t), (outer_r(cfg, top_y), top_y)


def foot_holes(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the foot's screw holes: on the circle hole_r at hole_deg on the round half, at side_hole_x on the
    sides."""
    f = cfg.foot
    pts = [(f.hole_r * math.cos(math.radians(a)), f.hole_r * math.sin(math.radians(a))) for a in f.hole_deg]
    return pts + [(f.side_hole_x, sz * f.hole_r) for sz in (1, -1)]


def chamfer_inset(cfg: BaseConfig = DEFAULT) -> float:
    """How far the 45 degree chamfer under the cap runs in from the wall's inside: its height."""
    return cfg.cap.underside_y - cfg.cap.chamfer_y0


def mount_inner_half(cfg: BaseConfig = DEFAULT) -> float:
    """The window between the joint's posts (|z| of the posts' inner faces, about the axis; the cables' way from the
    motor mount into the base): the MKS board's square about the motor's centre, and MountParams.room clear of it."""
    return MKS_SERVO42D_W / 2.0 + abs(cfg.motor.centre[1]) + cfg.mount.room


def mount_x1(cfg: BaseConfig = DEFAULT) -> float:
    """How far the motor mount's round end reaches along X at the plate's top: room past the board's square at the
    motor's centre, and the wall."""
    return cfg.motor.centre[0] + MKS_SERVO42D_W / 2.0 + cfg.mount.room + cfg.shell.wall


def mount_end_x(cfg: BaseConfig = DEFAULT) -> float:
    """The centre (on the X axis) of the motor mount's round end: the walls' radius at the plate's top short of
    mount_x1(), so the end round is the base's round half's, mirrored."""
    return mount_x1(cfg) - outer_r(cfg, cfg.plate.y[1])


def mount_foot_holes(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the motor mount's foot's screw holes: on hole_r about its end's centre at mount_hole_deg."""
    f, cx = cfg.foot, mount_end_x(cfg)
    return [(cx + f.hole_r * math.cos(math.radians(a)), f.hole_r * math.sin(math.radians(a))) for a in f.mount_hole_deg]


def slit_band(cfg: BaseConfig = DEFAULT) -> tuple[float, float]:
    """The slits' band along Y (SlitParams): over_flare above the foot's chamfer, h tall."""
    y0 = flare_points(cfg)[1][1] + cfg.slits.over_flare
    return (y0, y0 + cfg.slits.h)


def slits(cfg: BaseConfig = DEFAULT, part: str = "base") -> list[tuple[float, float, float]]:
    """The slits of one part ("base" or "mount") as (x, z, deg): each one's centre on the wall's mid-surface at the
    band's middle and its outward normal's angle (in the XZ plane, from +X toward +Z). The round ends - the base's half
    round about the axis, the mount's about mount_end_x() - as many at SlitParams.pitch along that mid-surface as fit,
    from side to side; the straight sides at multiples of pitch from the axis, short of the mount's round by half a
    pitch, none within joint_clear of the two parts' posts; each side's slit with the part it falls in."""
    sl, j, sides = cfg.slits, cfg.joint, cfg.shell.sides
    y0, y1 = slit_band(cfg)
    rm = outer_r(cfg, (y0 + y1) / 2.0) - cfg.shell.wall / 2.0
    cx, a0 = (0.0, 90.0) if part == "base" else (mount_end_x(cfg), -90.0)
    pts = []
    if sides is None:
        n = round(math.pi * rm / sl.pitch)
        for k in range(n + 1):
            a = math.radians(a0 + k * 180.0 / n)
            pts.append((cx + rm * math.cos(a), rm * math.sin(a), a0 + k * 180.0 / n))
        x_lo, x_hi = sl.pitch, mount_end_x(cfg) - sl.pitch / 2.0
    else:
        # faceted (ShellParams.sides): each flat of the end between the two sides' planes gets its slits at `pitch`
        # along it, centred on it and half a pitch clear of its corners, cut along its normal; the sides' planes run on
        # round onto the ends, the same half pitch short of the corners
        half = rm * math.tan(math.pi / sides) - sl.pitch / 2.0
        steps = range(-math.floor(half / sl.pitch), math.floor(half / sl.pitch) + 1)
        for k in range(1, sides // 2):
            deg = a0 + k * 360.0 / sides
            a = math.radians(deg)
            for m in steps:
                t = m * sl.pitch
                pts.append((cx + rm * math.cos(a) - t * math.sin(a), rm * math.sin(a) + t * math.cos(a), deg))
        x_lo, x_hi = -half, mount_end_x(cfg) + half
    gap = (j.split_x - j.post_t - sl.joint_clear, j.split_x + cfg.mount.post_t + sl.joint_clear)
    k = math.ceil(x_lo / sl.pitch - 1e-9)
    while k * sl.pitch <= x_hi + 1e-9:
        x = k * sl.pitch
        if not gap[0] < x < gap[1] and (x < j.split_x) == (part == "base"):
            pts += [(x, rm, 90.0), (x, -rm, -90.0)]
        k += 1
    return pts


def joint_bolt_points(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(z, y) of the 4 M4 through the mount's posts and the base's (JointParams, MountParams): bolt_out out of the
    window's edge, bolt_inset under the plate's underside and above the bottom face - also the (x, y) of the screw / nut
    patterns (base_motor_mount_screws), whose +Z the mounts turn down the base's -X."""
    s, j = cfg.shell, cfg.joint
    z = mount_inner_half(cfg) + j.bolt_out
    return [(sz * z, y) for sz in (1, -1) for y in (cfg.plate.y[0] - j.bolt_inset, s.y0 + j.bolt_inset)]


def joint_stations(cfg: BaseConfig = DEFAULT) -> dict[str, float]:
    """The joint's stations along X: the base posts' back face (the nuts' outer faces flush with it), the joint face,
    the mount posts' inner face (under the screws' heads), the nuts' bearing faces and the screws' tips."""
    j = cfg.joint
    back, head = j.split_x - j.post_t, j.split_x + cfg.mount.post_t
    return {"x_post": back, "x_split": j.split_x, "x_head": head, "x_nut_face": back + j.nut.h,
            "x_tip": head - j.screw_len}


# --- The base_yaw home sensor (YawHomeParams): the KY-003 under the 120T on its bracket under the plate ---------------
# The module's frame (lib/sensors.py: the board's underside on z = 0, its sensor end at x = 0, the chip toward -X, the
# header toward +X, the branded face +Z) in the base's: its +Z up the base's +Y, its +X (the header) along the base's
# -X, its +Y along the base's +Z - (x, y, z) -> (-x, z, y).
KY003_TURN = (-90.0, 0.0, 180.0)


def home_chip_xz(cfg: BaseConfig = DEFAULT) -> tuple[float, float]:
    """The KY-003's chip's centre (x, z): chip_r off the axis at chip_deg."""
    h = cfg.home
    a = math.radians(h.chip_deg)
    return (round(h.chip_r * math.cos(a), 9) + 0.0, round(h.chip_r * math.sin(a), 9) + 0.0)


def home_board_y(cfg: BaseConfig = DEFAULT) -> float:
    """The KY-003's board's underside: on the bracket's top, which lies on the plate's underside."""
    return cfg.plate.y[0]


def home_sensor_frame(cfg: BaseConfig = DEFAULT) -> tuple:
    """The KY-003's part frame in the base's: its chip's centre (ky003_hall_point()'s x, y) over home_chip_xz()."""
    cx, cz = home_chip_xz(cfg)
    hx, hy, _ = ky003_hall_point()
    return ((round(cx + hx, 6), home_board_y(cfg), round(cz - hy, 6)), KY003_TURN)


def home_to_base(cfg: BaseConfig, x: float, y: float) -> tuple[float, float]:
    """A point (x, y) of the KY-003's board plane (its part frame) as the base's (x, z)."""
    (ox, _, oz), _ = home_sensor_frame(cfg)
    return (round(ox - x, 6), round(oz + y, 6))


def home_bracket_frame(cfg: BaseConfig = DEFAULT) -> tuple:
    """The bracket's part frame in the base's (its fasteners' too): z = 0 on its underside, bracket_t under the plate's,
    +Z up the base's +Y, the base's (x, z) at its (x, -y) - lib/base/body.py's working frame."""
    return ((0.0, round(cfg.plate.y[0] - cfg.home.bracket_t, 6), 0.0), (-90.0, 0.0, 0.0))


def home_outline(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the bracket's outline in the base's frame."""
    cx, cz = home_chip_xz(cfg)
    return [(round(cx + x, 6), round(cz + z, 6)) for x, z in cfg.home.outline]


def home_screw_points(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the bracket's 2 M3s, through it and the plate into their nuts in the plate's top."""
    cx, cz = home_chip_xz(cfg)
    return [(round(cx + x, 6), round(cz + z, 6)) for x, z in cfg.home.screw_points]


def home_board_points(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the board's 2 M2s: its own holes (KY003_HOLE_INSET in from its sensor end and its long edges)."""
    y = KY003_BOARD_W / 2.0 - KY003_HOLE_INSET
    return [home_to_base(cfg, KY003_HOLE_INSET, s * y) for s in (1.0, -1.0)]


def _board_box(cfg: BaseConfig, x: tuple, y: tuple) -> tuple[float, float, float, float]:
    """A box of the board plane, x / y ranges in the KY-003's frame, as the base's (x0, x1, z0, z1)."""
    xs, zs = zip(*(home_to_base(cfg, a, b) for a in x for b in y), strict=True)
    return (min(xs), max(xs), min(zs), max(zs))


def home_board_box(cfg: BaseConfig = DEFAULT) -> tuple[float, float, float, float]:
    """The board's footprint, (x0, x1, z0, z1) in the base's frame."""
    return _board_box(cfg, (0.0, KY003_BOARD_L), (-KY003_BOARD_W / 2.0, KY003_BOARD_W / 2.0))


def home_reliefs(cfg: BaseConfig = DEFAULT) -> list[tuple[float, float, float, float, float]]:
    """The bracket's 2 slots, (x0, x1, z0, z1, depth under the board): across the header's pins' tails (one pitch behind
    its housing) and the chip's leads' tails (where they turn down), relief_w wide, relief_past deeper than each."""
    h = cfg.home
    pin_x = KY003_BOARD_L - KY003_HEADER_H / 2.0 - KY003_PIN_PITCH
    lead_x = KY003_LEAD_BEND_X + A3144_LEAD_T / 2.0
    rows = ((pin_x, h.header_relief_len, KY003_PIN_TAIL), (lead_x, h.lead_relief_len, KY003_LEAD_TAIL))
    return [(*_board_box(cfg, (x - h.relief_w / 2.0, x + h.relief_w / 2.0), (-ln / 2.0, ln / 2.0)), tail + h.relief_past)
            for x, ln, tail in rows]
