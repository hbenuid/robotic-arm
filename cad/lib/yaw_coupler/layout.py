"""The yaw coupler's derived points and outlines (pure math, no kernel): what lib/yaw_coupler/body.py and the tests
share. Outlines in the YZ plane are (y, z) pairs; the disc's profile is (r, y)."""
from __future__ import annotations

import math

from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.cycloidal import PILLAR_OVERSHOOT, arm_mount_points, pillar_half_width, shell_ends
from lib.fasteners import M4_NUT, M4_SHCS
from lib.yaw_coupler.params import DEFAULT, YawCouplerConfig

COUNTERBORE = M4_SHCS.head_dia + 0.4    # [DESIGN] the fork's M4 heads' counterbores (the drive's housing bolts' 7.4)
HEAD_SEAT = M4_SHCS.head_h + 0.5         # [DESIGN] ... the hub leg's this deep (4.5, the drive's)
NUT_SLOT = 0.2                           # [DESIGN] the motor leg's nuts' slots: this over the nut's AF and height


def disc_r(cfg: YawCouplerConfig, y: float) -> float:
    """The disc's radius at height y: the band, then the draft in to top_r."""
    d = cfg.disc
    if y <= d.band_y1:
        return d.band_r
    return d.band_r + (d.top_r - d.band_r) * (y - d.band_y1) / (d.top_y - d.band_y1)


def disc_profile(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(r, y) of the disc's half section, from the axis at the underside round to the axis at the top face."""
    d = cfg.disc
    band = [(d.band_r, d.band_y1)] if d.y0 < d.band_y1 else []
    return [(0.0, d.y0), (disc_r(cfg, d.y0), d.y0), *band, (d.top_r, d.top_y), (0.0, d.top_y)]


def flare_r(cfg: YawCouplerConfig, y: float) -> float:
    """The flare's radius at height y: the cone through the ring's top edge, its apex on the axis."""
    d, k = cfg.disc, cfg.yoke
    return d.ring_r * (y - k.flare_apex_y) / (d.ring_y1 - k.flare_apex_y)


def below_axis(cfg: YawCouplerConfig, r: float, z: float) -> float:
    """y of the point at |z| on the circle of r about the drive's axis, below the axis."""
    return cfg.yoke.axis_y - math.sqrt(r * r - z * z)


def _mirrored(half: list[tuple[float, float]]) -> list[tuple[float, float]]:
    """A (y, z) outline symmetric about z = 0 from its +Z half (listed bottom up)."""
    return half + [(y, -z) for y, z in reversed(half)]


def _bottom(cfg: YawCouplerConfig) -> float:
    """Where the yoke's prisms start: below the flare, which bounds them."""
    return cfg.disc.ring_y1 - 1.0


def od_point(cfg: YawCouplerConfig = DEFAULT) -> tuple[float, float]:
    """(y, z) where the outer face meets the housing's od - the yoke's widest point, both prisms' corner."""
    k = cfg.yoke
    return (below_axis(cfg, k.od_r, k.outer_z), k.outer_z)


def cheek_outline(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the -X cheek: up the outer face to the od point, straight to the cradle at cheek_top_z."""
    k = cfg.yoke
    return _mirrored([(_bottom(cfg), k.outer_z), od_point(cfg), (below_axis(cfg, k.cradle_r, k.cheek_top_z), k.cheek_top_z),
                      (k.axis_y, k.cheek_top_z)])


def middle_outline(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the middle body: up the outer face to the od point, the V-groove round the 45 degree pillar - across
    its end on the od, then down its near side to the cradle; with no V-grooves, the cheek's outline."""
    k = cfg.yoke
    if k.groove_z is None:
        return cheek_outline(cfg)
    near, end = k.groove_z
    return _mirrored([(_bottom(cfg), k.outer_z), od_point(cfg), (below_axis(cfg, k.od_r, end), end),
                      (below_axis(cfg, k.cradle_r, near), near), (k.axis_y, near)])


def channel_outline(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the channel under the bottom pillar: the floor, the walls opening up to the drive's axis."""
    k = cfg.yoke
    top = k.channel_half_z + (k.axis_y - k.channel_floor_y) * k.channel_slope
    return _mirrored([(k.channel_floor_y, k.channel_half_z), (k.axis_y, top)])


def pillar_point(cfg: YawCouplerConfig, deg: float, along: float, across: float) -> tuple[float, float]:
    """(y, z) of the point `along` the centre line of the drive's pillar at deg from straight down (toward +Z) from the
    drive's axis and `across` it (toward +Z at deg 0)."""
    a = math.radians(deg)
    return (cfg.yoke.axis_y - along * math.cos(a) + across * math.sin(a), along * math.sin(a) + across * math.cos(a))


def socket_outline(cfg: YawCouplerConfig, deg: float, wall: float = 0.0) -> list[tuple[float, float]]:
    """(y, z) of the socket round the drive's pillar at deg from straight down: its walls the pillar's sides
    (lib/cycloidal/layout.py pillar_half_width) socket_clear out, its floor flat across its centre line socket_clear
    past the housing's od (the pillar's end is an arc on it, its corners chamfered), its mouth where the pillar's own
    inner end is - inside the cradle, so the socket opens into it. `wall` further out on the walls and the floor: the
    outline of the socket's wall (socket_wall)."""
    k, h = cfg.yoke, DRIVE.housing
    gap = k.socket_clear + wall
    mouth, floor = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + gap
    grow = gap * math.hypot(1.0, pillar_half_width(DRIVE, 1.0) - pillar_half_width(DRIVE, 0.0))
    return [pillar_point(cfg, deg, r, side * (pillar_half_width(DRIVE, r) + grow))
            for r, side in ((mouth, -1.0), (floor, -1.0), (floor, 1.0), (mouth, 1.0))]


def nut_centres(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the housing bolts the cheek's nut pockets sit on: on the bolt circle about the drive's axis."""
    k = cfg.yoke
    return [(k.axis_y - k.bolt_circle_r * math.cos(math.radians(a)), k.bolt_circle_r * math.sin(math.radians(a)))
            for a in k.bolt_deg]


def hole_points(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, z) of the hub's 4 holes: on hole_r from hole_deg, 90 degrees apart."""
    h = cfg.hub
    return [(h.hole_r * math.cos(math.radians(h.hole_deg + 90.0 * i)), h.hole_r * math.sin(math.radians(h.hole_deg + 90.0 * i)))
            for i in range(4)]


# ---- the fork (ForkParams, DEFAULT) -------------------------------------------------------------------------------------
def fork_x(cfg: YawCouplerConfig, z: float) -> float:
    """This frame's x at the drive's z (the drive's +Z is this frame's -X)."""
    return cfg.fork.face_x - z


def fork_hub_leg_x(cfg: YawCouplerConfig = DEFAULT) -> tuple[float, float]:
    """(outer, inner) x of the hub leg: end_plate_gap past the shell's hub end, yoke_leg thick (the hub's face on its
    inner face)."""
    sh, z = DRIVE.shell, shell_ends(DRIVE)[1] + DRIVE.shell.end_plate_gap
    return fork_x(cfg, z + sh.yoke_leg), fork_x(cfg, z)


def fork_motor_leg_x(cfg: YawCouplerConfig = DEFAULT) -> tuple[float, float]:
    """(inner, outer) x of the motor leg: end_plate_gap past the shell's motor end, yoke_leg thick (the sleeve's end on
    its outer face)."""
    sh, z = DRIVE.shell, shell_ends(DRIVE)[0] - DRIVE.shell.end_plate_gap
    return fork_x(cfg, z), fork_x(cfg, z - sh.yoke_leg)


def fork_hub_bolts(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the hub's 4 bolts through the hub leg: the drive's arm-mount pattern (its +X is this frame's +Z)."""
    f = cfg.fork
    return [(f.axis_y + y, f.axis_z + x) for x, y in arm_mount_points(DRIVE)]


def fork_flare_top(cfg: YawCouplerConfig = DEFAULT) -> float:
    """y where the disc's draft up the legs' outer faces stops: flare_seat_flat under the counterbores of the hub's
    lowest bolts (the mounting area above it is the leg's flat yoke_leg)."""
    return min(y for y, _ in fork_hub_bolts(cfg)) - COUNTERBORE / 2.0 - cfg.fork.flare_seat_flat


def fork_motor_bolts(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(y, z) of the motor leg's 2 M4s along the drive's axis: motor_bolt_y high, motor_bolt_z either side of it."""
    f = cfg.fork
    return [(f.motor_bolt_y, f.axis_z + sz * f.motor_bolt_z) for sz in (-1.0, 1.0)]


def fork_motor_bolt_x(cfg: YawCouplerConfig = DEFAULT) -> tuple[float, float, float]:
    """x of the motor leg's M4s: (their heads' seats in the foot, their nuts' slot - its outer and inner wall) - the
    foot butting this part at the leg's outer face."""
    f, x1 = cfg.fork, fork_motor_leg_x(cfg)[1]
    nut_x1 = x1 - f.motor_nut_wall
    return x1 + f.motor_head_seat, nut_x1, nut_x1 - M4_NUT.h - NUT_SLOT
