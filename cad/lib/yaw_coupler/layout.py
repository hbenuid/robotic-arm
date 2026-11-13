"""The yaw coupler's derived points and outlines (pure math, no kernel): what lib/yaw_coupler/body.py and the tests
share. Outlines in the YZ plane are (y, z) pairs; the disc's profile is (r, y)."""
from __future__ import annotations

import math

from lib.cycloidal import DEFAULT_CONFIG as DRIVE
from lib.cycloidal import (
    PILLAR_OVERSHOOT,
    arm_mount_points,
    pillar_half_width,
    shell_ends,
    sleeve_bolt_z,
    stack_positions,
)
from lib.datum import BASE_TURN_DEG, CAPTURE_YAW_DEG, angles, compose, rotation
from lib.fasteners import M4_NUT, M4_SHCS
from lib.pulley.params import YAW, YAW_MAGNET_DEG, PulleyParams
from lib.sensors import (
    A3144_HALL_DEPTH,
    A3144_LEAD_T,
    HOME_MAGNET_T,
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
from lib.style import EDGE_CHAMFER
from lib.upper_arm.params import DEFAULT as _UPPER_ARM
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


def disc_corners(cfg: YawCouplerConfig, y: float) -> list[tuple[float, float, float]]:
    """(x, y, z) of the faceted disc's corners at height y (DiscParams.sides): its flats disc_r(y) off the base_yaw axis,
    one facing each of +/-X and +/-Z (lib/geom.py faceted_end, turned onto this frame's XZ plane)."""
    n = cfg.disc.sides
    r = disc_r(cfg, y) / math.cos(math.pi / n)
    return [(r * math.cos(math.pi * (2 * k + 1) / n), y, r * math.sin(math.pi * (2 * k + 1) / n)) for k in range(n)]


def fork_post_y(cfg: YawCouplerConfig = DEFAULT) -> float:
    """y where the legs' posts end and their faceted roots begin: where the disc's draft reaches the legs' outer faces
    (the two mirror each other) - the disc's flat facing a leg runs on up its outer face to there, in one plane."""
    d, x_out = cfg.disc, abs(fork_hub_leg_x(cfg)[0])
    return d.band_y1 + (d.band_r - x_out) * (d.top_y - d.band_y1) / (d.band_r - d.top_r)


def fork_root_points(cfg: YawCouplerConfig, x_in: float, x_out: float) -> list[tuple[float, float, float]]:
    """The corners whose convex hull is the faceted root under the leg between its inner face x_in and outer face x_out
    (lib/yaw_coupler/body.py): the post's section at fork_post_y - plate_r either side of the drive's axis, its outline
    chamfered EDGE_CHAMFER at 45 deg -, and the disc's corners outboard of the inner face at its rim (y0) and its top face
    (top_y), with the points where the disc's edges there cross that face."""
    d, f = cfg.disc, cfg.fork
    s, y = math.copysign(1.0, x_out), fork_post_y(cfg)
    a, b, h, c = min(x_in, x_out), max(x_in, x_out), f.plate_r, EDGE_CHAMFER
    post = [(a, -h + c), (a + c, -h), (b - c, -h), (b, -h + c), (b, h - c), (b - c, h), (a + c, h), (a, h - c)]
    points = [(x, y, f.axis_z + z) for x, z in post]
    for level in (d.y0, d.top_y):
        corners = disc_corners(cfg, level)
        for p, q in zip(corners, corners[1:] + corners[:1], strict=True):
            if s * p[0] > abs(x_in):
                points.append(p)
            if (s * p[0] > abs(x_in)) != (s * q[0] > abs(x_in)):
                t = (s * abs(x_in) - p[0]) / (q[0] - p[0])
                points.append((s * abs(x_in), level, p[2] + t * (q[2] - p[2])))
    return points


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


def fork_sleeve_bolt_x(cfg: YawCouplerConfig = DEFAULT) -> float:
    """x of the motor leg's radial M4s into the sleeve (lib/cycloidal/layout.py sleeve_bolt_z): the ring's middle."""
    return fork_x(cfg, sleeve_bolt_z(DRIVE))


def fork_sleeve_bolt_deg(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """Their angles about the drive's axis, from its +X (this frame's +Z) toward its +Y (this frame's +Y)."""
    return DRIVE.shell.sleeve_bolt_deg


# ---- the fork's screws and nuts (parts/base/fork_{hub,foot,sleeve}_{screws,nuts}, lib/mounts.py FORK_MOUNTS) -------------
# Each screw set's frame in this one: its screws along its +Z from the heads' bearing faces (z 0, the heads in -Z); its
# nut set's frame is the screw set's moved along +Z to the nuts' bearing faces. The hub's and the foot's +Z run along
# the drive's axis, their +Y up this frame's +Y; the sleeve's frame sits on the drive's axis, its axes the drive's, the
# screws radial in its z = 0 plane.
HUB_SCREWS_TURN = (0.0, 90.0, 0.0)      # +Z along this frame's +X (into the hub: the drive's -Z), +X along its -Z
FOOT_SCREWS_TURN = (0.0, -90.0, 0.0)    # +Z along this frame's -X (through the foot into the disc), +X along its +Z
SLEEVE_SCREWS_TURN = (0.0, -90.0, 0.0)  # +Z the drive's +Z (this frame's -X), +X the drive's +X (this frame's +Z)


def fork_hub_screw_frame(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """The hub's 4 screws' frame: on the drive's axis, in the plane of the floors of the hub leg's counterbores."""
    f = cfg.fork
    return ((round(fork_hub_leg_x(cfg)[0] + HEAD_SEAT, 6), f.axis_y, f.axis_z), HUB_SCREWS_TURN)


def fork_hub_screw_points() -> list[tuple[float, float]]:
    """(x, y) of the hub's 4 screws in their frame: the drive's arm-mount pattern, its x turned over (the frame's +X is
    the drive's -X)."""
    return [(-x, y) for x, y in arm_mount_points(DRIVE)]


def fork_hub_nut_z(cfg: YawCouplerConfig = DEFAULT) -> float:
    """The hub's captive nuts' bearing faces along the screws: the floors of their pockets, ShellParams.hub_nut_depth in
    from the hub's flange's inner face."""
    return round(fork_x(cfg, stack_positions(DRIVE)["z_hub"] + DRIVE.shell.hub_nut_depth) - fork_hub_screw_frame(cfg)[0][0], 6)


def fork_foot_screw_frame(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """The motor leg's foot's 2 screws' frame: motor_bolt_y high over the drive's axis, in the plane of the floors of the
    foot's counterbores."""
    f = cfg.fork
    return ((round(fork_motor_bolt_x(cfg)[0], 6), f.motor_bolt_y, f.axis_z), FOOT_SCREWS_TURN)


def fork_foot_screw_points(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, y) of the foot's 2 screws in their frame (its +X this frame's +Z): fork_motor_bolts(), off the drive's axis."""
    f = cfg.fork
    return [(z - f.axis_z, y - f.motor_bolt_y) for y, z in fork_motor_bolts(cfg)]


def fork_foot_nut_z(cfg: YawCouplerConfig = DEFAULT) -> float:
    """The foot's nuts' bearing faces along the screws: their slots' outer walls, motor_nut_wall in from the leg's outer face."""
    seat_x, nut_x1, _ = fork_motor_bolt_x(cfg)
    return round(seat_x - nut_x1, 6)


def fork_sleeve_screw_frame(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """The sleeve's 3 radial screws' frame (their nuts' too): on the drive's axis, in their plane - the ring's middle."""
    f = cfg.fork
    return ((round(fork_sleeve_bolt_x(cfg), 6), f.axis_y, f.axis_z), SLEEVE_SCREWS_TURN)


def fork_sleeve_seat_r(cfg: YawCouplerConfig = DEFAULT) -> float:
    """The sleeve's screws' heads' bearing faces' radius: the floors of the counterbores in the motor leg's ring flats
    (ForkParams.sleeve_head_seat deep)."""
    f = cfg.fork
    return f.plate_r - f.sleeve_head_seat


def fork_sleeve_nut_r(cfg: YawCouplerConfig = DEFAULT) -> float:
    """The sleeve's nuts' bearing faces' radius: the floors of their pockets in the sleeve's lugs (ShellParams.sleeve_nut_r
    to the nut's outer face)."""
    return DRIVE.shell.sleeve_nut_r + DRIVE.housing.bolt_nut_thickness


# The base_yaw 120T (gt2_pulley_120t#1, lib/mounts.py) under the coupler: upside down (its +Y down the axis), its hub
# end on the stub's end, turned HubParams.hole_deg about the axis onto the stub's diagonal holes.
YAW_PULLEY_TURN = 180.0                  # about this frame's X: the pulley's +Y down the axis


def yaw_pulley_frame(cfg: YawCouplerConfig = DEFAULT, pulley: PulleyParams = YAW) -> tuple:
    """The 120T's frame in this one: on the axis, its origin (the lower flange's top) the pulley's end_y from the stub's
    end, its hub end there."""
    return ((0.0, round(cfg.hub.stub_y0 + pulley.end_y, 6), 0.0), (YAW_PULLEY_TURN, cfg.hub.hole_deg, 0.0))


def yaw_magnet_deg(cfg: YawCouplerConfig = DEFAULT, deg: float = YAW_MAGNET_DEG) -> float:
    """Where the home magnet on the 120T (at `deg` in the pulley's own frame: lib/pulley/params.py YAW_MAGNET_DEG) stands
    about the base_yaw axis at zero, the pulley in its place: its angle in the BASE's part frame (in its XZ plane from +X
    toward +Z, in (-180, 180]). This frame is the capture's, turned CAPTURE_YAW_DEG about the axis, the base's
    BASE_TURN_DEG (lib/datum.py): an angle here is that much more there."""
    p = (math.cos(math.radians(deg)), 0.0, math.sin(math.radians(deg)))
    m = rotation(yaw_pulley_frame(cfg)[1])
    x, z = (sum(m[i][k] * p[k] for k in range(3)) for i in (0, 2))
    a = math.degrees(math.atan2(z, x)) + BASE_TURN_DEG - CAPTURE_YAW_DEG
    return round(a - 360.0 * math.ceil((a - 180.0) / 360.0), 9) + 0.0


# ---- the shoulder_pitch home sensor (HomeTrayParams, DEFAULT) -------------------------------------------------------------
# The drive's frame at shoulder_pitch zero lies along this one [REFERENCE] (the capture pose: tests/base/test_shoulder_home.py
# checks it through placements.json): its +X this frame's +Z, its +Y this frame's +Y, its +Z this frame's -X; the shell
# turns with the joint, so a point of it at angle theta in the drive's frame (from its +X toward +Y) stands at
# 90 - theta + q from this frame's +Y toward +Z. Angles round the drive's axis below: that one, `alpha`.


def _frame(origin, x_dir, z_dir) -> tuple:
    """A frame as data from its origin and its X and Z directions (unit, perpendicular)."""
    y_dir = (z_dir[1] * x_dir[2] - z_dir[2] * x_dir[1], z_dir[2] * x_dir[0] - z_dir[0] * x_dir[2],
             z_dir[0] * x_dir[1] - z_dir[1] * x_dir[0])
    m = tuple((x_dir[i], y_dir[i], z_dir[i]) for i in range(3))
    return tuple(round(v, 9) + 0.0 for v in origin), angles(m)


def _apply(frame_data: tuple, p) -> tuple[float, float, float]:
    """A point of a frame (as data) in its parent's coordinates."""
    m = rotation(frame_data[1])
    return tuple(frame_data[0][i] + sum(m[i][k] * p[k] for k in range(3)) for i in range(3))


def home_pillar_deg(cfg: YawCouplerConfig = DEFAULT) -> float:
    """The magnet's pillar in the drive's frame: bolt `pillar` from bolt_start_deg (from its +X toward +Y)."""
    h = DRIVE.housing
    return h.bolt_start_deg + cfg.home.pillar * 360.0 / h.bolt_count


def home_alpha(cfg: YawCouplerConfig = DEFAULT) -> float:
    """Where the magnet stands round the drive's axis at home_deg - and the chip: from this frame's +Y toward +Z, in
    [0, 360)."""
    return round((90.0 - home_pillar_deg(cfg) + cfg.home.home_deg) % 360.0, 9) + 0.0


def home_hall_r(cfg: YawCouplerConfig = DEFAULT) -> float:
    """The A3144's hall element off the drive's axis: past the pillar's face (od / 2), the magnet, `air` and the
    element's depth under the chip's branded face."""
    return DRIVE.housing.od / 2.0 + HOME_MAGNET_T + cfg.home.air + A3144_HALL_DEPTH


def home_hall_point(cfg: YawCouplerConfig = DEFAULT) -> tuple[float, float, float]:
    """The hall element in this frame: on the line from the drive's axis through the magnet at home, at the magnet's z."""
    f, a, r = cfg.fork, math.radians(home_alpha(cfg)), home_hall_r(cfg)
    return (fork_x(cfg, cfg.home.magnet_z), f.axis_y + r * math.cos(a), f.axis_z + r * math.sin(a))


def home_sensor_frame(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """The KY-003's part frame in this one: its branded face (+Z) toward the drive's axis, its board (+X) along the turn
    toward the back (-Z) and down, its hall element (ky003_hall_point()) on home_hall_point()."""
    a = math.radians(home_alpha(cfg))
    z_dir = (0.0, -math.cos(a), -math.sin(a))
    x_dir = (0.0, -math.sin(a), math.cos(a))
    hall, hp = home_hall_point(cfg), ky003_hall_point()
    origin = tuple(hall[i] - hp[0] * x_dir[i] - hp[2] * z_dir[i] for i in range(3))
    return _frame(origin, x_dir, z_dir)


def home_to_coupler(cfg: YawCouplerConfig, p) -> tuple[float, float, float]:
    """A point of the KY-003's frame in this one."""
    return _apply(home_sensor_frame(cfg), p)


def home_tray_frame(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """The tray's part frame in this one: z = 0 on the pocket's floor (its underside), +Z up this frame's +Y, this frame's
    (x, z) at its (x, -y) - it prints on its underside."""
    return ((0.0, cfg.yoke.pocket_y0, 0.0), (-90.0, 0.0, 0.0))


def home_tray_box(cfg: YawCouplerConfig = DEFAULT) -> tuple[float, float, float, float]:
    """The tray's footprint in this frame, (x0, x1, z0, z1): the pocket's less wall_clear a side."""
    hx, hz = cfg.yoke.pocket_half
    c = cfg.home.wall_clear
    return (-hx + c, hx - c, -hz + c, hz - c)


def home_board_points(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, y) of the board's 2 M2s in the KY-003's frame: its own holes, KY003_HOLE_INSET in from its sensor end and its
    long edges."""
    y = KY003_BOARD_W / 2.0 - KY003_HOLE_INSET
    return [(KY003_HOLE_INSET, s * y) for s in (1.0, -1.0)]


def home_plate_box(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """The board's plate in the KY-003's frame: ((x0, x1), (y0, y1), (z0, z1)) - under the board, plate_t thick."""
    h = cfg.home
    return (h.plate_x, (-h.plate_w / 2.0, h.plate_w / 2.0), (-h.plate_t, 0.0))


def home_block_points(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """The 8 points in this frame whose convex hull is the block: the plate's outer face over hull_x, its full width, and
    the tray's top over block_z, as wide."""
    h = cfg.home
    w = h.plate_w / 2.0
    top = cfg.yoke.pocket_y0 + h.tray_t
    face = [home_to_coupler(cfg, (x, y, -h.plate_t)) for x in h.hull_x for y in (-w, w)]
    floor = [(x, top, z) for z in h.block_z for x in (-w, w)]
    return face + floor


def home_reliefs(cfg: YawCouplerConfig = DEFAULT) -> list[tuple[tuple, tuple, float]]:
    """The plate's 2 slots in the KY-003's frame, ((x0, x1), (y0, y1), depth under the board): across the header's pins'
    tails (one pitch behind its housing) and the chip's leads' tails (where they turn down), relief_w wide, relief_past
    deeper than each - through the plate where that is past its thickness."""
    h = cfg.home
    pin_x = KY003_BOARD_L - KY003_HEADER_H / 2.0 - KY003_PIN_PITCH
    lead_x = KY003_LEAD_BEND_X + A3144_LEAD_T / 2.0
    rows = ((pin_x, h.header_relief_len, KY003_PIN_TAIL), (lead_x, h.lead_relief_len, KY003_LEAD_TAIL))
    return [((x - h.relief_w / 2.0, x + h.relief_w / 2.0), (-ln / 2.0, ln / 2.0), tail + h.relief_past) for x, ln, tail in rows]


def home_magnet_frame(cfg: YawCouplerConfig = DEFAULT) -> tuple:
    """The magnet's part frame (parts/base/shoulder_home_magnet: its south pole's face on z = 0, the disc in +Z) in
    j1_link's: glued on the pillar's outer face at the drive's z magnet_z, its pole face out (od / 2 + its thickness off
    the drive's axis), its +Z in at the axis - through lib/upper_arm/params.py ArmParams' drive frame (lib/upper_arm/link.py
    drive_frame(): the drive's z drive_z_at_y0 at y = 0, its +X at drive_x_deg, its +Z that frame's -Y)."""
    t = math.radians(home_pillar_deg(cfg))
    r = DRIVE.housing.od / 2.0 + HOME_MAGNET_T
    in_drive = _frame((r * math.cos(t), r * math.sin(t), cfg.home.magnet_z), (0.0, 0.0, 1.0), (-math.cos(t), -math.sin(t), 0.0))
    arm = _UPPER_ARM.arm
    d = math.radians(arm.drive_x_deg)
    drive = _frame((0.0, arm.drive_z_at_y0, 0.0), (math.cos(d), 0.0, math.sin(d)), (0.0, -1.0, 0.0))
    return compose(drive, in_drive)
