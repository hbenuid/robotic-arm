"""Pure-math layout of the upper arm: the hole patterns the builder and the tests share, as (x, z) in j1_link's
part frame (every hole runs along Y); the elbow_pitch home sensor's frames as data (ElbowHomeParams)."""
from __future__ import annotations

import math

from lib.belts import GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, idler_on_run, pulley_circle
from lib.datum import angles, rotation
from lib.pulley.params import DEFAULT as _PULLEY
from lib.sensors import (
    A3144_HALL_DEPTH,
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
from lib.upper_arm.params import DEFAULT, ELBOW_BELT, UpperArmConfig


def hub_bolt_points(cfg: UpperArmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The 4 hub bolt holes: bolt_angle_deg, then every 90 degrees (the angle measured atan2(z, x))."""
    h = cfg.hub
    r = h.bolt_circle_dia / 2.0
    angles = [math.radians(h.bolt_angle_deg + 90.0 * k) for k in range(4)]
    return [(r * math.cos(a), r * math.sin(a)) for a in angles]


def cove_axes(cfg: UpperArmConfig = DEFAULT) -> tuple[tuple[float, float], tuple[float, float]]:
    """The pad's R10 fills in the opening: ((x, y) of the -X cove's axis, along Z - tangent to the opening's -X wall
    and to the underside's level), ((y, |z|) of the Z coves' axes, along X - tangent to the opening's Z walls and
    through the cavity's edge at the underside's level)."""
    p, oh, y0 = cfg.pad, cfg.hub.opening_half, cfg.slab.y0
    rise = math.sqrt(p.cove_r ** 2 - (p.cavity_half_z - (oh - p.cove_r)) ** 2)
    return (-oh + p.cove_r, y0 + p.cove_r), (y0 + rise, oh - p.cove_r)


def pad_holes(cfg: UpperArmConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """The 4 holes of the motor's seat about the pad's axis, (x, z, diameter): LEGACY's pad floor's NEMA 17 square,
    DEFAULT's web's - the elbow gearbox's M4 (lib/upper_arm/params.py)."""
    return list(cfg.pad.holes)


def socket_points(cfg: UpperArmConfig = DEFAULT) -> tuple[list, list]:
    """(shoulder-half sockets, elbow-half sockets) in the underside; both empty without sockets."""
    if cfg.sockets is None:
        return [], []
    return list(cfg.sockets.shoulder), list(cfg.sockets.elbow)


def arm_slide(cfg: UpperArmConfig = DEFAULT) -> float:
    """How far along +Y the plate - DEFAULT's slab, its elbow half's depth end to end - slides with the arm rising off
    the drive's shell (ArmParams): its underside (ElbowParams.y0) onto the underside of the web under the elbow
    gearbox's face (y_outer, less web_t and the screws' heads) - the shell's hub end, the face the part prints on; 0
    without an arm (29.5)."""
    if cfg.arm is None:
        return 0.0
    a = cfg.arm
    return a.y_outer - a.web_t - a.seat_screw.head_h - cfg.elbow.y0


def elbow_belt_loop(cfg: UpperArmConfig = DEFAULT, *, idler: bool = True) -> list[tuple[float, float, float]]:
    """The elbow belt's loop (lib/belts.py) in the belt's own frame: the elbow's 20T (on its gearbox) on the origin (the pad's
    axis), the elbow 90T on +X (the elbow axis, ELBOW_MOTOR_CENTRES on) and - with cfg.idler - the tensioner on the run
    ElbowIdlerParams.run, ElbowIdlerParams.along from the origin, pushed in till the loop is ELBOW_BELT long (the stock
    belt is longer than the open loop). The belt's row (lib/mounts.py elbow_belt#1) puts its +Y on j1_link's +Z."""
    loop = [pulley_circle(0.0, 0.0, GT2_PULLEY_20T_TEETH), pulley_circle(cfg.slab.elbow_x - cfg.pad.x, 0.0, GT2_PULLEY_90T_TEETH)]
    if idler and cfg.idler is not None:
        loop.insert(cfg.idler.run + 1, idler_on_run(loop, cfg.idler.run, cfg.idler.along, ELBOW_BELT))
    return loop


def elbow_idler_x(cfg: UpperArmConfig = DEFAULT) -> float:
    """The elbow idler's axis along the link (j1_link's x): ElbowIdlerParams.along from the motor's axis (the pad's)."""
    return round(cfg.pad.x + cfg.idler.along, 6)


def elbow_idler_z(cfg: UpperArmConfig = DEFAULT) -> float:
    """The elbow idler's axis across the link (j1_link's z) with the stock belt: elbow_belt_loop()'s idler, its belt-frame
    y; its x is elbow_idler_x()."""
    return elbow_belt_loop(cfg)[cfg.idler.run + 1][1]


def slab_taper_x(cfg: UpperArmConfig = DEFAULT) -> float:
    """Where the arm's slab (ArmParams) starts tapering onto a faceted end narrower than it, along the link: the elbow
    idler's nut slot's far side (its flats + nut_fit across X about elbow_idler_x()) - the slab keeps its width over
    the idler's slots (125.2)."""
    i = cfg.idler
    return round(elbow_idler_x(cfg) + (i.nut.af + i.nut_fit) / 2.0, 6)


# --- the elbow_pitch home sensor (ElbowHomeParams), in j1_link's part frame unless named ---------------------------------
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


def home_end_x(cfg: UpperArmConfig = DEFAULT) -> float:
    """The slab's end flat along the link (its faceted end's flat on +x, ArmParams.end_r past the elbow axis): the
    face the clip's spine runs down (206.0)."""
    return cfg.slab.elbow_x + cfg.arm.end_r


def home_slab_y(cfg: UpperArmConfig = DEFAULT) -> tuple[float, float]:
    """(underside, top) of the slab at the elbow end, along N: the faces the clip's jaw and top arm bear on (5.5, 23.5)."""
    s = arm_slide(cfg)
    return cfg.elbow.y0 + s, cfg.elbow.relief_y + s


def home_chip_r(cfg: UpperArmConfig = DEFAULT) -> float:
    """The hall element's - and the magnet's - distance from the elbow axis (23.77)."""
    return math.hypot(cfg.home.chip_x, cfg.home.chip_z)


def home_chip_deg(cfg: UpperArmConfig = DEFAULT) -> float:
    """The hall element round the elbow axis: atan2(z, x) in this frame, from the end flat's normal (-22.25)."""
    return math.degrees(math.atan2(cfg.home.chip_z, cfg.home.chip_x))


def home_magnet_face_y(cfg: UpperArmConfig = DEFAULT) -> float:
    """The magnet's pole face along N: on the 90T's outer face (its own face_y, j1_link's -Y from pulley_y), magnet_t
    out (-8.9)."""
    h = cfg.home
    return h.pulley_y - (_PULLEY.face_y + h.magnet_t)


def home_hall_point(cfg: UpperArmConfig = DEFAULT) -> tuple[float, float, float]:
    """The hall element: chip_x / chip_z off the elbow axis, `air` and its depth under the magnet's pole face."""
    h = cfg.home
    return (cfg.slab.elbow_x + h.chip_x, home_magnet_face_y(cfg) - h.air - A3144_HALL_DEPTH, h.chip_z)


def home_sensor_frame(cfg: UpperArmConfig = DEFAULT) -> tuple:
    """The KY-003's part frame in this one: its branded face (+Z) up N at the 90T's face, its board (+X) along +z, its
    hall element (ky003_hall_point()) on home_hall_point()."""
    x_dir, z_dir = (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)
    hall, hp = home_hall_point(cfg), ky003_hall_point()
    origin = tuple(hall[i] - hp[0] * x_dir[i] - hp[2] * z_dir[i] for i in range(3))
    return _frame(origin, x_dir, z_dir)


def home_to_link(cfg: UpperArmConfig, p) -> tuple[float, float, float]:
    """A point of the KY-003's frame in this one."""
    return _apply(home_sensor_frame(cfg), p)


def home_board_points(cfg: UpperArmConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, y) of the board's 2 M2s in the KY-003's frame: its own holes, KY003_HOLE_INSET in from its sensor end and its
    long edges."""
    y = KY003_BOARD_W / 2.0 - KY003_HOLE_INSET
    return [(KY003_HOLE_INSET, s * y) for s in (1.0, -1.0)]


def home_plate_box(cfg: UpperArmConfig = DEFAULT) -> tuple:
    """The board's plate in the KY-003's frame: ((x0, x1), (y0, y1), (z0, z1)) - under the board, plate_t thick."""
    h = cfg.home
    return (h.plate_x, (-h.plate_w / 2.0, h.plate_w / 2.0), (-h.plate_t, 0.0))


def home_reliefs(cfg: UpperArmConfig = DEFAULT) -> list[tuple[tuple, tuple, float]]:
    """The plate's 2 slots in the KY-003's frame, ((x0, x1), (y0, y1), depth under the board): across the header's pins'
    tails (one pitch behind its housing) and the chip's leads' tails (where they turn down), relief_w wide, relief_past
    deeper than each - through the plate where that is past its thickness."""
    h = cfg.home
    pin_x = KY003_BOARD_L - KY003_HEADER_H / 2.0 - KY003_PIN_PITCH
    lead_x = KY003_LEAD_BEND_X + A3144_LEAD_T / 2.0
    rows = ((pin_x, h.header_relief_len, KY003_PIN_TAIL), (lead_x, h.lead_relief_len, KY003_LEAD_TAIL))
    return [((x - h.relief_w / 2.0, x + h.relief_w / 2.0), (-ln / 2.0, ln / 2.0), tail + h.relief_past) for x, ln, tail in rows]


def home_clip_z(cfg: UpperArmConfig = DEFAULT) -> tuple[float, float]:
    """The clip's span across the link: from the plate's near end (the face it prints on), `width` on (-7.99, 8.01)."""
    z0 = home_to_link(cfg, (cfg.home.plate_x[0], 0.0, 0.0))[2]
    return z0, z0 + cfg.home.width


def home_clip_frame(cfg: UpperArmConfig = DEFAULT) -> tuple:
    """The clip's part frame in this one: this frame moved to the end flat (x), the slab's underside (y) and the clip's
    near side (z) - its +Z up as it prints on its -Z face."""
    return ((home_end_x(cfg), home_slab_y(cfg)[0], round(home_clip_z(cfg)[0], 9)), (0.0, 0.0, 0.0))


def home_magnet_deg(cfg: UpperArmConfig = DEFAULT) -> float:
    """The magnet round the elbow axis in the 90T's own frame (atan2(z, x) there): over the chip at home_deg - in this
    frame the joint turns a point by -q about +Y (atan2(z, x) - q), the 90T's own angles run the other way round from
    pulley_x_deg (17.66)."""
    return cfg.home.pulley_x_deg - (home_chip_deg(cfg) + cfg.home.home_deg)


def home_magnet_frame(cfg: UpperArmConfig = DEFAULT) -> tuple:
    """The magnet's part frame (parts/joints/elbow_home_magnet: its south pole's face on z = 0, the disc in +Z) in the
    elbow 90T's own (its outer face at face_y, +Y outward): glued on that face, home_chip_r() off the axis at
    home_magnet_deg(), its +Z in at the face."""
    a, r = math.radians(home_magnet_deg(cfg)), home_chip_r(cfg)
    return ((round(r * math.cos(a), 9) + 0.0, _PULLEY.face_y + cfg.home.magnet_t, round(r * math.sin(a), 9) + 0.0),
            (90.0, 0.0, 0.0))
