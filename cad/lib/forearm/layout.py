"""Pure-math layout of the forearm: hole patterns and the pieces every builder and test shares."""
from __future__ import annotations

import math

from lib.belts import GT2_IDLER_WIDTH, GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, idler_on_run, pulley_circle
from lib.forearm.params import DEFAULT, PULLEY_90T, ForearmConfig
from lib.motors import MKS_SERVO42D_HEADERS
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
from lib.units import NUDGE


def disc_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The elbow disc's 4x M4, on the axes (LEGACY)."""
    r = cfg.disc.bolt_r
    return [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)]


def disc_bolt_angles(cfg: ForearmConfig = DEFAULT) -> list[float]:
    """The hex-pocket key angle of each bolt (radians): a vertex points at the disc centre."""
    return [math.atan2(y, x) for x, y in disc_bolt_points(cfg)]


def elbow_end_x(cfg: ForearmConfig = DEFAULT) -> float:
    """Where the web ends toward the elbow: the wall's wrist face with the roll joint, else the elbow pivot (the
    disc's round end continues to +45)."""
    return cfg.roll_end.wall_x[0] if cfg.roll else 0.0


def _grid(cfg: ForearmConfig, x_max: float) -> list[tuple[float, float]]:
    """The socket grid columns that lie at x < x_max (a part that ends before a column has no socket there)."""
    s = cfg.sockets
    return [(x, y) for x in s.grid_x if x + s.dia / 2.0 < x_max for y in (s.grid_y, -s.grid_y)]


def link_socket_points(cfg: ForearmConfig = DEFAULT) -> tuple[list, list]:
    """(top-face sockets, bottom-face sockets) of the web; both empty without sockets."""
    if cfg.sockets is None:
        return [], []
    grid = _grid(cfg, elbow_end_x(cfg))
    return grid, grid + list(cfg.sockets.wrist)


def flange_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The wall's holes as (y, z) about the roll axis (y 0, z axis_z): the rotor clamp's 4 screws (clamp_points, the
    module's x-y) as j2_link sees them - host y = module y, host z = module x + axis_z."""
    return [(y, x + cfg.roll_end.axis_z) for x, y in clamp_points(cfg)]


def motor_hole_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The wrist motor's 4 M3 holes through the web (x, y) - the NEMA 17 square about its axis at motor_x
    (MotorHoleParams), the screws' (parts/joints/wrist_motor_screws) - or none on the slide (LEGACY)."""
    if cfg.holes is None:
        return []
    h = cfg.holes.bolt_half
    return [(cfg.motor_x + sx, sy) for sx in (-h, h) for sy in (-h, h)]


def screws_under_the_web(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The wall's screws (y, z) whose heads, channel_clear all round, would land in the web behind the wall: each
    gets a channel in the web's underside (screw_channel)."""
    w = cfg.web
    reach = cfg.drive.clamp_screw.head_dia / 2.0 + cfg.roll_end.channel_clear
    return [(y, z) for y, z in flange_bolt_points(cfg) if z - reach < w.z1 and z + reach > w.z0]


def screw_channel(z: float, cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(x0, x1, half_width, z_top) of the channel for the screw at height z: from the motor's pilot opening - the central
    slot's elbow-end arc centre, or the round hole's centre - (the channel runs into it) to the wall, the head +
    channel_clear either side, open to the web's underside up to the head's top + channel_clear."""
    reach = cfg.drive.clamp_screw.head_dia / 2.0 + cfg.roll_end.channel_clear
    x0 = cfg.slot.centre_x[1] if cfg.slot is not None else cfg.motor_x
    return x0, elbow_end_x(cfg), reach, z + reach


def clamp_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The rotor clamp's 4x M4 in the module x-y plane - through the forearm wall, the pulley and the shaft: on
    clamp_circle_dia, from clamp_angle_deg, 90 deg apart."""
    d = cfg.drive
    r = d.clamp_circle_dia / 2.0
    return [(r * math.cos(math.radians(d.clamp_angle_deg + 90.0 * k)), r * math.sin(math.radians(d.clamp_angle_deg + 90.0 * k)))
            for k in range(4)]


def clamp_nut_pocket(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float]:
    """(af, z0, z1) of each clamp screw's nut pocket in the shaft's flange: a closed hex on the screw's axis, clamp_nut_af
    across flats, a flat toward the bore (lib/geom.py nut_pocket); from the shaft's rear face (open: the nut goes in
    there) to the nut's bearing face."""
    d, S = cfg.drive, stack_positions(cfg)
    return d.clamp_nut_af, S["z_shaft_end"], S["z_clamp_nut_seat"]


def wrist_belt_loop(cfg: ForearmConfig = DEFAULT, *, idler: bool = True) -> list[tuple[float, float, float]]:
    """The wrist belt's loop (lib/belts.py) in the belt's own frame: the motor's 20T on the origin, the wrist 90T on +X
    (the belt frame sits on the motor's axis at wrist_belt_z, turned 180 deg about Z from j2_link's: wrist_belt_frame());
    with cfg.idler, the tensioner on the run leaving the 20T (local -Y), IdlerParams.along from the motor, pushed in till
    the loop is RollEndParams.wrist_belt long (the stock belt is longer than the open loop)."""
    loop = [pulley_circle(0.0, 0.0, GT2_PULLEY_20T_TEETH), pulley_circle(cfg.motor_x - cfg.web.wrist_x, 0.0, GT2_PULLEY_90T_TEETH)]
    if idler and cfg.idler is not None:
        loop.insert(1, idler_on_run(loop, 0, cfg.idler.along, cfg.roll_end.wrist_belt))
    return loop


def roll_belt_loop(cfg: ForearmConfig = DEFAULT, *, idler: bool = True) -> list[tuple[float, float, float]]:
    """The roll belt's loop (lib/belts.py) in the belt's own frame: the roll motor's 20T on the origin (the elbow axis),
    the pulley's 90T ring on +X (the roll axis, centre_distance on) and - with idler - the tensioner on the run back from
    the ring, RollIdlerParams.along from the motor, pushed in till the loop is roll_belt long (the stock belt is longer
    than the open loop). The belt's row in assemblies/forearm_roll_drive.py turns +X onto the module's +Y."""
    d = cfg.drive
    loop = [pulley_circle(0.0, 0.0, GT2_PULLEY_20T_TEETH), pulley_circle(d.centre_distance, 0.0, d.ring_teeth)]
    if idler:
        loop.append(idler_on_run(loop, 1, d.idler.along, d.roll_belt))
    return loop


def roll_idler_xy(cfg: ForearmConfig = DEFAULT) -> tuple[float, float]:
    """The roll belt's idler axis in the module frame (x, y): roll_belt_loop()'s idler through the belt's row (its +X on
    the module's +Y, its +Y on the module's -X, from the motor's axis)."""
    S = stack_positions(cfg)
    bx, by, _ = roll_belt_loop(cfg)[2]
    return (S["x_motor"] - by, S["y_motor"] + bx)


def wrist_belt_frame(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The wrist belt's frame as data in j2_link's frame: on the motor's axis at the belt's mid-plane, its +X toward the
    wrist (j2_link's -X), +Z = j2_link's +Z."""
    return ((cfg.motor_x, 0.0, cfg.roll_end.wrist_belt_z), (0.0, 0.0, 180.0))


def wrist_idler_xy(cfg: ForearmConfig = DEFAULT) -> tuple[float, float]:
    """The wrist idler's axis in j2_link's frame (x, y): wrist_belt_loop()'s idler through wrist_belt_frame()."""
    x, y, _ = wrist_belt_loop(cfg)[1]
    return (cfg.motor_x - x, -y)


def roll_idler_spacer_h(cfg: ForearmConfig = DEFAULT) -> float:
    """The roll idler's printed spacer's height (RollIdlerParams): from the frame's front face up to the idler's
    underside, level with the belt."""
    S = stack_positions(cfg)
    return round(S["z_idler"] - S["z_front"], 6)


def roll_idler_setback(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """The roll idler nut's way in (RollIdlerParams.setback_h), (y_face, y_back, z_low, z_top): the motor pocket's far
    face (y_face, motor_pocket's y1) set back to y_back - the nut channel's near side - from the plate's inner face
    (z_top, the motor's face) down to z_low, then 45 deg back to y_face, (y_back - y_face) further down."""
    i = cfg.drive.idler
    _y0, y1, _z0, z1 = motor_pocket(cfg)
    _x, iy = roll_idler_xy(cfg)
    return (y1, iy - (i.nut.af / 2.0 + i.nut_fit), z1 - i.setback_h, z1)


def wrist_idler_spacer_h(cfg: ForearmConfig = DEFAULT) -> float:
    """The wrist idler's printed spacer's height (IdlerParams): from the idler's top face, level with the belt, up to the
    web's underside."""
    return round(cfg.web.z0 - (cfg.roll_end.wrist_belt_z + GT2_IDLER_WIDTH / 2.0), 6)


def lap_z(cfg: ForearmConfig = DEFAULT) -> float:
    """The split's lap plane (WallSplitParams): the web's mid-plane - the tongue above it, the web's end below it."""
    return round((cfg.web.z0 + cfg.web.z1) / 2.0, 6)


def lap_screw_head_z(cfg: ForearmConfig = DEFAULT) -> float:
    """The lap screws' heads' bearing face (WallSplitParams): the counterbores' floor, head_h up from the web's underside."""
    return round(cfg.web.z0 + cfg.split.screw.head_h, 6)


def lap_nut_z(cfg: ForearmConfig = DEFAULT) -> float:
    """The lap nuts' bearing face (WallSplitParams): their pockets' floor, the nut's height under the web's top face (the
    nut flush with it)."""
    return round(cfg.web.z1 - cfg.split.nut.h, 6)


def module_frame_in_host(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The roll drive's module frame as data in j2_link's frame before the forearm's turn (lib/mounts.py ModuleMount,
    roll_turn_deg - the elbow side's view of the forearm: the drive keeps its pose, j2_link turns about the roll axis):
    origin on the roll axis at the elbow axis' station (the elbow axis runs along module X, elbow_offset below it),
    module +Z = host -X (toward the wrist), module +X = host +Z."""
    return ((0.0, 0.0, cfg.roll_end.axis_z), (0.0, -90.0, 0.0))


def stack_positions(cfg: ForearmConfig = DEFAULT) -> dict[str, float]:
    """Module-frame stations of the roll drive (RollDriveParams; the frame, the shaft and the pulley builders emit their
    geometry at these stations, their rows at 0). From the forearm wall back: the rotor clamp's heads in the wall's wrist
    face, the ring's front face on the wall, the ring, the pulley's Ø33 shoulder on down to bearing 2 (the ring's rear
    flange over the frame's front face); the motor's face where its 20T, hub out, meets the ring
    (pulley_gap), the plate in front of it - its face the block's front face, the motor screws' heads flush in it, and
    nothing proud of it: the frame prints flat on it -; bearing 2 right under that face (its seat open through it),
    the lip (the spacer between the outer rings), bearing 1 - the pulley's neck meets the shaft at its front face -, the
    shaft behind it - its rear face at the rotor clamp's tips -, the bay bored on through the back past the tower's rear
    (tower_rear, the outline's). The elbow axis runs along X at y = y_elbow, under the tower at z = 0; the motor sits on
    it. Every number a row or a test needs comes from here."""
    d, w = cfg.drive, cfg.roll_end
    z_wall = -w.wall_x[1]                                         # 44.75: the forearm wall's elbow face = the ring's front face
    z_ring = z_wall - d.ring_flange_t - d.ring_width              # 36.55: the teeth
    z_ring_flange_1 = z_ring - d.ring_flange_t                    # 35.35: the ring's rear flange, 1.5 over the frame's front face
    z_ring_mid = z_ring + d.ring_width / 2.0                      # 40.05
    z_motor_face = z_ring_mid - d.t20                             # 26.85: the 20T's teeth level with the ring's, its far end pulley_gap over the face
    z_front = z_motor_face + d.pad_t                              # 33.85: the plate's front face = the block's front face
    z_boss = z_front                                              # 33.85: the bearing boss's top = bearing 2's front face, flush with it
    z_bearing_2 = z_boss - d.bearing_width                        # 26.85
    z_lip = z_bearing_2 - d.lip                                   # 14.85: the spacer's rear = bearing 1's front face
    z_bearing_1 = z_lip - d.bearing_width                         # 7.85: the seat's rear, into the bay
    z_wall_back = -w.wall_x[0]                                    # 56: the wall's wrist face
    z_clamp_head = z_wall_back - d.clamp_screw.head_h - d.clamp_head_clear   # 51.75: the clamp's counterbores' floor, in the wall
    z_shaft_end = z_clamp_head - d.clamp_screw_len                # 1.75: the clamp's tips = the shaft's rear face
    z_clamp_nut = z_shaft_end + 2 * d.clamp_pitch                 # 3.15: each clamp nut's far face
    z_meet = z_lip                                                # 14.85: the pulley's neck meets the shaft's journal at bearing 1's front face
    z_idler = z_ring_mid - GT2_IDLER_WIDTH / 2.0                  # 35.55: the idler's underside (its channel on the belt) = the spacer's land
    return {
        "z_block": 0.0, "z_shaft": 0.0, "z_pulley": 0.0,
        "z_tower_rear": d.tower_rear,                             # 0.75: the tower's rear in the profile (the rear line's anchor)
        "z_shaft_end": z_shaft_end,
        "z_clamp_nut": z_clamp_nut,
        "z_clamp_nut_seat": z_clamp_nut + d.clamp_nut.h,          # 6.35: its bearing face, toward the pulley
        "z_clamp_head": z_clamp_head,
        "z_stop_lug": z_shaft_end,                                # 1.8 .. +stop_lug_t on the shaft's flange
        "z_stop_post": z_bearing_1 - d.stop_post_t,               # 3.0 .. 7.85 on the tower's face behind bearing 1
        "z_shoulder_1": z_bearing_1 - d.run_gap,                  # 6.85: the shaft's Ø33 shoulder on bearing 1's inner ring
        "z_bearing_1": z_bearing_1, "z_lip": z_lip,
        "z_meet": z_meet,
        "z_bearing_2": z_bearing_2, "z_boss": z_boss,
        "z_ring_flange_1": z_ring_flange_1, "z_ring": z_ring, "z_ring_mid": z_ring_mid,
        "z_ring_end": z_wall,                                     # 44.75: the ring's front flange face
        "z_front": z_front,
        "z_wall": z_wall,                                         # 44.75: ... = the pulley's flat front face = the wall's elbow face
        "z_wall_back": z_wall_back,
        "z_motor_face": z_motor_face,
        "z_motor_board": z_motor_face - d.motor.body_length,      # -12.65: the board's frame (the motor's rear face)
        "z_motor_screws": z_front - d.motor_holes.screw.head_h,   # 30.85: the motor screws' counterbores' floor = their heads' bearing face
        "z_20t_end": z_front + d.pulley_gap,                      # 36.55: the 20T's far end (hub out: its teeth's end), toward the motor
        "z_20t": z_front + d.pulley_gap + d.t20_len,              # 51.0: the 20T's hub face, toward the wrist
        "z_idler": z_idler, "z_idler_top": z_idler + GT2_IDLER_WIDTH,   # 44.55: its top, the screw head's bearing face
        "z_idler_roof": z_front - d.idler.roof,                   # 30.85: the nut's channel's roof = the nut's bearing face
        "y_elbow": d.elbow_y,                                     # -74: the elbow axis (along X)
        "x_motor": 0.0, "y_motor": d.motor_y,                     # on the elbow axis
        "x_cradle": d.block_x[0] + d.web_t,                       # -23: the pocket's floor
        "x_side": d.block_x[1] - d.side_t,                        # 23: the side wall's inside, the pocket's other side
    }


def motor_pocket(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(y0, y1, z0, z1) of the motor's pocket in the frame (between the floor, x_cradle, and the side wall, x_side): the
    motor and its board (the wider: the board's cover) + pocket_clear - y0 its edge at the motor; the frame cuts it on
    down out through the octagon's bottom (the motor slides up into it) -; from pocket_rear behind the board's cover to
    the plate (the motor's face)."""
    d, S = cfg.drive, stack_positions(cfg)
    half = max(d.motor.body_width, d.board_w) / 2.0 + d.pocket_clear
    return (S["y_motor"] - half, S["y_motor"] + half, S["z_motor_board"] - d.board_stack - d.pocket_rear, S["z_motor_face"])


def side_headers(cfg: ForearmConfig = DEFAULT) -> dict[str, tuple[tuple[float, ...], int]]:
    """The MKS board's two side headers in the module frame, the board turned motor_spin_deg with the motor (lib/motors.py
    MKS_SERVO42D_HEADERS): name -> ((x0, x1, y0, y1, z0, z1), the way its plug goes in along X: +1 / -1)."""
    d, S = cfg.drive, stack_positions(cfg)
    a = math.radians(d.motor_spin_deg)
    c, s = round(math.cos(a)), round(math.sin(a))      # the board's quarter turn about the motor axis
    out = {}
    for name in ("side_neg_x", "side_pos_x"):
        (x0, y0, z0, x1, y1, z1), (ux, uy) = MKS_SERVO42D_HEADERS[name]
        xs = [S["x_motor"] + c * x - s * y for x in (x0, x1) for y in (y0, y1)]
        ys = [S["y_motor"] + s * x + c * y for x in (x0, x1) for y in (y0, y1)]
        way = c * ux - s * uy                            # the plug's way in, turned: along X (+1 / -1)
        assert s * ux + c * uy == 0 and abs(way) == 1, "the side plugs face along X, at the walls"
        out[name] = ((min(xs), max(xs), min(ys), max(ys), S["z_motor_board"] + z0, S["z_motor_board"] + z1), way)
    return out


def plug_slots(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float, float, float, float, float]]:
    """(x0, x1, y0, y1, z0, z1) of the two plug grooves, one inside each wall (the side wall, the floor) at the MKS
    board's side headers (side_headers): from the pocket out to plug_skin inside the frame's face, from the header's top
    (plug_clear over it) down out through the octagon's bottom, plug_clear round the header's z - and motor_drop more
    behind it, for the slide: the motor + board slide up with the cables plugged, each cable bent straight down at its
    plug into its groove."""
    d, S = cfg.drive, stack_positions(cfg)
    grooves = []
    for (_x0, _x1, _y0, y1, z0, z1), way in side_headers(cfg).values():
        x = (S["x_side"] - NUDGE, d.block_x[1] - d.plug_skin) if way > 0 else (d.block_x[0] + d.plug_skin, S["x_cradle"] + NUDGE)
        bottom = S["y_elbow"] - d.end_r - 1.0           # past the octagon's bottom flat
        grooves.append((*x, bottom, y1 + d.plug_clear, z0 - d.plug_clear - d.motor_drop, z1 + d.plug_clear))
    return grooves


def side_plug_room(cfg: ForearmConfig = DEFAULT) -> dict[str, float]:
    """The straight room each side plug has out from its header's face to its groove's skin - less than MKS_PLUG_ROOM
    straight out: each cable bends straight down at its plug, along its groove."""
    d = cfg.drive
    room = {}
    for name, ((x0, x1, *_), way) in side_headers(cfg).items():
        room[name] = (d.block_x[1] - d.plug_skin - x1) if way > 0 else (x0 - (d.block_x[0] + d.plug_skin))
    return room


def pad_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The frame plate's 4 M3 holes in the module x-y plane (the motor's 31 mm square, axis-aligned): the motor screws'
    (parts/joints/roll_motor_screws), their heads in counterbores in the front face (RollDriveParams.motor_holes)."""
    from lib.cycloidal.layout import motor_bolt_points
    S = stack_positions(cfg)
    return [(S["x_motor"] + x, S["y_motor"] + y) for x, y in motor_bolt_points()]


def pulley_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The elbow 90T pulley's 4x M4 into the web's underside, as (y, z) ABOUT THE ELBOW AXIS (add y_elbow for the
    module frame): the SolidWorks coupler's pattern (pulley_bolt_r out, on the axes) turned pulley_bolt_deg."""
    d = cfg.drive
    return [(d.pulley_bolt_r * math.cos(a), d.pulley_bolt_r * math.sin(a))
            for a in (math.radians(d.pulley_bolt_deg + 90.0 * k) for k in range(4))]


def nut_channel_end(cfg: ForearmConfig = DEFAULT) -> float:
    """Module x where each pulley bolt's hex channel ends: up through the floor, nut_channel_past into the motor's
    pocket (the nuts go in from there before the motor)."""
    return stack_positions(cfg)["x_cradle"] + cfg.drive.nut_channel_past


def coupler_steps(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """The elbow flange under the frame's underside as (diameter, x0, x1) cylinders about the elbow axis, the face
    downward: the post over j1_link's bore (its foot chamfered post_chamfer: roll.py build_elbow_flange), the shoulder on the
    upper elbow bearing's inner ring, the stub in it."""
    d = cfg.drive
    return [(d.post_dia, *d.post_x), (d.step_dia, *d.step_x), (d.stub_dia, *d.stub_x)]


# --- the wrist_pitch home sensor (WristHomeParams), in j2_link's part frame unless named --------------------------------
def _frame(origin, x_dir, z_dir) -> tuple:
    """A frame as data from its origin and its X and Z directions (unit, perpendicular). lib.datum is imported here, not
    at the top: it imports lib.base.params, which imports this package's params (a cycle at import)."""
    from lib.datum import angles

    y_dir = (z_dir[1] * x_dir[2] - z_dir[2] * x_dir[1], z_dir[2] * x_dir[0] - z_dir[0] * x_dir[2],
             z_dir[0] * x_dir[1] - z_dir[1] * x_dir[0])
    m = tuple((x_dir[i], y_dir[i], z_dir[i]) for i in range(3))
    return tuple(round(v, 9) + 0.0 for v in origin), angles(m)


def _apply(frame_data: tuple, p) -> tuple[float, float, float]:
    """A point of a frame (as data) in its parent's coordinates."""
    from lib.datum import rotation

    m = rotation(frame_data[1])
    return tuple(frame_data[0][i] + sum(m[i][k] * p[k] for k in range(3)) for i in range(3))


def home_end_x(cfg: ForearmConfig = DEFAULT) -> float:
    """The wrist boss's end flat along x (its faceted end's flat on -x, half its width past the wrist pivot): the face the
    clip's cap and spine run down (-199.094)."""
    return cfg.web.wrist_x - cfg.boss.dia / 2.0


def home_boss_z(cfg: ForearmConfig = DEFAULT) -> tuple[float, float]:
    """(underside, top) of the wrist boss along N: the faces the jaw bears on and the cap stops under (-5.0, 13.0)."""
    return cfg.web.z0, cfg.boss.z1


def home_chip_r(cfg: ForearmConfig = DEFAULT) -> float:
    """The hall element's - and the magnet's - distance from the wrist axis (23.77)."""
    return math.hypot(cfg.home.chip_x, cfg.home.chip_y)


def home_chip_deg(cfg: ForearmConfig = DEFAULT) -> float:
    """The hall element round the wrist axis: atan2(y, x) about the wrist pivot (-157.75)."""
    return math.degrees(math.atan2(cfg.home.chip_y, cfg.home.chip_x))


def home_pulley_z(cfg: ForearmConfig = DEFAULT) -> float:
    """The wrist 90T's own origin (its lower flange's top) along N: the belt's plane (roll_end.wrist_belt_z) plus half its
    tooth band - its +Y runs down N (-9.2)."""
    return cfg.roll_end.wrist_belt_z + PULLEY_90T.band_y1 / 2.0


def home_magnet_face_z(cfg: ForearmConfig = DEFAULT) -> float:
    """The magnet's pole face along N: on the 90T's outer face (its own face_y down N from home_pulley_z()), magnet_t
    out (-19.4)."""
    return home_pulley_z(cfg) - (PULLEY_90T.face_y + cfg.home.magnet_t)


def home_hall_point(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float]:
    """The hall element: chip_x / chip_y off the wrist pivot, `air` and its depth under the magnet's pole face."""
    h = cfg.home
    return (cfg.web.wrist_x + h.chip_x, h.chip_y, home_magnet_face_z(cfg) - h.air - A3144_HALL_DEPTH)


def home_sensor_frame(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The KY-003's part frame in this one: its branded face (+Z) up N at the 90T's face, its board (+X) along +y, its
    hall element (ky003_hall_point()) on home_hall_point()."""
    x_dir, z_dir = (0.0, 1.0, 0.0), (0.0, 0.0, 1.0)
    hall, hp = home_hall_point(cfg), ky003_hall_point()
    origin = tuple(hall[i] - hp[0] * x_dir[i] - hp[2] * z_dir[i] for i in range(3))
    return _frame(origin, x_dir, z_dir)


def home_to_link(cfg: ForearmConfig, p) -> tuple[float, float, float]:
    """A point of the KY-003's frame in this one."""
    return _apply(home_sensor_frame(cfg), p)


def home_board_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """(x, y) of the board's 2 M2s in the KY-003's frame: its own holes, KY003_HOLE_INSET in from its sensor end and its
    long edges."""
    y = KY003_BOARD_W / 2.0 - KY003_HOLE_INSET
    return [(KY003_HOLE_INSET, s * y) for s in (1.0, -1.0)]


def home_plate_box(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The board's plate in the KY-003's frame: ((x0, x1), (y0, y1), (z0, z1)) - under the board, plate_t thick."""
    h = cfg.home
    return (h.plate_x, (-h.plate_w / 2.0, h.plate_w / 2.0), (-h.plate_t, 0.0))


def home_reliefs(cfg: ForearmConfig = DEFAULT) -> list[tuple[tuple, tuple, float]]:
    """The plate's 2 slots in the KY-003's frame, ((x0, x1), (y0, y1), depth under the board): across the header's pins'
    tails (one pitch behind its housing) and the chip's leads' tails (where they turn down), relief_w wide, relief_past
    deeper than each - through the plate where that is past its thickness."""
    h = cfg.home
    pin_x = KY003_BOARD_L - KY003_HEADER_H / 2.0 - KY003_PIN_PITCH
    lead_x = KY003_LEAD_BEND_X + A3144_LEAD_T / 2.0
    rows = ((pin_x, h.header_relief_len, KY003_PIN_TAIL), (lead_x, h.lead_relief_len, KY003_LEAD_TAIL))
    return [((x - h.relief_w / 2.0, x + h.relief_w / 2.0), (-ln / 2.0, ln / 2.0), tail + h.relief_past) for x, ln, tail in rows]


def home_clip_frame(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The clip's part frame in this one: on its outer face over the end flat (gap and wall past it) at the boss's
    underside, its +Z along +x (up as it prints, on that face), its +X along +y, its +Y up N."""
    h = cfg.home
    return _frame((home_end_x(cfg) - h.gap - h.wall, 0.0, home_boss_z(cfg)[0]), (0.0, 1.0, 0.0), (1.0, 0.0, 0.0))


def home_magnet_deg(cfg: ForearmConfig = DEFAULT) -> float:
    """The magnet round the wrist axis in the 90T's own frame (atan2(z, x) there): over the chip at home_deg - in this
    frame the joint turns a point by +q about +Z (atan2(y, x) + q), the 90T's own angles run on from pulley_x_deg the same
    way round (104.11)."""
    return home_chip_deg(cfg) - cfg.home.home_deg - cfg.home.pulley_x_deg


def home_magnet_frame(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The magnet's part frame (parts/joints/wrist_home_magnet: its south pole's face on z = 0, the disc in +Z) in the
    wrist 90T's own (its outer face at face_y, +Y outward): glued on that face, home_chip_r() off the axis at
    home_magnet_deg(), its +Z in at the face."""
    a, r = math.radians(home_magnet_deg(cfg)), home_chip_r(cfg)
    return ((round(r * math.cos(a), 9) + 0.0, PULLEY_90T.face_y + cfg.home.magnet_t, round(r * math.sin(a), 9) + 0.0),
            (90.0, 0.0, 0.0))
