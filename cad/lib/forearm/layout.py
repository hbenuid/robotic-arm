"""Pure-math layout of the forearm: hole patterns and the pieces every builder and test shares."""
from __future__ import annotations

import math

from lib.belts import GT2_IDLER_WIDTH, GT2_PULLEY_20T_TEETH, GT2_PULLEY_90T_TEETH, idler_on_run, pulley_circle
from lib.fasteners import M3_PITCH
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.motors import MKS_PLUG_ROOM, MKS_SERVO42D_HEADERS
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
    """The rotor flange's bolts as (y, z) about the roll axis (y 0, z axis_z), bolt_count on the bolt circle from
    bolt_angle_deg (the wall's holes; the flange's own pattern is the same in the module frame)."""
    r = cfg.roll_end
    rad = r.bolt_circle_dia / 2.0
    return [(rad * math.cos(math.radians(r.bolt_angle_deg + i * 360.0 / r.bolt_count)),
             r.axis_z + rad * math.sin(math.radians(r.bolt_angle_deg + i * 360.0 / r.bolt_count)))
            for i in range(r.bolt_count)]


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
    r, w = cfg.roll_end, cfg.web
    reach = r.screw.head_dia / 2.0 + r.channel_clear
    return [(y, z) for y, z in flange_bolt_points(cfg) if z - reach < w.z1 and z + reach > w.z0]


def screw_channel(z: float, cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(x0, x1, half_width, z_top) of the channel for the screw at height z: from the motor's pilot opening - the central
    slot's elbow-end arc centre, or the round hole's centre - (the channel runs into it) to the wall, the head +
    channel_clear either side, open to the web's underside up to the head's top + channel_clear."""
    r = cfg.roll_end
    reach = r.screw.head_dia / 2.0 + r.channel_clear
    x0 = cfg.slot.centre_x[1] if cfg.slot is not None else cfg.motor_x
    return x0, elbow_end_x(cfg), reach, z + reach


def end_nut_pocket(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(r_in, r_out, z0, z1) of each wall screw's nut pocket in the pulley's core, module frame, radially along its
    screw's angle: from inside the cable bore (it opens there) past the nut's outer corner (a corner radial, a flat
    either side) by end_nut_fit; from end_nut_fit under the nut to its bearing face."""
    d, S = cfg.drive, stack_positions(cfg)
    r_out = cfg.roll_end.bolt_circle_dia / 2.0 + d.end_nut.af / math.sqrt(3.0) + d.end_nut_fit
    return d.bore / 2.0 - 1.0, r_out, S["z_end_nut"] - d.end_nut_fit, S["z_end_nut_seat"]


def clamp_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The rotor clamp's 4x M3 in the module x-y plane: on clamp_circle_dia, from clamp_angle_deg (between the wall's
    screws)."""
    d = cfg.drive
    r = d.clamp_circle_dia / 2.0
    return [(r * math.cos(math.radians(d.clamp_angle_deg + 90.0 * k)), r * math.sin(math.radians(d.clamp_angle_deg + 90.0 * k)))
            for k in range(4)]


def clamp_nut_pocket(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(r_in, r_out, z0, z1) of each clamp screw's nut pocket in the shaft's flange, radially along its screw's angle
    like end_nut_pocket's: from inside the cable bore past the nut's corner; from the shaft's rear face (open: the nut
    goes in there) to the nut's bearing face."""
    d, S = cfg.drive, stack_positions(cfg)
    r_out = d.clamp_circle_dia / 2.0 + d.clamp_nut.af / math.sqrt(3.0) + d.end_nut_fit
    return d.bore / 2.0 - 1.0, r_out, S["z_shaft_end"], S["z_clamp_nut_seat"]


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


def module_frame_in_host(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The roll drive's module frame as data in j2_link's frame before the forearm's turn (lib/mounts.py ModuleMount,
    roll_turn_deg - the elbow side's view of the forearm: the drive keeps its pose, j2_link turns about the roll axis):
    origin on the roll axis at the elbow axis' station (the elbow axis runs along module X, elbow_offset below it),
    module +Z = host -X (toward the wrist), module +X = host +Z."""
    return ((0.0, 0.0, cfg.roll_end.axis_z), (0.0, -90.0, 0.0))


def stack_positions(cfg: ForearmConfig = DEFAULT) -> dict[str, float]:
    """Module-frame stations of the roll drive (RollDriveParams; the frame, the shaft and the pulley builders emit their
    geometry at these stations, their rows at 0). From the forearm wall back: the ring's front face on the wall, the
    ring, bearing 2 right under it (the bearing boss's top), the lip, bearing 1 (the tower's rear face), the shaft behind it -
    its rear face at the rotor clamp's tips -, the bay, the tower's rear; the motor's face where its 20T meets the
    ring (pulley_seat), the plate in front of it - its face the block's front face, the motor screws' heads flush in it.
    The elbow axis runs along X at y = y_elbow, under the tower at
    z = 0; the motor sits on it. Every number a row or a test needs comes from here."""
    d, w = cfg.drive, cfg.roll_end
    z_wall = -w.wall_x[1]                                         # 48: the forearm wall's elbow face = the ring's front face
    z_ring = z_wall - d.ring_flange_t - d.ring_width              # 39.8: the teeth
    z_ring_flange_1 = z_ring - d.ring_flange_t                    # 38.6: the ring's rear flange
    z_boss = z_ring_flange_1 - d.run_gap                          # 37.6: the bearing boss's top = bearing 2's front face
    z_bearing_2 = z_boss - d.bearing_width                        # 30.6
    z_lip = z_bearing_2 - d.lip                                   # 28.6
    z_bearing_1 = z_lip - d.bearing_width                         # 21.6: the tower's rear face
    z_wall_back = -w.wall_x[0]                                    # 56: the wall's wrist face = its screws' heads
    z_end_tip = z_wall_back - w.screw_len                         # 40: their tips, in the pulley's core
    z_end_nut = z_end_tip + 2 * M3_PITCH                          # 41: each nut's far face, 2 pitches short of the tip
    z_spigot_end = z_wall + w.flange_recess_depth                 # 50: the pulley's front face
    z_clamp_head = z_spigot_end - d.clamp_screw.head_h - d.clamp_head_clear   # 46.8: the clamp's counterbores' floor
    z_shaft_end = z_clamp_head - d.clamp_screw_len                # 11.8: the clamp's tips = the shaft's rear face
    z_clamp_nut = z_shaft_end + 2 * M3_PITCH                      # 12.8: each clamp nut's far face
    z_meet = z_lip + d.lip / 2.0                                  # 29.6: the shaft's hub meets the pulley's, in the lip's middle
    z_bay = z_shaft_end - (z_meet - z_bearing_1) - d.bay_clear    # 2.75: the bay's floor - room to slide the shaft in
    z_ring_mid = z_ring + d.ring_width / 2.0                      # 43.3
    z_motor_face = z_ring_mid - d.t20                             # 26.85: the 20T's teeth level with the ring's, its hub pulley_seat over the pilot boss
    z_front = z_motor_face + d.pad_t                              # 33.85: the plate's front face = the block's front face
    z_idler = z_ring_mid - GT2_IDLER_WIDTH / 2.0                  # 38.8: the idler's underside (its channel on the belt) = the riser's land
    return {
        "z_block": 0.0, "z_shaft": 0.0, "z_pulley": 0.0,
        "z_tower_rear": z_bay - d.tower_wall,                     # 0.75: the tower's rear in the profile
        "z_bay": z_bay, "z_shaft_end": z_shaft_end,
        "z_clamp_nut": z_clamp_nut,
        "z_clamp_nut_seat": z_clamp_nut + d.clamp_nut.h,          # 15.2: its bearing face, toward the pulley
        "z_clamp_head": z_clamp_head,
        "z_end_tip": z_end_tip, "z_end_nut": z_end_nut,
        "z_end_nut_seat": z_end_nut + d.end_nut.h,                # 43.4: each wall nut's bearing face, toward the wall
        "z_stop_lug": z_shaft_end,                                # 11.8 .. +stop_lug_t on the shaft's flange
        "z_stop_post": z_bearing_1 - d.stop_post_t,               # 16.8 .. 21.6 on the tower's rear face
        "z_shoulder_1": z_bearing_1 - d.run_gap,                  # 20.6: the shaft's Ø33 shoulder on bearing 1's inner ring
        "z_bearing_1": z_bearing_1, "z_lip": z_lip,
        "z_meet": z_meet,
        "z_bearing_2": z_bearing_2, "z_boss": z_boss,
        "z_ring_flange_1": z_ring_flange_1, "z_ring": z_ring, "z_ring_mid": z_ring_mid,
        "z_ring_end": z_wall,                                     # 48: the ring's front flange face
        "z_front": z_front,
        "z_wall": z_wall,                                         # 48: the spigot starts here
        "z_spigot_end": z_spigot_end,
        "z_wall_back": z_wall_back,
        "z_motor_face": z_motor_face,
        "z_motor_board": z_motor_face - d.motor.body_length,      # -12.65: the board's frame (the motor's rear face)
        "z_motor_screws": z_front - d.motor_holes.screw.head_h,   # 30.85: the motor screws' counterbores' floor = their heads' bearing face
        "z_20t": z_motor_face + d.motor.pilot_height + d.pulley_seat,   # 32.35: the 20T's hub face, in the plate's pilot hole
        "z_idler": z_idler, "z_idler_top": z_idler + GT2_IDLER_WIDTH,   # 47.8: its top, the screw head's bearing face
        "z_idler_roof": z_front - d.idler.roof,                   # 30.85: the nut's channel's roof = the nut's bearing face
        "y_elbow": d.elbow_y,                                     # -74: the elbow axis (along X)
        "x_motor": 0.0, "y_motor": d.motor_y,                     # on the elbow axis
        "x_cradle": d.block_x[0] + d.web_t,                       # -23: the pocket's floor
    }


def motor_pocket(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(y0, y1, z0, z1) of the motor's pocket in the frame (from the floor, x_cradle, out through the open +N face): the
    motor and its board (the wider: the board's cover) + pocket_clear; from pocket_rear behind the board's cover to the
    plate (the motor's face)."""
    d, S = cfg.drive, stack_positions(cfg)
    half = max(d.motor.body_width, d.board_w) / 2.0 + d.pocket_clear
    return (S["y_motor"] - half, S["y_motor"] + half, S["z_motor_board"] - d.board_stack - d.pocket_rear, S["z_motor_face"])


def plug_slots(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float, float, float, float, float]]:
    """(x0, x1, y0, y1, z0, z1) of the two plug slots off the motor's pocket, one at each of the MKS board's side headers
    (lib/motors.py MKS_SERVO42D_HEADERS, the board turned motor_spin_deg with the motor): plug_clear round the header's
    outline, from its face along Y MKS_PLUG_ROOM past it and plug_spare more, and from the header up through the open +N
    face."""
    d, S = cfg.drive, stack_positions(cfg)
    a = math.radians(d.motor_spin_deg)
    c, s = round(math.cos(a)), round(math.sin(a))      # the board's quarter turn about the motor axis
    slots = []
    for name in ("side_neg_x", "side_pos_x"):
        (x0, y0, z0, x1, y1, z1), (ux, uy) = MKS_SERVO42D_HEADERS[name]
        xs = [S["x_motor"] + c * x - s * y for x in (x0, x1) for y in (y0, y1)]
        ys = [S["y_motor"] + s * x + c * y for x in (x0, x1) for y in (y0, y1)]
        out = s * ux + c * uy                            # the plug's way in, turned: along Y (+1 / -1)
        assert c * ux - s * uy == 0 and abs(out) == 1, "the side plugs face along Y"
        face = max(ys) if out > 0 else min(ys)
        reach = face + out * (MKS_PLUG_ROOM[name] + d.plug_spare)
        slots.append((min(xs) - d.plug_clear, d.block_x[1] + NUDGE, min(face, reach), max(face, reach),
                      S["z_motor_board"] + z0 - d.plug_clear, S["z_motor_board"] + z1 + d.plug_clear))
    return slots


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
    """The frame's underside as (diameter, x0, x1) cylinders about the elbow axis, the face downward: the lip in
    j1_link's recess, the boss, the journal, the shoulder on the upper elbow bearing's inner ring, the stub in it."""
    d = cfg.drive
    return [(d.lip_dia, *d.lip_x), (d.boss_dia, *d.boss_x), (d.journal_dia, *d.journal_x), (d.step_dia, *d.step_x),
            (d.stub_dia, *d.stub_x)]


def flange_bolt_points_module(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The wall's bolts in the module x-y plane (they run through the pulley and the shaft): the wall's (y, z) pattern
    seen from the module (module x = host z - axis_z, module y = host y)."""
    return [(z - cfg.roll_end.axis_z, y) for y, z in flange_bolt_points(cfg)]
