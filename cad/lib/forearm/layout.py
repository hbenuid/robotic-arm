"""Pure-math layout of the forearm: hole patterns and the pieces every builder and test shares."""
from __future__ import annotations

import math

from lib.forearm.params import DEFAULT, ForearmConfig


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


def module_frame_in_host(cfg: ForearmConfig = DEFAULT) -> tuple:
    """The roll drive's module frame as data in j2_link's frame (lib/mounts.py ModuleMount): origin on the roll axis
    at the elbow-axis crossing, module +Z = host -X (toward the wrist), module +X = host +Z."""
    return ((0.0, 0.0, cfg.roll_end.axis_z), (0.0, -90.0, 0.0))


def stack_positions(cfg: ForearmConfig = DEFAULT) -> dict[str, float]:
    """Module-frame stations of the roll drive (RollDriveParams; the block, the shaft and the cap builders emit their
    geometry at these stations - the block's and the shaft's rows are at 0, the cap's at z_cap). The elbow axis
    crosses the roll axis at z = 0, inside the block. Every number a row or a test needs comes from here."""
    d, w = cfg.drive, cfg.roll_end
    z_end = d.block_z[0]                                          # -40: the rear end wall's outer face
    z_lip = z_end + d.end_wall                                    # -37: the end wall's inner face, the lip begins
    z_seat = z_lip + d.lip                                        # -35: bearing 1
    z_bore = z_seat + d.bearing_width                             # -28: the clearance bore (the shaft's shoulder 1)
    z_cavity = d.cavity_z0                                        # 16: the cavity, open to the front face
    z_ring = d.ring_z0                                            # 18: the teeth
    z_ring_flange_1 = z_ring - d.ring_flange_t                    # 16.8
    z_ring_mid = z_ring + d.ring_width / 2.0                      # 21.5
    z_ring_end = z_ring + d.ring_width + d.ring_flange_t          # 26.2: the second flange's face
    z_face = d.block_z[1]                                         # 36: the front face = the cap = bearing 2
    z_neck = z_face + d.bearing_width                             # 43
    z_cap_outer = z_neck + d.cap_lip                              # 45: the cap's outer face, the stops stand on it
    z_wall = -w.wall_x[1]                                         # 48: the forearm wall's elbow face = the spigot starts
    z_motor_face = z_ring_mid - d.t20                             # 7.05
    z_motor_board = z_motor_face - d.motor.body_length            # -32.45
    return {
        "z_block": 0.0, "z_shaft": 0.0,
        "z_end": z_end, "z_lip": z_lip, "z_seat": z_seat, "z_bore": z_bore, "z_cavity": z_cavity,
        "z_bearing_1": z_seat, "z_bearing_2": z_face,
        "z_shaft_end": z_lip + d.shaft_end_clear,                 # -36
        "z_shoulder_1": z_bore, "z_ring_flange_1": z_ring_flange_1, "z_ring": z_ring, "z_ring_mid": z_ring_mid,
        "z_ring_end": z_ring_end,
        "z_face": z_face, "z_cap": z_face, "z_neck": z_neck, "z_cap_outer": z_cap_outer,
        "z_stop_post": z_cap_outer,                               # 45..47.5 on the cap's outer face
        "z_stop_lug": z_cap_outer,                                # 45..47.5 on the neck
        "z_wall": z_wall,                                         # 48: the spigot starts here
        "z_spigot_end": z_wall + w.flange_recess_depth,           # 50
        "z_motor_face": z_motor_face,
        "z_motor_board": z_motor_board,
        "z_pad_top": z_motor_face + d.pad_t,                      # 10.05: the plate's front face
        "z_20t": z_motor_face + d.pad_t + d.pulley_lift,          # 10.55: the 20T's hub face
        "z_cheek": z_motor_board + 1.0,                           # -31.45: the cheeks start 1 mm clear of the board
        "x_motor": 0.0, "y_motor": d.motor_y,
        "y_plate_top": d.motor_y + d.plate_w / 2.0,               # 83.9
    }


def pad_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The motor plate's 4 bolt slots (centres) in the module x-y plane (the motor's 31 mm square, axis-aligned)."""
    from lib.cycloidal.layout import motor_bolt_points
    S = stack_positions(cfg)
    return [(S["x_motor"] + x, S["y_motor"] + y) for x, y in motor_bolt_points()]


def cap_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The end cap's 4x M3 into the block's front face: cap_bolt_inset inside the outline's corners."""
    d = cfg.drive
    xs = (d.block_x[1] - d.cap_bolt_inset, d.block_x[0] + d.cap_bolt_inset)
    ys = (d.block_y[0] + d.cap_bolt_inset, d.block_y[1] - d.cap_bolt_inset)
    return [(x, y) for y in ys for x in xs]


def pulley_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The elbow 90T pulley's 4x M4 into the block's underside, as (y, z) about the elbow axis (module X): on the
    axes, pulley_bolt_r out - the SolidWorks coupler's pattern."""
    r = cfg.drive.pulley_bolt_r
    return [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)]


def coupler_steps(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float, float]]:
    """The block's underside as (diameter, x0, x1) cylinders about the elbow axis, the block's face downward: the lip
    in j1_link's recess, the boss, the journal, the stub the elbow bearings sit on."""
    d = cfg.drive
    return [(d.lip_dia, *d.lip_x), (d.boss_dia, *d.boss_x), (d.journal_dia, *d.journal_x), (d.stub_dia, *d.stub_x)]


def belt_window(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float, float]:
    """(z0, z1, half_x, y0) of the belt window through the block's top wall: the ring's flanges + the margin along Z,
    +/- half_x, from y0 inside the cavity out through the top."""
    d, S = cfg.drive, stack_positions(cfg)
    return (S["z_ring_flange_1"] - d.belt_window_margin, S["z_ring_end"] + d.belt_window_margin,
            d.belt_window_half_x, d.belt_window_y0)


def flange_bolt_points_module(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The wall's bolts in the module x-y plane (they run into the shaft's end wall): the wall's (y, z) pattern seen
    from the module (module x = host z - axis_z, module y = host y)."""
    return [(z - cfg.roll_end.axis_z, y) for y, z in flange_bolt_points(cfg)]
