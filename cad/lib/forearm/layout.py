"""Pure-math layout of the forearm: hole patterns and the pieces every builder and test shares."""
from __future__ import annotations

import math

from lib.forearm.params import DEFAULT, ForearmConfig


def disc_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The elbow disc's 4x M4, on the axes."""
    r = cfg.disc.bolt_r
    return [(r, 0.0), (0.0, r), (-r, 0.0), (0.0, -r)]


def disc_bolt_angles(cfg: ForearmConfig = DEFAULT) -> list[float]:
    """The hex-pocket key angle of each bolt (radians): a vertex points at the disc centre."""
    return [math.atan2(y, x) for x, y in disc_bolt_points(cfg)]


def elbow_end_x(cfg: ForearmConfig = DEFAULT) -> float:
    """Where the web (and the caps) end toward the elbow: the wall's wrist face with the roll joint, else the
    elbow pivot (the disc's round end continues to +45)."""
    return cfg.roll_end.wall_x[0] if cfg.roll else 0.0


def _grid(cfg: ForearmConfig, x_max: float) -> list[tuple[float, float]]:
    """The socket grid columns that lie at x < x_max (a part that ends before a column has no socket there)."""
    s = cfg.sockets
    return [(x, y) for x in s.grid_x if x + s.dia / 2.0 < x_max for y in (s.grid_y, -s.grid_y)]


def link_socket_points(cfg: ForearmConfig = DEFAULT) -> tuple[list, list]:
    """(top-face sockets, bottom-face sockets) of the web."""
    grid = _grid(cfg, elbow_end_x(cfg))
    return grid, grid + list(cfg.sockets.wrist)


def cap1_socket_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    return _grid(cfg, elbow_end_x(cfg))


def cap2_socket_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    x_max = elbow_end_x(cfg) if cfg.roll else -(cfg.cap2.outer_elbow_r ** 2 - cfg.web.half_w ** 2) ** 0.5
    return _grid(cfg, x_max) + list(cfg.sockets.wrist)


def motor_window(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float]:
    """(x0, x1, half_w) of j2_cap_1's motor window: window_len long, window_offset from the motor axis."""
    c = cfg.cap1
    centre = cfg.motor_x + c.window_offset
    return centre - c.window_len / 2.0, centre + c.window_len / 2.0, c.window_half_w


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
    """Module-frame stations of the roll drive (RollDriveParams; the housing, the shaft and the cap builders emit their
    geometry at these stations - the housing's and the shaft's rows are at 0, the cap's at z_cap). Every number a
    row or a test needs comes from here."""
    d, w = cfg.drive, cfg.roll_end
    z_lip = d.z_end + d.end_wall                                   # 43
    z_seat = z_lip + d.lip                                        # 46: bearing 1
    z_cavity = z_seat + d.bearing_width                           # 53: the cavity (the shaft's shoulder 1)
    z_ring_flange_1 = z_cavity + d.ring_gap                       # 56.8
    z_ring = z_ring_flange_1 + d.ring_flange_t                    # 58: the teeth
    z_ring_mid = z_ring + d.ring_width / 2.0                      # 61.5
    z_ring_end = z_ring + d.ring_width + d.ring_flange_t          # 66.2: the second flange's face (shoulder 2)
    z_face = z_ring_end + d.ring_gap                              # 70: the housing's wrist face = the cap = bearing 2
    z_neck = z_face + d.bearing_width                             # 77
    z_cap_outer = z_neck + d.cap_lip                              # 79
    z_wall = -w.wall_x[1]                                         # 88: the forearm wall's elbow face = the shaft's end
    z_motor_face = z_ring_mid - d.t20                             # 47.05
    return {
        "z_block": 0.0, "z_shaft": 0.0,
        "z_end": d.z_end, "z_lip": z_lip, "z_seat": z_seat, "z_cavity": z_cavity,
        "z_bearing_1": z_seat, "z_bearing_2": z_face,
        "z_shaft_end": z_lip + d.shaft_end_clear,                 # 43.5
        "z_shoulder_1": z_cavity, "z_ring_flange_1": z_ring_flange_1, "z_ring": z_ring, "z_ring_mid": z_ring_mid,
        "z_ring_end": z_ring_end, "z_shoulder_2": z_ring_end,
        "z_face": z_face, "z_cap": z_face, "z_neck": z_neck, "z_cap_outer": z_cap_outer,
        "z_stop_post": z_cap_outer,                               # 79..82 on the cap's outer face
        "z_stop_lug": z_cap_outer + d.stop_t - 1.0,               # 81..84 on the neck (1 mm overlap with the post)
        "z_wall": z_wall,                                         # 88: the spigot starts here
        "z_spigot_end": z_wall + w.flange_recess_depth,           # 90
        "z_motor_face": z_motor_face,
        "z_motor_board": z_motor_face - d.motor.body_length,      # 7.55
        "z_pad_top": z_motor_face + d.pad_t,                      # 50.05
        "z_20t": z_motor_face + d.pad_t + d.pulley_lift,          # 50.55: the 20T's hub face
        "x_motor": d.motor_x, "y_motor": d.motor_y,
    }


def pad_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The motor pad's 4 bolt slots (centres) in the module x-y plane (the motor's 31 mm square, axis-aligned)."""
    from lib.cycloidal.layout import motor_bolt_points
    d = cfg.drive
    return [(d.motor_x + x, d.motor_y + y) for x, y in motor_bolt_points()]


def pad_slot_angle_deg(cfg: ForearmConfig = DEFAULT) -> float:
    """The tension slots' direction: from the roll axis toward the motor axis, from +X toward +Y."""
    d = cfg.drive
    return math.degrees(math.atan2(d.motor_y, d.motor_x))


def cap_bolt_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The end cap's 2x M3 into the housing's lugs: the top face side (+X) and below in the swing plane (-Y)."""
    return [(cfg.drive.lug_y, 0.0), (0.0, -cfg.drive.lug_y)]


def flange_bolt_points_module(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    """The wall's bolts in the module x-y plane (they run into the shaft's end wall): the wall's (y, z) pattern seen
    from the module (module x = host z - axis_z, module y = host y)."""
    return [(z - cfg.roll_end.axis_z, y) for y, z in flange_bolt_points(cfg)]
