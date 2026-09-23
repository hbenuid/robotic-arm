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


def _grid(cfg: ForearmConfig, x_max: float) -> list[tuple[float, float]]:
    """The socket grid columns that lie at x < x_max (a part that ends before a column has no socket there)."""
    s = cfg.sockets
    return [(x, y) for x in s.grid_x if x + s.dia / 2.0 < x_max for y in (s.grid_y, -s.grid_y)]


def link_socket_points(cfg: ForearmConfig = DEFAULT) -> tuple[list, list]:
    """(top-face sockets, bottom-face sockets) of the web."""
    x_max = 0.0 if not cfg.roll else cfg.wall_x[0]
    grid = _grid(cfg, x_max)
    return grid, grid + list(cfg.sockets.wrist)


def cap1_socket_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    return _grid(cfg, 0.0 if not cfg.roll else cfg.wall_x[0])


def cap2_socket_points(cfg: ForearmConfig = DEFAULT) -> list[tuple[float, float]]:
    x_max = -(cfg.cap2.outer_elbow_r ** 2 - cfg.web.half_w ** 2) ** 0.5 if not cfg.roll else cfg.wall_x[0]
    return _grid(cfg, x_max) + list(cfg.sockets.wrist)


def motor_window(cfg: ForearmConfig = DEFAULT) -> tuple[float, float, float]:
    """(x0, x1, half_w) of j2_cap_1's motor window: window_len long, centred on the motor axis."""
    c = cfg.cap1
    return cfg.motor_x - c.window_len / 2.0, cfg.motor_x + c.window_len / 2.0, c.window_half_w
