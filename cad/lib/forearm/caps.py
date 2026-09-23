"""build123d builders of the forearm's two caps from a ForearmConfig, in j2_link's part frame (the parts carry
the LOCAL_FROM_REF that maps their SolidWorks frames here)."""
from __future__ import annotations

from cadgen import build123d as bd

from lib.cycloidal.geom import cylinder, single_solid
from lib.forearm.layout import cap1_socket_points, cap2_socket_points, elbow_end_x, motor_window
from lib.forearm.link import slab
from lib.forearm.params import DEFAULT, ForearmConfig
from lib.units import NUDGE


def _sockets(body, points, r: float, depth: float, z0: float):
    for xy in points:
        body = body - cylinder(r, depth + NUDGE, xy, z0=z0)
    return body


def build_cap_1(cfg: ForearmConfig = DEFAULT):
    """The lid: rim on the web's top face, 1.5 lid, the pocket between, the motor window through the lid."""
    c, w = cfg.cap1, cfg.web
    height = c.z1 - c.z0
    x_end = elbow_end_x(cfg)
    span = x_end - w.wrist_x
    # outline: the flat-sided strip from the elbow end to the wrist axis (+ the elbow disc's round end unless the
    # roll wall is the end), minus the concave arc round the wrist boss
    body = slab(span, 2.0 * w.half_w, height, (x_end + w.wrist_x) / 2.0, c.z0)
    if not cfg.roll:
        body = body + cylinder(cfg.disc.dia / 2.0, height, z0=c.z0)
    body = body - cylinder(c.outer_wrist_r, height + 2 * NUDGE, (w.wrist_x, 0.0), z0=c.z0 - NUDGE)
    # the pocket (rim face up to the lid): the strip |y| < pocket_half_w ending at the arc about the wrist axis;
    # toward the elbow it wraps the elbow axis (LEGACY) or runs out through the wall face (the wall closes it)
    depth = height - c.lid
    pocket = slab(span + 2 * NUDGE, 2.0 * c.pocket_half_w, depth + NUDGE, (x_end + w.wrist_x) / 2.0 + NUDGE, c.z0 - NUDGE)
    if not cfg.roll:
        pocket = pocket + cylinder(c.pocket_elbow_r, depth + NUDGE, z0=c.z0 - NUDGE)
    pocket = pocket - cylinder(c.pocket_wrist_r, depth + 3 * NUDGE, (w.wrist_x, 0.0), z0=c.z0 - 2 * NUDGE)
    body = body - pocket
    # the motor window through the lid
    x0, x1, half_w = motor_window(cfg)
    body = body - slab(x1 - x0, 2.0 * half_w, c.lid + 2 * NUDGE, (x0 + x1) / 2.0, c.z1 - c.lid - NUDGE)
    body = _sockets(body, cap1_socket_points(cfg), cfg.sockets.dia / 2.0, cfg.sockets.depth, c.z0 - NUDGE)
    return single_solid(body)


def build_cap_2(cfg: ForearmConfig = DEFAULT):
    """The belt tray: rim on the web's bottom face, 1.5 floor, the pocket between (open toward the elbow), the
    arc channel about the elbow axis through floor and rims."""
    c, w = cfg.cap2, cfg.web
    height = c.z1 - c.z0
    x_end = elbow_end_x(cfg)
    span = x_end - w.wrist_x
    wx = (w.wrist_x, 0.0)
    # outline: the strip from the elbow end to the wrist axis + the wrist boss's round end, minus (LEGACY) the
    # concave arc round the elbow; with the roll wall the strip simply ends at the wall
    body = slab(span, 2.0 * w.half_w, height, (x_end + w.wrist_x) / 2.0, c.z0)
    body = body + cylinder(cfg.boss.dia / 2.0, height, wx, z0=c.z0)
    if not cfg.roll:
        body = body - cylinder(c.outer_elbow_r, height + 2 * NUDGE, z0=c.z0 - NUDGE)
    # the pocket (floor up to the rim face): the strip |y| < pocket_half_w wrapping the wrist axis, running out
    # through the elbow end
    depth = height - c.floor
    pocket = slab(span + 2 * NUDGE, 2.0 * c.pocket_half_w, depth + NUDGE, (x_end + w.wrist_x) / 2.0 + NUDGE, c.z1 - depth)
    pocket = pocket + cylinder(c.pocket_wrist_r, depth + NUDGE, wx, z0=c.z1 - depth)
    body = body - pocket
    if not cfg.roll:
        # the arc channel from the outer face (LEGACY: it lies in the region the wall replaces)
        r0, r1 = c.channel_r
        channel = cylinder(r1, c.channel_depth + NUDGE, z0=c.z0 - NUDGE) - cylinder(r0, c.channel_depth + 3 * NUDGE, z0=c.z0 - 2 * NUDGE)
        body = body - channel
    body = _sockets(body, cap2_socket_points(cfg), cfg.sockets.dia / 2.0, cfg.sockets.depth, c.z1 - cfg.sockets.depth)
    return single_solid(body)
