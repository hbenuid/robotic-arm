"""Derived numbers of the cycloidal drive - hole patterns, engagement depths and the axial
stack positions of every part. Pure math (no build123d) so tests and tools can use it cheaply.

``stack_positions`` is the module layout that assemblies/cycloidal_drive.py places parts with;
the numbers are those of the drive repo's assembly.py (NOT its export.py, which had the ring
pins 1 mm and the output pins 2 mm off).
"""
from __future__ import annotations

import math

from lib.cycloidal.params import DEFAULT_CONFIG, DriveConfig, compute_housing_bolt_angles
from lib.geom import hex_circumdiameter  # the arm-wide helper, re-exported for the drive's callers

__all__ = [
    "PILLAR_OVERSHOOT", "arm_mount_angles", "arm_mount_points", "compute_housing_bolt_angles", "hex_circumdiameter",
    "housing_bolt_points", "hub_height", "motor_bolt_counterbore_depth", "motor_bolt_points", "output_pin_points",
    "pillar_corners", "pillar_half_width", "ring_pin_engagement", "ring_pin_hole_depth", "ring_pin_hole_dia", "ring_pin_points", "stack_positions",
]


def _circle_points(radius: float, angles: list[float]) -> list[tuple[float, float]]:
    return [(radius * math.cos(a), radius * math.sin(a)) for a in angles]


def housing_bolt_points(cfg: DriveConfig = DEFAULT_CONFIG) -> list[tuple[float, float]]:
    """The M4 housing bolts (bolt_count) on the 125 mm circle."""
    return _circle_points(cfg.housing.bolt_circle_dia / 2.0, compute_housing_bolt_angles(cfg))


PILLAR_OVERSHOOT = 1.0    # [DESIGN] pillars past the bore (-) and OD (+); trimmed flush by the bore/OD cuts


def pillar_corners(cfg: DriveConfig, angle: float) -> list[tuple[float, float]]:
    """(x, y) of the housing pillar on the bolt at ``angle`` (radians, from +X): the radially flipped
    trapezoid, ``pillar_inner_w`` wide at the bore - PILLAR_OVERSHOOT, ``pillar_outer_w`` at the od
    + PILLAR_OVERSHOOT, in the order inner -, outer -, outer +, inner + (lib/cycloidal/housing.py)."""
    h = cfg.housing
    inner_r, outer_r = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + PILLAR_OVERSHOOT
    c, s = math.cos(angle), math.sin(angle)
    local = [
        (inner_r, -h.pillar_inner_w / 2.0),
        (outer_r, -h.pillar_outer_w / 2.0),
        (outer_r, +h.pillar_outer_w / 2.0),
        (inner_r, +h.pillar_inner_w / 2.0),
    ]
    return [(lx * c - ly * s, lx * s + ly * c) for lx, ly in local]


def pillar_half_width(cfg: DriveConfig, along: float) -> float:
    """The housing pillar's half width at ``along`` from the drive's axis, on its centre line (the sides of
    pillar_corners, carried on past its ends)."""
    h = cfg.housing
    inner_r, outer_r = h.bore_dia / 2.0 - PILLAR_OVERSHOOT, h.od / 2.0 + PILLAR_OVERSHOOT
    return (h.pillar_inner_w + (h.pillar_outer_w - h.pillar_inner_w) * (along - inner_r) / (outer_r - inner_r)) / 2.0


def ring_pin_points(cfg: DriveConfig = DEFAULT_CONFIG) -> list[tuple[float, float]]:
    """21 ring pins on the 108 mm circle, from +X."""
    g = cfg.gear
    return _circle_points(g.ring_pin_circle_radius, [2 * math.pi * i / g.num_ring_pins for i in range(g.num_ring_pins)])


def output_pin_points(cfg: DriveConfig = DEFAULT_CONFIG) -> list[tuple[float, float]]:
    """4 output pins on the 60 mm circle at 0/90/180/270 deg (disc-local)."""
    d = cfg.disc
    return _circle_points(d.output_pin_circle_dia / 2.0, [2 * math.pi * i / d.output_pin_count for i in range(d.output_pin_count)])


def motor_bolt_points(cfg: DriveConfig = DEFAULT_CONFIG) -> list[tuple[float, float]]:
    """NEMA 17 bolt square (31 mm), in the drive repo's order (+,+) (-,+) (-,-) (+,-)."""
    h = cfg.motor.bolt_pattern_square / 2.0
    return [(h, h), (-h, h), (-h, -h), (h, -h)]


def arm_mount_angles(cfg: DriveConfig = DEFAULT_CONFIG) -> list[float]:
    """Hub arm-mount bolt angles (radians): 45 deg + k*90 - between the output pins."""
    hub = cfg.output_hub
    off = math.radians(hub.arm_mount_angle_offset_deg)
    return [off + 2 * math.pi * i / hub.arm_mount_bolt_count for i in range(hub.arm_mount_bolt_count)]


def arm_mount_points(cfg: DriveConfig = DEFAULT_CONFIG) -> list[tuple[float, float]]:
    return _circle_points(cfg.output_hub.arm_mount_bolt_circle_dia / 2.0, arm_mount_angles(cfg))


def ring_pin_hole_dia(cfg: DriveConfig = DEFAULT_CONFIG) -> float:
    """4.20 mm clearance holes for the 4 mm ring / output pins."""
    return cfg.gear.ring_pin_dia - cfg.tolerances.ring_pin_press_sub


def ring_pin_engagement(cfg: DriveConfig = DEFAULT_CONFIG) -> float:
    """Pin length outside the bore zone, per end (3.5): motor plate one side, bearing-zone wall the other."""
    return (cfg.gear.ring_pin_length - cfg.stack_up.bore_zone) / 2.0


def ring_pin_hole_depth(cfg: DriveConfig = DEFAULT_CONFIG) -> float:
    """Blind ring-pin holes in the ring gear body: bore zone + engagement (31.5)."""
    return cfg.stack_up.bore_zone + ring_pin_engagement(cfg)


def motor_bolt_counterbore_depth(cfg: DriveConfig = DEFAULT_CONFIG) -> float:
    """M3 head pocket depth in the motor plate's inner face (3): the 10 mm thread reaches
    (hole depth - margin) into the motor, the rest of the plate is the pocket."""
    m, s = cfg.motor, cfg.stack_up
    plate_t = s.motor_plate_wall + s.motor_plate_inner_wall
    return plate_t - (m.motor_bolt_thread_length - (m.bolt_hole_depth - m.motor_bolt_thread_margin))


def hub_height(cfg: DriveConfig = DEFAULT_CONFIG) -> float:
    """Output hub height (28): bearing-grip zone + output wall + proud extension."""
    s = cfg.stack_up
    return s.output_bearing_total + s.output_wall + cfg.output_hub.proud_above_housing


def stack_positions(cfg: DriveConfig = DEFAULT_CONFIG) -> dict[str, float]:
    """Z (and eccentric X) positions of every part in the module frame (Z=0 = motor-plate outer face).

    Parts whose builder already emits geometry at its stack position (eccentric shaft, motor,
    motor plate) are at 0."""
    s, e, h, m = cfg.stack_up, cfg.gear.eccentricity, cfg.housing, cfg.motor
    return {
        "x_disc1": +e, "x_disc2": -e,
        "z_motor_plate": 0.0,
        "z_motor": 0.0,
        "z_mks_board": -m.body_length,                                               # -48: the MKS SERVO42D kit on the motor's rear face
        "z_eccentric_shaft": 0.0,
        "z_ring_gear_body": s.z_motor_plate_inner,                                   # 9
        "z_disc1": s.z_disc1,                                                        # 13
        "z_disc2": s.z_disc2,                                                        # 25
        "z_6814_1": s.z_output_bearings,                                             # 37
        "z_6814_2": s.z_output_bearings + cfg.bearings.out_width,                    # 47
        "z_hub": s.z_output_bearings,                                                # 37
        "z_625": s.z_output_bearings,                                                # 37
        "z_ring_pins": s.z_motor_plate_inner - ring_pin_engagement(cfg),             # 5.5
        "z_output_pins": s.z_bearing_top - cfg.output_hub.output_hub_pin_ceiling - cfg.disc.output_pin_length,   # 11
        "z_support_pin": s.z_disc2 + cfg.disc.thickness - cfg.shaft.support_pin_hole_depth,    # 24
        "z_motor_bolts": s.z_motor_plate_inner - m.motor_bolt_total_length - m.motor_bolt_recess,   # -5
        "z_housing_bolts": h.bolt_counterbore_depth - h.bolt_head_height,            # 0.5
        "z_housing_nuts": s.total_housing_depth - h.bolt_nut_depth,                  # 56
        "hub_top": s.z_output_bearings + hub_height(cfg),                            # 65
    }
