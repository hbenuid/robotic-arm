"""Epitrochoid profile of the cycloidal discs - pure numpy, no CAD (spec section 8).

Ported verbatim from cycloidal_drive@2f1f67d src/profiles.py; ``profile_points`` adds the disc-2
phase rotation that the CadQuery disc builder applied inline.
"""
from __future__ import annotations

import math

import numpy as np

from lib.cycloidal.params import DEFAULT_CONFIG, DriveConfig


def compute_epitrochoid(R: float, r: float, N: int, e: float, num_points: int = 2000) -> list[tuple[float, float]]:
    """(x, y) points of the disc profile: ring-pin circle radius R (54), pin radius r (2),
    N ring pins (21), eccentricity e (1.5). ``endpoint=False`` leaves no duplicated closing
    point, which is what makes the periodic spline through them legal."""
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
    psi = np.arctan2(np.sin((1 - N) * theta), (R / (e * N)) - np.cos((1 - N) * theta))
    x = R * np.cos(theta) - r * np.cos(theta + psi) - e * np.cos(N * theta)
    y = -R * np.sin(theta) + r * np.sin(theta + psi) + e * np.sin(N * theta)
    return [(float(x[i]), float(y[i])) for i in range(num_points)]


def compute_profile_radii(points: list[tuple[float, float]]) -> tuple[float, float]:
    """(min_radius, max_radius) of a profile, for clearance checks."""
    arr = np.array(points)
    radii = np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2)
    return float(radii.min()), float(radii.max())


def profile_points(cfg: DriveConfig = DEFAULT_CONFIG, phase_offset_deg: float = 0.0) -> list[tuple[float, float]]:
    """The disc profile for ``cfg``, rotated by ``phase_offset_deg`` about the disc centre
    (disc 1: 0; disc 2: ``cfg.gear.disc2_phase_deg``). Plain Python floats."""
    g = cfg.gear
    points = compute_epitrochoid(
        R=g.ring_pin_circle_radius, r=g.ring_pin_radius, N=g.num_ring_pins, e=g.eccentricity,
        num_points=cfg.profile.num_points,
    )
    if phase_offset_deg != 0.0:
        a = math.radians(phase_offset_deg)
        cos_a, sin_a = math.cos(a), math.sin(a)
        points = [(cos_a * x - sin_a * y, sin_a * x + cos_a * y) for (x, y) in points]
    return points
