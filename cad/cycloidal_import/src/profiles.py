"""Epitrochoid profile generation for cycloidal discs.

Pure numpy math — no CadQuery dependency. Profile equations from spec Section 8.
"""

import numpy as np
from typing import List, Tuple


def compute_epitrochoid(
    R: float,
    r: float,
    N: int,
    e: float,
    num_points: int = 2000,
) -> List[Tuple[float, float]]:
    """Compute the cycloidal disc epitrochoid profile.

    Parameters
    ----------
    R : float
        Ring pin circle radius (mm). Spec: 54.0
    r : float
        Ring pin radius (mm). Spec: 2.0
    N : int
        Number of ring pins. Spec: 21
    e : float
        Eccentricity (mm). Spec: 1.5
    num_points : int
        Number of points per revolution.

    Returns
    -------
    List of (x, y) tuples suitable for CadQuery splineApprox.
    """
    theta = np.linspace(0, 2 * np.pi, num_points, endpoint=False)

    psi = np.arctan2(
        np.sin((1 - N) * theta),
        (R / (e * N)) - np.cos((1 - N) * theta),
    )

    x = R * np.cos(theta) - r * np.cos(theta + psi) - e * np.cos(N * theta)
    y = -R * np.sin(theta) + r * np.sin(theta + psi) + e * np.sin(N * theta)

    return [(float(x[i]), float(y[i])) for i in range(num_points)]


def compute_profile_radii(
    points: List[Tuple[float, float]],
) -> Tuple[float, float]:
    """Return (min_radius, max_radius) of the profile for clearance checks."""
    arr = np.array(points)
    radii = np.sqrt(arr[:, 0] ** 2 + arr[:, 1] ** 2)
    return float(radii.min()), float(radii.max())
