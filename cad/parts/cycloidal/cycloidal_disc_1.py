"""cycloidal_disc_1 - cycloidal disc 1 of the 20:1 drive (20 lobes, 0 deg profile phase).

Ported from cycloidal_drive@2f1f67d src/cycloidal_disc.py (build_cycloidal_disc()). The reference
reference/cycloidal_disc_1.step is that CadQuery builder's own export (manifest kind "designed").
Local frame: profile centred on the disc axis, z 0..thickness (10); in the drive it orbits at
(+e, 0) at stack z_disc1 (13) with a 6003 bearing in its 35.10 bore. PETG, 100 % infill.
"""
import pathlib

from build123d import Location
from cadgen import step
from lib.cycloidal import DEFAULT_CONFIG, DriveConfig
from lib.cycloidal.disc import build_disc

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME
CONVERTED = True
LOCAL_FROM_REF = Location()
# OCCT's basic volume integration is ~0.3 % off on the 2000-knot spline face (both for this part
# and its reference); tests/cycloidal/test_port.py compares the adaptive-precision volume to 1e-4.
REF_VOL_TOL = 0.005
REF_BBOX_TOL = 0.02


def phase_deg(cfg: DriveConfig = DEFAULT_CONFIG) -> float:
    return 0.0


def build(cfg: DriveConfig = DEFAULT_CONFIG):
    return build_disc(cfg, phase_deg(cfg))


@step
def cycloidal_disc_1():
    """Return the disc at its LOCAL origin."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    cycloidal_disc_1()   # build: writes the sibling cycloidal_disc_1.step (preview: ./cadtool show parts/cycloidal/cycloidal_disc_1.py)
