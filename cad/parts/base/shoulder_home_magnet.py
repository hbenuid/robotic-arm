"""shoulder_home_magnet - purchased (COTS) part: the shoulder_pitch home magnet, an N52 NdFeB disc (lib/sensors.py
HOME_MAGNET_*), the same as base_yaw's.

Glued onto the outer face of a pillar of the cycloidal drive's turning shell (lib/yaw_coupler/params.py HomeTrayParams
pillar / magnet_z: over the coupler's pocket), its SOUTH pole out - toward the KY-003 on its tray in the pocket
(ky003_hall_sensor on shoulder_home_tray): the A3144 switches on a south pole at its branded face. It turns with the
shell (j1_link's) and stands over the chip at shoulder_pitch SHOULDER_PITCH_HOME_DEG (lib/params.py). Mark the pole before
gluing (the module's LED lights with the south pole toward the chip) and centre it on the pillar's face: the face is
8.8 mm wide, and 1 mm off along it moves home 0.9 deg (docs/open_issues.md).

In the arm: x1 (shoulder_home_magnet#1, a mount on j1_link#1 - lib/mounts.py SHOULDER_HOME_MOUNTS; upper_arm_link).

No catalog model (none adopted), no export and NO reference file: a part with no reference (lib/reference.py
NO_REFERENCE) - its envelope IS the geometry (a plain disc) and tests/base/test_shoulder_home.py locks its numbers.
Frame: axis on Z, its south pole's face on z = 0, the disc in +Z (toward the face it is glued to).
"""
import pathlib

from cad import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.geom import cylinder
from lib.params import HOME_MAGNET_DIA, HOME_MAGNET_T, SHOULDER_HOME_MAGNET_MASS_G

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = SHOULDER_HOME_MAGNET_MASS_G   # [ESTIMATE] the modelled NdFeB (lib/params.py)
PURCHASE_SPEC = f"{HOME_MAGNET_DIA:g} x {HOME_MAGNET_T:g} mm N52 NdFeB disc magnet (magnetised through its thickness)"
PURCHASE_QTY = 1   # pieces per occurrence
PURCHASE_NOTE = ("the shoulder_pitch home magnet: glued onto the drive shell's pillar over the coupler's pocket, centred "
                 "on its face, south pole out (check it on the KY-003: its LED lights)")
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The plain disc: the geometry until a vendor model exists."""
    return pattern([cylinder(HOME_MAGNET_DIA / 2.0, HOME_MAGNET_T)])


@step(out="shoulder_home_magnet.step")
def shoulder_home_magnet():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    shoulder_home_magnet()   # build: writes the sibling shoulder_home_magnet.step
