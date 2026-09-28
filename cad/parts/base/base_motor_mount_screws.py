"""base_motor_mount_screws - purchased (COTS) part: the base motor mount's 4 M4 socket head cap screws (ISO 4762).

They bolt the motor mount (base_motor_mount) to the base (base) across the joint face.

In the arm (lib/mounts.py, hosted on base_motor_mount#1): the heads on the inside faces of the mount's two ribs, the
shanks along the base's -X through both ribs into the M4 nuts pressed into the base's ribs
(base_motor_mount_nuts), the tips out into the base's cavity. Length: lib/base/params.py JointParams.screw_len.

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like elbow_pulley_screws - its envelope IS the geometry (lib/fasteners.py
shcs(): no thread, the head's hex socket) and its reference is that envelope
(reference/native/base_motor_mount_screws.step, tools/reference/import_native.py). Frame: axis on Z, the heads'
bearing faces on z=0, the heads in -Z and the shanks in +Z; the four at lib/base/layout.py joint_bolt_points() as
(x, y) = the base's (z, y) - the mount turns +Z down the base's -X.
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT as BASE
from lib.base import joint_bolt_points
from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import shcs
from lib.params import BASE_MOUNT_SCREWS_MASS_G, M4_SHCS

LENGTH = BASE.joint.screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = BASE_MOUNT_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M4_SHCS.d:g} x {LENGTH:g} socket head cap screw (ISO 4762)"
PURCHASE_QTY = len(joint_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "the base motor mount to the base, from inside the mount into the M4 nuts pressed into the base's ribs (base_motor_mount_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(shcs(M4_SHCS, LENGTH, xy) for xy in joint_bolt_points())


@step
def base_motor_mount_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    base_motor_mount_screws()   # build: writes the sibling base_motor_mount_screws.step
