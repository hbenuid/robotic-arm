"""forearm_roll_mount_screws - purchased (COTS) part: the roll motor mount's 4 M3 countersunk screws (ISO 10642).

They clamp the motor mount (forearm_roll_motor_mount) into the step on the elbow block's top (forearm_roll_block).

In the arm (assemblies/forearm_roll_drive.py, a row of the module): the heads flush with the mount's base under the
motor, the shanks down through its countersunk holes into the block and through the M3 nuts in its two channels
(forearm_roll_mount_nuts); the tips end in clearance holes past the nuts. Length: lib/forearm/params.py
RollDriveParams.mount_screw_len (overall, as ISO 10642 measures it).

No catalog model (the catalog has single fasteners only, vendor/README.md) and no SolidWorks export: a NATIVE COTS
pattern part (lib/reference.py NATIVE_COTS) like elbow_pulley_screws - its envelope IS the geometry (lib/fasteners.py
csk(): no thread, the head's hex socket) and its reference is that envelope
(reference/native/forearm_roll_mount_screws.step, tools/reference/import_native.py). Frame: axis on Z, the heads' flat
tops on z=0, the cones and shanks in +Z; the four at lib/forearm/layout.py mount_bolt_points() as (x, y) = the module's
(x, z) - the module turns +Z down its -Y.
"""
import pathlib

from cadgen import step

from lib.cots import hybrid, pattern
from lib.datum import IDENTITY
from lib.fasteners import csk
from lib.forearm import DEFAULT as FOREARM
from lib.forearm import mount_bolt_points
from lib.params import M3_CSK, ROLL_MOUNT_SCREWS_MASS_G

LENGTH = FOREARM.drive.mount_screw_len

NAME = pathlib.Path(__file__).stem
COTS = True
MASS_G = ROLL_MOUNT_SCREWS_MASS_G   # [ESTIMATE] the modelled steel (lib/params.py)
PURCHASE_SPEC = f"M{M3_CSK.d:g} x {LENGTH:g} countersunk socket screw (ISO 10642)"
PURCHASE_QTY = len(mount_bolt_points())   # pieces per occurrence (the whole pattern)
PURCHASE_NOTE = "the roll motor mount into the elbow block's step, into the M3 nuts in its channels (forearm_roll_mount_nuts)"
VENDOR_STEP = pathlib.Path(__file__).resolve().parents[2] / "vendor" / f"{NAME}.step"
VENDOR_TO_REF = IDENTITY


def _envelope():
    """The four plain screws: the geometry until a vendor model exists."""
    return pattern(csk(M3_CSK, LENGTH, xy) for xy in mount_bolt_points())


@step
def forearm_roll_mount_screws():
    """Vendor geometry if present, else the envelope - always a labelled shape."""
    return hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope)


if __name__ == "__main__":
    forearm_roll_mount_screws()   # build: writes the sibling forearm_roll_mount_screws.step
