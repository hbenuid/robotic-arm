"""yaw_idler_spacer - the printed spacer of the base_yaw belt's tensioner (parametric build123d, lib/base/body.py
build_yaw_idler_spacer): a stepped sleeve on the idler's axle between the idler and the motor mount's plate.

base_motor_mount prints plate-down (its plate's top on the bed), so no riser grows from the plate's top: the riser is a
part of its own, as on the elbow (parts/joints/elbow_idler_spacer) - its riser (YawIdlerParams.riser_dia) on the
plate's top, its land (land_dia x land_h) under the idler's bearing inner ring only - the idler's flanges clear of the
riser -, its bore the axle's slot (slot_w); YAW_IDLER_SPACER_H tall, so the idler stands level with the belt
(lib/params.py). Frame: axis on Z, the land's face on z=0, the riser's on z=YAW_IDLER_SPACER_H. Prints riser down.

In the arm: x1 (yaw_idler_spacer#1, a mount on base_motor_mount#1 - lib/mounts.py YAW_IDLER_MOUNTS).

No SolidWorks export and NO reference file (lib/reference.py NO_REFERENCE): designed here, its own test locks its
numbers - tests/base/test_yaw_idler.py.
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT
from lib.base.body import build_yaw_idler_spacer
from lib.datum import IDENTITY

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/base/test_yaw_idler.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (on the idler's axle)


@step
def yaw_idler_spacer():
    """The spacer at its local origin (the land's face on the axis); the assembly owns placement."""
    part = build_yaw_idler_spacer(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    yaw_idler_spacer()   # build: writes the sibling yaw_idler_spacer.step
