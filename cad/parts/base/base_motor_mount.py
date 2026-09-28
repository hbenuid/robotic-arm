"""base_motor_mount - the base_yaw motor's MOUNT: the base's +X lobe as its own part, bolted to the base across the
plane x = split_x (lib/base/params.py JointParams), so it can be reprinted or swapped without the base.

The lobe of the SolidWorks base from the joint face out: the side walls and the end wall (the cable notch at its
foot), the 5 mm plate flush with their tops carrying the 48 mm motor's seat on its underside (4 slots of +/- travel
along X for the motor's M3s - the base_yaw belt's tension -, the pilot's window slotted with them, the belt slot from
the joint face, the U rim round the motor's face), and at the joint face a rib inside each side wall (rib_w deep,
rib_t thick, from the bottom face up to the plate) with 2 M4 clearance holes along X. 4x M4 SHCS
(base_motor_mount_screws), their heads on the ribs' inside faces, run through into the M4 nuts pressed into the base's
ribs (base_motor_mount_nuts); the hex key reaches them from inside the lobe, beside the motor.

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/base/body.py build_motor_mount(cfg), every
number lib/base/params.py), in the base's part frame - lib/mounts.py places it on base#1 at identity; its reference is
its accepted build, reference/native/base_motor_mount.step (tools/reference/import_native.py). PETG, printed plate
down (the plate's top and the walls' tops are one face): no supports. In the arm: x1 (base_link).
"""
import pathlib

from cadgen import step

from lib.base import DEFAULT, BaseConfig
from lib.base.body import build_motor_mount
from lib.datum import IDENTITY

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/native/<NAME>.step - the accepted build
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: BaseConfig = DEFAULT):
    return build_motor_mount(cfg)


@step
def base_motor_mount():
    """The part in the base's part frame (lib/mounts.py places it on base#1 at identity)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    base_motor_mount()   # build: writes the sibling base_motor_mount.step
