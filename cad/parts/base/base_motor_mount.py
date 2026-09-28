"""base_motor_mount - the base_yaw motor's MOUNT: a box round the motor, bolted to the base's end across the plane
x = split_x (lib/base/params.py JointParams), so it can be reprinted or swapped without the base.

In place of the SolidWorks base's +X lobe (the base's full width), a box only as wide as the motor's MKS board and
MountParams.room round it for the wiring (lib/base/layout.py mount_inner_half(), mount_x1()): two side walls and an
end wall from the bottom face (it stands on the table) up to the 5 mm plate across their tops, which carries the
48 mm motor's seat on its underside (4 slots of +/- travel along X for the motor's M3s - the base_yaw belt's tension
-, the pilot's window slotted with them; no rim round the motor's face: the slots hold it). Open underneath, and open
toward the base: the window between the base's two posts takes the cables into the base. An ear outside each side
wall at the joint face (ear_w wide, ear_t thick, full height) with 2 M4 clearance holes along X: 4x M4 SHCS
(base_motor_mount_screws),
their heads on the ears' outer faces - turned from outside, along -X beside the walls - run through the ears and the
base's posts into the M4 nuts pressed into the posts (base_motor_mount_nuts).

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
