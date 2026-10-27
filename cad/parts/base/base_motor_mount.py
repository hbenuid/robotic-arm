"""base_motor_mount - the base_yaw motor's MOUNT: the base's other half, bolted to the base's end across the plane
x = split_x (lib/base/params.py JointParams), so it can be reprinted or swapped without the base.

In place of the SolidWorks base's +X lobe, the base's D carried on at full width (MountParams): the same walls,
leaning out at the base's draft and shell.wall thick, from the joint face out to a round end that mirrors the base's
round half (its centre lib/base/layout.py mount_end_x(), the end mount_x1() out at the plate's top, MountParams.room
past the MKS board's square), from the bottom face (it stands on the table) up to the 5 mm plate across their tops; the
base's foot carried on round it (the flange and the chamfer, 4 screw holes with spot-faces, FootParams.mount_hole_deg);
the band of slits through the walls the base's walls have too (SlitParams, lib/base/layout.py slits()). The plate
carries the 48 mm motor's seat on its underside (lib/base/params.py MotorParams.holes / pad_t: a pad round the motor's
face under the plate, 4 round holes through both for the motor's M3s (parts/base/yaw_motor_screws), their heads flush
in counterbores in the plate's top, the pilot's round hole; no rim round the motor's face: the screws hold it) and the
base_yaw belt's tensioner (lib/base/params.py YawIdlerParams): the idler's
axle's slot through the plate across the belt, and under the plate two rails either side of its nut's slot - the nut
bears up on the plate's underside between them and slides with the axle (lib/base/body.py _idler_seat). Open
underneath, and open toward the base: the window between the two parts' posts takes the cables into the base. Two
posts at the joint face (post_t deep, from the window out to the walls, full height below the plate) facing the
base's, with 2 M4 clearance holes each along X: 4x M4 SHCS (base_motor_mount_screws), their heads on the posts' inner
faces inside the mount - turned along -X from underneath, the base off the table - run through both pairs of posts
into the M4 nuts pressed into the base's posts (base_motor_mount_nuts).

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/base/body.py build_motor_mount(cfg), every
number lib/base/params.py), in the base's part frame - lib/mounts.py places it on base#1 at identity; its reference is
its accepted build, reference/native/base_motor_mount.step (tools/reference/import_native.py). PETG, printed plate
down (the plate's top and the walls' tops are one face): the slits upright; the foot's chamfer and rim overhang at the
top of the print (docs/open_issues.md). In the arm: x1 (base_link).
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


@step(out="base_motor_mount.step")
def base_motor_mount():
    """The part in the base's part frame (lib/mounts.py places it on base#1 at identity)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    base_motor_mount()   # build: writes the sibling base_motor_mount.step
