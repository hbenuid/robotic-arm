"""forearm_roll_motor_mount - the forearm roll motor's MOUNT, bolted onto the elbow block's top: a base that fills the step cut
into the block's top (the block's own rounded outline, y 32..36, from its rear face to the plate's front face - its top where the
block's top was, so the motor keeps its clearance) and the vertical 3 mm plate rooted in it, UP in the swing plane (the four
tension slots for the motor's M3s, the Ø22.3 pilot slot); 4x M3 countersunk (flush, under the motor) down through the base into
the M3 nuts in the block's channels (forearm_roll_mount_screws / _nuts).

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/forearm/roll.py build_motor_mount(cfg), every number
lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py
places it at 0); its reference is its accepted build, reference/native/forearm_roll_motor_mount.step
(tools/reference/import_native.py). PETG, printed base down. In the arm: x1, inside forearm_roll_drive#1 - elbow_link.
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_motor_mount

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/native/<NAME>.step - the accepted build
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: ForearmConfig = DEFAULT):
    return build_motor_mount(cfg)


@step
def forearm_roll_motor_mount():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_motor_mount()   # build: writes the sibling forearm_roll_motor_mount.step
