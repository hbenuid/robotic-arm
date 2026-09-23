"""forearm_roll_block - the forearm roll drive's STATOR - the elbow block: the SolidWorks disc's j3_coupler#1 interface (Ø54.89 bore,
4x M4 into captive hex nuts), the Ø70 housing round the roll axis - the closed elbow end (cable exit through its +X wall), the lip
bearing 1 stops on, its Ø52.15 seat, the Ø62 cavity the shaft's 90T ring runs in with the belt window in its +Y wall, a flat
underneath above the upper arm's slab, two lugs for the end cap - and the motor pad tower UP in the swing plane (slotted for belt
tension, the Ø22.3 pilot slot) for the 40 mm kit motor.

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/forearm/roll.py build_block(cfg), every number
lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py
places it at 0); its reference is its accepted build, reference/native/forearm_roll_block.step (tools/reference/import_native.py).
PETG. In the arm: x1, inside forearm_roll_drive#1 - elbow_link (the stator rides with the elbow pulley + coupler).
"""
import pathlib

from cadgen import step
from lib.datum import IDENTITY
from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_block

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/native/<NAME>.step - the accepted build
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: ForearmConfig = DEFAULT):
    return build_block(cfg)


@step
def forearm_roll_block():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_block()   # build: writes the sibling forearm_roll_block.step (preview: ./cadtool show parts/joints/forearm_roll_block.py)
