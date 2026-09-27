"""forearm_roll_block - the forearm roll drive's STATOR - the elbow block, which is also the elbow's output flange: a rounded box
round the roll axis (66 x 72 x 76) whose underside repeats the SolidWorks j3_coupler's lip, Ø62 boss, Ø40 journal and Ø30 stub down
into j1_link's recess and bore (the elbow 90T pulley bolts up through the stub into 4x M4 nuts in hex channels that open into the
core bore - j3_coupler#1 is retired),
the housing bore that CROSSES the elbow axis - the rear end wall with the Ø26 cable exit on the axis, the lip bearing 1 stops on,
its Ø52.15 seat, the clearance bore round the shaft's core, the Ø62 cavity the shaft's 90T ring runs in (open through the front
face, the belt window in the top wall), the end cap's 4x M3 in the front face - and, on the block's top UP in the swing plane, the
step the bolt-on motor mount's base fills (forearm_roll_motor_mount: the 40 mm kit motor's plate), its 4 clearance holes and the two
channels the mount's M3 nuts slide into from the rear face.

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/forearm/roll.py build_block(cfg), every number
lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py
places it at 0); its reference is its accepted build, reference/native/forearm_roll_block.step (tools/reference/import_native.py).
PETG. In the arm: x1, inside forearm_roll_drive#1 - elbow_link (the stator rides with the elbow pulley; it IS the elbow coupler).
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
    forearm_roll_block()   # build: writes the sibling forearm_roll_block.step
