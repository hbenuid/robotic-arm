"""forearm_roll_shaft - the rear piece of the forearm roll drive's ROTOR, ELBOW_ROLL_OFFSET above the elbow axis: a collar
that holds bearing 1 (the rear 6806) - a Ø30.3 journal through bearing 1 to its front face, where the pulley's neck meets
it (forearm_roll_pulley, the front piece and the output, its hub the long one), the Ø33 shoulder on bearing 1's inner
ring, the Ø40 flange behind it with the rotor clamp's 4 M3 nuts (on Ø24.5, pockets open to its rear face and into the
bore: the clamp's screws come from the pulley's front face through both pieces) and the hard-stop lug, at +X, that meets
the frame's post just past +/- FOREARM_ROLL_LIMIT_DEG; the Ø18 cable bore (the cables leave into the frame's bay behind
it, bored on through the frame's back - the way the shaft goes in).

NATIVE part (lib/reference.py NATIVE): designed here in build123d (lib/forearm/roll.py build_shaft(cfg), every number
lib/forearm/params.py RollDriveParams, in the drive's MODULE frame at its stack station - assemblies/forearm_roll_drive.py
places it at 0); its reference is its accepted build, reference/native/forearm_roll_shaft.step (tools/reference/import_native.py).
PETG. In the arm: x1, inside forearm_roll_drive#1 - forearm_link (with the pulley, the forearm's elbow end).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT, ForearmConfig
from lib.forearm.roll import build_shaft

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/native/<NAME>.step - the accepted build
CONVERTED = True
LOCAL_FROM_REF = IDENTITY
REF_VOL_TOL = 1e-4
REF_BBOX_TOL = 0.02


def build(cfg: ForearmConfig = DEFAULT):
    return build_shaft(cfg)


@step(out="forearm_roll_shaft.step")
def forearm_roll_shaft():
    """The part in the module frame at its stack station (the module owns placement)."""
    part = build()
    part.label = NAME
    return part


if __name__ == "__main__":
    forearm_roll_shaft()   # build: writes the sibling forearm_roll_shaft.step
