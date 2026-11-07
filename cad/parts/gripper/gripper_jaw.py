"""gripper_jaw - one of the gripper's two jaws (PapaBravo's "Rack & Pinion Robotic Gripper Jaw", Printables, CC BY 4.0):
an arm under the base's plate, clamped to its rack's posts by two M3 into the nuts in its bosses, and at one end the
finger - its gripping face flat, its back a tilted half-round, its top rounded. Parametric build123d
(lib/gripper/body.py build_jaw(), every number lib/gripper/params.py JawParams), in the export's frame: the arm's
bottom on z = 0, the finger toward +z, its face at x = 11.5 facing -x.

Source export: 'Jaw.step' (product 'Jaw', 94 751 bytes, sha256 1034eae0e46b..., in
rack-pinion-robotic-gripper-jaw-model_files.zip; lib/reference.py MEASURED): mm, 3 solids (fused: 5878.94 mm^3),
bbox (-28, -8, 0) .. (28, 16, 45). The finger's top rounds are a rolling ball here, a little leaner than the export's
blend (1.4 mm^3 in all).

A measured conversion (parts/AGENTS.md Part states): tests/gripper/test_measured_parts.py locks the build to the export's
numbers. NOT PLACED yet (UNPLACED below).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.gripper import DEFAULT
from lib.gripper.body import build_jaw

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: it grips
REFERENCE = None              # a measured conversion: no reference file (lib/reference.py MEASURED)
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the export's frame
UNPLACED = "the rack & pinion gripper's module (assemblies/gripper.py) places it, replacing the SolidWorks gripper"


@step(out="gripper_jaw.step")
def gripper_jaw():
    """The jaw at its local origin; the assembly owns placement."""
    part = build_jaw(DEFAULT.jaw)
    part.label = NAME
    return part


if __name__ == "__main__":
    gripper_jaw()   # build: writes the sibling gripper_jaw.step
