"""gripper_base - the printed frame of the gripper (PapaBravo's "Rack & Pinion Robotic Gripper Jaw", Printables, CC BY
4.0): a plate with the jaws' two slots, a wall each long side and a rib inside it (each rack runs between the two), and
two posts the MG996R's tabs screw onto - the servo hangs upside down between them, its spline over the plate's middle.
Parametric build123d (lib/gripper/body.py build_base(), every number lib/gripper/params.py BaseParams), in the export's
frame: the plate's bottom on z = 0, centred, the posts toward +z.

Source export: 'Base.step' (product 'Base', 166 620 bytes, sha256 79647804864c..., in
rack-pinion-robotic-gripper-jaw-model_files.zip; lib/reference.py MEASURED): mm, 5 solids (fused: 13669.42 mm^3),
bbox (-40, -18, 0) .. (40, 18, 22).

A measured conversion (parts/AGENTS.md Part states): the export was measured once and is not committed - no reference
file, no manifest entry; tests/gripper/test_measured_parts.py locks the build to the export's numbers. NOT PLACED yet
(UNPLACED below).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.gripper import DEFAULT
from lib.gripper.body import build_base

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # a measured conversion: no reference file (lib/reference.py MEASURED)
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the export's frame
UNPLACED = "the rack & pinion gripper's module (assemblies/gripper.py) places it, replacing the SolidWorks gripper"


@step(out="gripper_base.step")
def gripper_base():
    """The base at its local origin (the plate's bottom, centred); the assembly owns placement."""
    part = build_base(DEFAULT.base)
    part.label = NAME
    return part


if __name__ == "__main__":
    gripper_base()   # build: writes the sibling gripper_base.step
