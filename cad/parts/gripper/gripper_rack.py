"""gripper_rack - one of the gripper's two racks (PapaBravo's "Rack & Pinion Robotic Gripper Jaw", Printables, CC BY
4.0): a bar with 17 teeth along one long edge, the pinion between the two racks; its two split posts run in the base's
slot and an M3 through each clamps the jaw under the plate (the head sunk in the bar). Parametric build123d
(lib/gripper/body.py build_rack(), every number lib/gripper/params.py RackParams), in the export's frame: the bar's
bottom on z = 0, the teeth toward -y, the posts toward +z.

Source export: 'Rack.step' (product 'Rack', 338 909 bytes, sha256 e16f934aecf4..., in
rack-pinion-robotic-gripper-jaw-model_files.zip; lib/reference.py MEASURED): mm, 20 solids (fused: 2737.60 mm^3),
bbox (-28, 6.2955, 0) .. (28, 16, 10.3).

A measured conversion (parts/AGENTS.md Part states): tests/gripper/test_measured_parts.py locks the build to the export's
numbers. assemblies/gripper.py places it.
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.gripper import DEFAULT
from lib.gripper.body import build_rack

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: it drives a jaw
REFERENCE = None              # a measured conversion: no reference file (lib/reference.py MEASURED)
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the export's frame


@step(out="gripper_rack.step")
def gripper_rack():
    """The rack at its local origin; the assembly owns placement."""
    part = build_rack(DEFAULT.rack)
    part.label = NAME
    return part


if __name__ == "__main__":
    gripper_rack()   # build: writes the sibling gripper_rack.step
