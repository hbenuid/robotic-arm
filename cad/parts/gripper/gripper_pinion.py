"""gripper_pinion - the gripper's pinion (PapaBravo's "Rack & Pinion Robotic Gripper Jaw", Printables, CC BY 4.0): a
14-tooth 14.5 deg involute spur, pressed onto the MG996R's spline (its tight bore from the top: the spline's teeth cut
their own grooves - no teeth in the bore) and held by the servo's M3, its head sunk in the bottom; it drives both racks.
Parametric build123d (lib/gripper/body.py build_pinion(), every number lib/gripper/params.py PinionParams), in the
export's frame: on the z axis, its bottom (the head's side) on z = 0, a tooth centred on +x.

Source export: 'Spur Gear (14 teeth) (1).step' (product 'Spur Gear (14 teeth) (1)', 1 037 775 bytes, sha256
3a192054d925..., 'SpurGear-14 teeth.step' in rack-pinion-robotic-gripper-jaw-model_files.zip; lib/reference.py MEASURED):
mm, 1 solid, 833.97 mm^3, bbox (-8.16, -8.0297, 0) .. (8.16, 8.0297, 6.5). Its flanks are fitted: a 14.5 deg involute
of module 1.02 thinned to 1.528 on the pitch circle (2-D symmetric difference 0.018 mm^2 against the export's).

A measured conversion (parts/AGENTS.md Part states): tests/gripper/test_measured_parts.py locks the build to the export's
numbers. assemblies/gripper.py places it.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.gripper import DEFAULT
from lib.gripper.body import build_pinion

NAME = pathlib.Path(__file__).stem
ACCENT = True                 # the accent colour, lib/style.py: it drives the racks
REFERENCE = None              # a measured conversion: no reference file (lib/reference.py MEASURED)
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the export's frame


@step(out="gripper_pinion.step")
def gripper_pinion():
    """The pinion at its local origin (on the z axis, the head's side on z = 0); the assembly owns placement."""
    part = build_pinion(DEFAULT.pinion)
    part.label = NAME
    return part


if __name__ == "__main__":
    gripper_pinion()   # build: writes the sibling gripper_pinion.step
