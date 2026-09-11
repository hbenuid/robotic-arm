"""gripper_slider - import wrapper around the SolidWorks reference (reference/gripper_slider.step).

SolidWorks product: 'Gripper Mechanism Slider_Gripper Mechanism Slider'
Source export:      step/Gripper Mechanism Slider_Gripper Mechanism Slider.STEP
Reference: mm units, 1 solid(s), volume 16553.7 mm^3,
           bbox size (18, 26, 60) mm, bbox min (-9, 0, -30) mm.
In the arm: x2 (gripper_slider#1, gripper_slider#2).

Not yet parametric: gripper_slider() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from build123d import Location
from cadgen import step
from lib import reference

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once gripper_slider() is parametric build123d
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def gripper_slider():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(LOCAL_FROM_REF)
    shape.label = NAME
    return shape


if __name__ == "__main__":
    gripper_slider()   # build: writes the sibling gripper_slider.step (preview: ./cadtool show parts/gripper/gripper_slider.py)
