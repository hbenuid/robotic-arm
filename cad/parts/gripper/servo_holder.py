"""servo_holder - import wrapper around the SolidWorks reference (reference/servo_holder.step).

SolidWorks product: 'Servo Holder_Servo Holder'
Source export:      step/Servo Holder_Servo Holder.STEP
Reference: mm units, 1 solid(s), volume 14670.7 mm^3,
           bbox size (44, 15, 72.5) mm, bbox min (-22, 0, -36.25) mm.
In the arm: x1 (servo_holder#1).

Not yet parametric: servo_holder() returns the reference geometry in the SolidWorks part-file
frame. See parts/_templates/wrapper.py for how to convert it to build123d.
"""
import pathlib

from build123d import Location
from cadgen import step
from lib import reference

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/<NAME>.step
CONVERTED = False             # True once servo_holder() is parametric build123d
LOCAL_FROM_REF = Location()   # reference frame -> this part's local frame (identity = SolidWorks frame)


@step
def servo_holder():
    """Return the reference geometry as a labelled Solid/Compound in this part's local frame."""
    shape = reference.load(REFERENCE).moved(LOCAL_FROM_REF)
    shape.label = NAME
    return shape


if __name__ == "__main__":
    servo_holder()   # build: writes the sibling servo_holder.step (preview: ./cadtool show parts/gripper/servo_holder.py)
