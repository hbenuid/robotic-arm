"""Gripper mechanism - the 'Gripper Mechanism' SolidWorks sub-assembly (19 occurrences),
built in its own frame; assemblies/arm.py places the whole module at "gripper#1".

Run:  ./cadtool gen assemblies/gripper.py       -> assemblies/gripper.step (git-ignored)
      ./cadtool viewer assemblies/gripper.step  -> its page on the machine's cad server
"""

from cadgen import step

from assemblies._occurrences import occurrence_children
from lib.assembly import assembly

# (part, role, placements.json key) in SolidWorks document order. Duplicate parts carry an
# ordinal role (finger/rail/end/slider/link pairs): labels gripper_end:1, gripper_end:2, ...
OCCURRENCES = [
    ("servo_holder",         None, "servo_holder#1"),
    ("mg996r_servo",         None, "mg996r_servo#1"),
    ("mg996r_horn",          None, "mg996r_horn#1"),
    ("gripper_cover",        None, "gripper_cover#1"),
    ("gripper_finger_left",  "1",  "gripper_finger_left#1"),
    ("gripper_rail_6mm",     "1",  "gripper_rail_6mm#1"),
    ("gripper_end",          "1",  "gripper_end#1"),
    ("gripper_finger_right", "1",  "gripper_finger_right#1"),
    ("gripper_rail_6mm",     "2",  "gripper_rail_6mm#2"),
    ("gripper_slider",       "1",  "gripper_slider#1"),
    ("gripper_slider",       "2",  "gripper_slider#2"),
    ("gripper_finger_left",  "2",  "gripper_finger_left#2"),
    ("gripper_finger_right", "2",  "gripper_finger_right#2"),
    ("gripper_end",          "2",  "gripper_end#2"),
    ("gripper_link_1",       "1",  "gripper_link_1#1"),
    ("gripper_link_2",       "1",  "gripper_link_2#1"),
    ("gripper_link_1",       "2",  "gripper_link_1#2"),
    ("gripper_link_2",       "2",  "gripper_link_2#2"),
    ("gripper_j3_connector", None, "gripper_j3_connector#1"),
]


@step(out="gripper.step")
def gripper():
    """The gripper module in its own (SolidWorks sub-assembly) frame, every part in its colour as in the arm
    (_occurrences.colour(): the fingers orange, the other printed parts off-white, servo, horn and rails grey)."""
    return assembly("gripper", occurrence_children(OCCURRENCES, tint=True))


if __name__ == "__main__":
    gripper()   # build: writes the sibling gripper.step
