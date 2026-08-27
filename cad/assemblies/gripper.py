"""Gripper mechanism - the 'Gripper Mechanism' SolidWorks sub-assembly (19 occurrences),
built in its own frame; assemblies/arm.py places the whole module at "gripper#1".

Run:  ./cadtool step assemblies/gripper.py      -> assemblies/gripper.step (git-ignored)
      ./cadtool python -m assemblies.gripper    -> preview in the OCP CAD Viewer
"""
# --- path shim: files inside assemblies/ -> parent.parent (= cad/) --------------------
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from assemblies._occurrences import add_occurrences  # noqa: E402
from lib.assembly import AssemblyHelper  # noqa: E402

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


def gen_step():
    """The gripper module in its own (SolidWorks sub-assembly) frame."""
    asm = AssemblyHelper("gripper")
    add_occurrences(asm, OCCURRENCES)
    return asm.build()


if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
