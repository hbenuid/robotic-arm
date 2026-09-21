"""The arm with its three covers left off - a WORKING VIEW for the links the caps sit on (j1_cap
over j1_link, j2_cap_1 / j2_cap_2 over j2_link), NOT the robot: assemblies/arm.py stays the
reference-matched assembly that the totals, robot/ and the URDF describe.

The tables are arm.py's minus HIDDEN (never retyped, so a part added to or moved in the arm shows up
here too); same base_link frame (arm.arm_from_w()), same GROUPS component tree and tints. A cadgen
model takes no parameters and the freshness gate does not see environment variables, so "the arm
without its caps" is a second model with its own STEP rather than a switch in arm().

The links are shared part models: an edit to parts/<group>/<link>.py reaches both arms on their own
next gen. The caps are SolidWorks wrappers and do not follow a link change, and this view cannot show
a cap that no longer fits - after changing j1_link / j2_link, rebuild and check assemblies/arm.py too.

Run:  ./cadtool gen assemblies/arm_no_caps.py    -> assemblies/arm_no_caps.step (git-ignored)
      ./cadtool show assemblies/arm_no_caps.py   -> preview in the OCP CAD Viewer (no build)
      ./cadtool viewer                           -> http://127.0.0.1:3245/?file=assemblies/arm_no_caps.step
"""

from cadgen import step

from assemblies import arm as full
from assemblies._occurrences import grouped_children
from lib.assembly import assembly

HIDDEN = ("j1_cap#1", "j2_cap_1#1", "j2_cap_2#1")   # placements.json keys left out of this view

OCCURRENCES = [row for row in full.OCCURRENCES if row[2] not in HIDDEN]
GROUPS = [(label, tint, tuple(key for key in keys if key not in HIDDEN)) for label, tint, keys in full.GROUPS]


@step
def arm_no_caps():
    """arm.arm() without the HIDDEN occurrences: 'arm_no_caps' -> the same GROUPS component nodes,
    tints and base_link frame."""
    return assembly("arm_no_caps", grouped_children(OCCURRENCES, GROUPS, full.MODULES, full.MODULE_TINTS,
                                                    root=full.arm_from_w()))


if __name__ == "__main__":
    arm_no_caps()   # build: writes the sibling arm_no_caps.step (preview: ./cadtool show assemblies/arm_no_caps.py)
