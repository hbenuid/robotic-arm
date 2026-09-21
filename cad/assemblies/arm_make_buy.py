"""The arm sorted by where each part comes from - a WORKING VIEW, NOT the robot: assemblies/arm.py stays
the reference-matched assembly that the totals, robot/ and the URDF describe.

    arm_make_buy
    |- printed   the 3D-printed parts             (MAKE_BUY_TINTS["printed"])
    |- bought    the purchased parts, COTS = True (MAKE_BUY_TINTS["bought"])

Same parts, same poses and the same base_link frame (Z up) as arm.step, but the tree is the make/buy
label (parts.bought()) instead of the links, flattened: the gripper's and the drive's parts sit directly
under the two nodes, so hiding 'bought' in a viewer leaves exactly what has to be printed. The rows are
arm.py's tables expanded (never retyped): a part added to the arm, the gripper or the drive shows up
here too. A cadgen model takes no parameters and the freshness gate does not see environment variables,
so this is a second model with its own STEP rather than a switch in arm(). The lists behind the two
nodes: ./cadtool python tools/bom.py.

Run:  ./cadtool gen assemblies/arm_make_buy.py    -> assemblies/arm_make_buy.step (git-ignored)
      ./cadtool show assemblies/arm_make_buy.py   -> preview in the OCP CAD Viewer (no build)
      ./cadtool viewer                            -> http://127.0.0.1:3245/?file=assemblies/arm_make_buy.step
"""

from cadgen import step

from assemblies import arm as full
from assemblies._occurrences import make_buy_children, module_rows, world_rows
from lib import placements as P
from lib.assembly import assembly
from lib.datum import base_frame


def _rows() -> list:
    """Every leaf of the arm as (part, role, WORLD placement): arm.OCCURRENCES with the module rows
    expanded - the code-driven drive through world_rows(), the SolidWorks-driven gripper through its
    own table's world placements."""
    rows = []
    for name, role, key in full.OCCURRENCES:
        if name not in full.MODULES:
            rows.append((name, role, P.location(key, "world")))
        elif P.OCCURRENCES[key].get("designed"):
            rows += world_rows(key)
        else:
            rows += [(part, part_role, P.location(part_key, "world")) for part, part_role, part_key in module_rows(name)]
    return rows


@step
def arm_make_buy():
    """arm.arm()'s leaves under the 'printed' / 'bought' nodes, tinted, in the base_link frame."""
    return assembly("arm_make_buy", make_buy_children(_rows(), into=base_frame()))


if __name__ == "__main__":
    arm_make_buy()   # build: writes the sibling arm_make_buy.step (preview: ./cadtool show assemblies/arm_make_buy.py)
