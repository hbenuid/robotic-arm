"""wrist_link - the wrist body, the wrist_pitch stage and the wrist roll drive's frame (parametric build123d,
lib/wrist/link.py build_wrist_link(cfg), in the SolidWorks part frame - origin on the plate's underside at the centre
of the SolidWorks tower's round end, +Z up through the plate; the pitch axis along Z at x -45.446).

DEFAULT (what the part builds): a 5 mm plate, its round end on the pitch axis j3_coupler#2's seat (a Ø29.9 hole clear
of the wrist 90T's nuts, the flange's 4x M4 at r 35 on the axes into hex nut pockets in the underside), and on its
underside - the side away from the forearm - the wrist roll drive's frame (lib/wrist/roll.py build_frame,
WristRollParams; docs/wrist_roll.md), laid out as the forearm roll's frame: seen along the pitch axis the plate's
octagon hulled to the tower round the roll axis, WRIST_ROLL_OFFSET across, from the plate to the closed far face, its
rear one slanted plane, the front face's long edges faceted - one body round the pancake, centred on the pitch axis in
its pocket (open down through the bottom - the frame's end beyond the motor, where it slides in - its front wall the
plate it bolts to, slotted for the belt's tension, a groove inside the side wall for its plug), its MKS board in a
window open at the rear and the bottom; the tower: the 6806 pair on a lip, the bearing boss rising out of the front
facets, the shaft's bay with the hard stop's post bored on through the rear under a removable back panel
(wrist_back_panel, held by wrist_back_magnets); the coupler nut the tower covers drops in through a hex channel from
the bay. LEGACY (REFERENCE_BUILD) is the
SolidWorks part: the plate with a round end on each axis, the tower on the other - an R39 wall round an R30 bore, a
wedge under a slope inside it (its edge with the bore filleted R10), cheeks either side of a 42 mm slot, a 64 x 42
block out to the end face the dropped gripper_clamp_bracket bolted to. Every number: lib/wrist/params.py (WristConfig;
the SolidWorks part measured on the reference 2026-09-27).

SolidWorks product: 'final component arm qwrist movement'
Source export:      step/final component arm qwrist movement.STEP
Reference: mm units, 1 solid(s), volume 113601.4 mm^3,
           bbox size (124.446, 78, 44) mm, bbox min (-84.446, -39, -0) mm.
In the arm: x1 (wrist_link#1).

Conversion: build_wrist_link(LEGACY) reproduces the reference (REFERENCE_BUILD - tests/test_reference_match.py); the
model builds DEFAULT (tests/wrist/ lock the features).
"""
import pathlib

from cadgen import step

from lib.datum import IDENTITY
from lib.wrist import DEFAULT, LEGACY
from lib.wrist.link import build_wrist_link

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in the SolidWorks part frame


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_wrist_link(LEGACY)


@step(out="wrist_link.step")
def wrist_link():
    """The wrist body at its local origin (the plate's underside, the tower's axis); the assembly owns placement."""
    part = build_wrist_link(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_link()   # build: writes the sibling wrist_link.step
