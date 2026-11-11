"""elbow_home_clip - the printed clip of the elbow_pitch home sensor (parametric build123d, lib/upper_arm/home.py
build_home_clip): a C over the upper arm's slab at its end flat, outside the elbow belt, it holds the KY-003 hall module
(ky003_hall_sensor) under the elbow 90T's outer face, facing a magnet glued on that face (elbow_home_magnet).

The spine down the end flat, the top arm on the slab's top face (under the roll frame, which turns over it), the jaw
under its underside short of the 90T's rim; below the pulley's face the spine turns in as a web to the board's plate,
the board on it chip up on 2 M2 (elbow_home_board_screws, elbow_home_board_nuts) through its own holes, the heads under
the plate; relief slots under the header's pins and the chip's leads. Nothing on the arm changes: held by its fit and a
dab of glue (the user's choice over screws). Every number: lib/upper_arm/params.py ElbowHomeParams. Frame
(lib/upper_arm/layout.py home_clip_frame()): j1_link's axes, its origin on the end flat at the slab's underside and the
clip's near side. Prints on its side - its -Z face on the bed, the C's profile flat, the M2 holes teardrops.

In the arm: x1 (elbow_home_clip#1, a mount on j1_link#1 - lib/mounts.py ELBOW_HOME_MOUNTS).

No SolidWorks export and NO reference file: designed here, its own test locks its numbers -
tests/upper_arm/test_elbow_home.py.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.upper_arm import DEFAULT
from lib.upper_arm.home import build_home_clip

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/upper_arm/test_elbow_home.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (home_clip_frame())


@step(out="elbow_home_clip.step")
def elbow_home_clip():
    """The clip at its local origin; the assembly owns placement."""
    part = build_home_clip(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    elbow_home_clip()   # build: writes the sibling elbow_home_clip.step
