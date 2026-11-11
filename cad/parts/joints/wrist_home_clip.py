"""wrist_home_clip - the printed clip of the wrist_pitch home sensor (parametric build123d, lib/forearm/home.py
build_home_clip): the elbow's clip (elbow_home_clip) on the forearm's wrist end, outside the wrist belt; it holds the
KY-003 hall module (ky003_hall_sensor) under the wrist 90T's outer face, facing a magnet glued on that face
(wrist_home_magnet).

No top arm (the elbow's has one): the wrist frame turns 2 mm over the wrist boss and sweeps right round it. Instead a cap
round the boss's faceted end - its end flat and the two facets beside it, which place it in plan -, a jaw under the
boss short of the 90T's rim, the spine on down the end flat; below the pulley's face the spine turns in as a web to the
board's plate, the board on it chip up on 2 M2 (wrist_home_board_screws, wrist_home_board_nuts) through its own holes, the
heads under the plate; relief slots under the header's pins and the chip's leads. Nothing on the arm changes: held by
glue alone (the user's choice over screws). Every number: lib/forearm/params.py WristHomeParams. Frame
(lib/forearm/layout.py home_clip_frame()): its origin on its outer face over the end flat at the boss's underside, +Z
along j2_link's +x. Prints on that face - the cap's facet walls at 45 deg, the M2 holes teardrops.

In the arm: x1 (wrist_home_clip#1, a mount on j2_link#1 - lib/mounts.py WRIST_HOME_MOUNTS).

No SolidWorks export and NO reference file: designed here, its own test locks its numbers -
tests/forearm/test_wrist_home.py.
"""
import pathlib

from cad import step

from lib.datum import IDENTITY
from lib.forearm import DEFAULT
from lib.forearm.home import build_home_clip

NAME = pathlib.Path(__file__).stem
REFERENCE = None              # no reference file: tests/forearm/test_wrist_home.py holds its numbers
CONVERTED = True
LOCAL_FROM_REF = IDENTITY     # modelled in its own frame (home_clip_frame())


@step(out="wrist_home_clip.step")
def wrist_home_clip():
    """The clip at its local origin; the assembly owns placement."""
    part = build_home_clip(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    wrist_home_clip()   # build: writes the sibling wrist_home_clip.step
