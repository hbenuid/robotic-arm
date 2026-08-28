"""Robot link 'link1' - its part occurrences in the link's own frame (see robot/frames.py).

    ./cadtool gen robot/links/link1.py            -> robot/links/link1.step (git-ignored review artifact)
    ./cadtool python tools/export_link_meshes.py   -> robot/meshes/link1.stl (used by robot/arm.urdf)
"""
import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[2]))

from robot._links import build_link  # noqa: E402

LINK = pathlib.Path(__file__).stem


def gen_step():
    return build_link(LINK)


if __name__ == "__main__":
    from ocp_vscode import show
    show(gen_step())
