"""wrist_roll_link - the rigid link as a cadgen model in the LINK frame (robot/_links.build_link):
running this file writes robot/links/wrist_roll_link.step (git-ignored). The URDF meshes are NOT this
STEP: tools/robot/export_link_meshes.py writes robot/meshes/wrist_roll_link.stl from the same builder."""
from cadgen import step

from robot._links import build_link


@step
def wrist_roll_link():
    return build_link("wrist_roll_link")


if __name__ == "__main__":
    wrist_roll_link()
