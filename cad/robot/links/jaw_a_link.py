"""jaw_a_link - the rigid link as a cadgen model in the LINK frame (robot/_links.build_link):
running this file writes robot/links/jaw_a_link.step (git-ignored). The URDF meshes are NOT this
STEP: tools/robot/export_link_meshes.py writes robot/meshes/jaw_a_link.stl from the same builder."""
from cadgen import step
from robot._links import build_link


@step
def jaw_a_link():
    return build_link("jaw_a_link")


if __name__ == "__main__":
    jaw_a_link()
