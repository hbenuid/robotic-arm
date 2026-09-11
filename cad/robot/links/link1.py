"""link1 - the rigid link as a cadgen model in the LINK frame (robot/_links.build_link):
running this file writes robot/links/link1.step (git-ignored). The URDF meshes are NOT this
STEP: tools/robot/export_link_meshes.py writes robot/meshes/link1.stl from the same builder."""
from cadgen import step
from robot._links import build_link


@step
def link1():
    return build_link("link1")


if __name__ == "__main__":
    link1()
