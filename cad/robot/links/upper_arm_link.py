"""upper_arm_link - the rigid link as a cad model in the LINK frame (robot/_links.build_link):
running this file writes robot/links/upper_arm_link.step (git-ignored). The URDF meshes are NOT this
STEP: tools/robot/export_link_meshes.py writes robot/meshes/upper_arm_link.stl from the same builder."""
from cad import step

from robot._links import build_link


@step(out="upper_arm_link.step")
def upper_arm_link():
    return build_link("upper_arm_link")


if __name__ == "__main__":
    upper_arm_link()
