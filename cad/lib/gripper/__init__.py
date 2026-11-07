"""The gripper: PapaBravo's rack & pinion gripper (Printables, CC BY 4.0) - its four measured parts (gripper_base,
gripper_rack, gripper_jaw, gripper_pinion) as parametric build123d.

Pure-Python config here (lib/gripper/params.py); the build123d builders in lib/gripper/body.py (imported explicitly by
their users, so importing this package never pulls in OCCT). Never imports lib/params.py.
"""
from lib.gripper.params import DEFAULT, BaseParams, GripperConfig, JawParams, PinionParams, RackParams  # noqa: F401
