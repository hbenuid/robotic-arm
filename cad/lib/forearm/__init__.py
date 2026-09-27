"""The forearm (j2_link) as parametric build123d, and the forearm roll drive that splits it.

Pure-Python config and layout here (lib/forearm/params.py, layout.py); the build123d builders in
lib/forearm/link.py and roll.py (imported explicitly by the parts that need them, so importing this package
never pulls in OCCT). Never imports lib/params.py - that module re-exports the interface values from here.
"""
from lib.forearm.layout import (  # noqa: F401
    belt_window, cap_bolt_points, coupler_steps, disc_bolt_angles,
    disc_bolt_points, elbow_end_x, flange_bolt_points, flange_bolt_points_module, link_socket_points,
    module_frame_in_host, mount_bolt_points, mount_nut_pocket_open_y, nut_channel_end, pad_bolt_points, pulley_bolt_points, stack_positions,
)
from lib.forearm.params import (  # noqa: F401
    DEFAULT, LEGACY, ElbowDiscParams, ForearmConfig, RollDriveParams, RollEndParams,
    SlotParams, SocketParams, WebParams, WristBossParams,
)
