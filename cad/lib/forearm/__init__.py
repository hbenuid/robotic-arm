"""The forearm (j2_link + its two caps) as parametric build123d, and the forearm roll drive that splits it.

Pure-Python config and layout here (lib/forearm/params.py, layout.py); the build123d builders in
lib/forearm/link.py and caps.py (imported explicitly by the parts that need them, so importing this package
never pulls in OCCT). Never imports lib/params.py - that module re-exports the interface values from here.
"""
from lib.forearm.layout import (  # noqa: F401
    cap1_socket_points, cap2_socket_points, disc_bolt_angles, disc_bolt_points, link_socket_points, motor_window,
)
from lib.forearm.params import (  # noqa: F401
    DEFAULT, LEGACY, Cap1Params, Cap2Params, ElbowDiscParams, ForearmConfig, SlotParams, SocketParams,
    WebParams, WristBossParams,
)
