"""j2_cap_2 - the belt tray under the forearm web (parametric build123d, lib/forearm/caps.py build_cap_2(cfg),
modelled in j2_link's frame: rim on the web's bottom face z = 8, outer face z = -5.5).

A 1.5 mm floor under a 12 mm pocket (|y| < 35, wrapping the wrist axis, open toward the elbow - only the floor lip
closes it there) in which the wrist-pitch belt runs from the motor's 20T to the wrist 90T; an arc channel about the
elbow axis (r 70.75..93.25, 10 deep from the outer face, through floor and rims - purpose unknown, it clears nothing
of j1_link); eight Ø5.18 x 2 sockets mirroring the web's. The outline stops at a concave arc round the elbow disc.
Every number: lib/forearm/params.py (Cap2Params).

SolidWorks product: 'cap of joint 2 piece 2 8526'
Source export:      step/cap of joint 2 piece 2 8526.STEP
Reference: mm units, 1 solid(s), volume 75151.4 mm^3,
           bbox size (223.377, 90, 13.5) mm, bbox min (-753.703, 885.488, -13.5) mm - modelled in assembly context
           ~1 m from its part origin and upside down; LOCAL_FROM_REF (from placements.json) maps it under the web.
In the arm: x1 (j2_cap_2#1).

Conversion: build_cap_2(LEGACY) reproduces the reference (REFERENCE_BUILD); the model builds DEFAULT.
"""
import pathlib

from cadgen import step

from lib.forearm import DEFAULT, LEGACY
from lib.forearm.caps import build_cap_2

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = ((-785.325366, -930.487557, -5.5), (-180.0, 0.0, 180.0))   # SolidWorks part frame -> j2_link's [REFERENCE]
REF_BBOX_TOL = 0.02


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_cap_2(LEGACY)


@step
def j2_cap_2():
    """The belt tray in j2_link's frame; the assembly owns placement (through LOCAL_FROM_REF)."""
    part = build_cap_2(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j2_cap_2()   # build: writes the sibling j2_cap_2.step
