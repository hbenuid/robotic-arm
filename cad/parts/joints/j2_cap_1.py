"""j2_cap_1 - the lid over the forearm web's motor side (parametric build123d, lib/forearm/caps.py build_cap_1(cfg),
modelled in j2_link's frame: rim on the web's top face z = 19, lid outer face z = 33.5).

A 1.5 mm lid over a 13 mm pocket (|y| < 35, wrapping the elbow axis, ending at an arc about the wrist axis); the
rim covers the elbow disc's nut pockets and the web's edges; the 60 x 48 window over the wrist-pitch motor's body
(centred on lib/params.py J2_MOTOR_SLIDE_X); eight Ø5.18 x 2 sockets mirroring the web's. The outline stops at a
concave arc round the wrist boss. Every number: lib/forearm/params.py (Cap1Params).

SolidWorks product: 'cap 1 joint 2 8726'
Source export:      step/cap 1 joint 2 8726.STEP
Reference: mm units, 1 solid(s), volume 97819.5 mm^3,
           bbox size (245.461, 90, 14.5) mm, bbox min (-200.461, -45, -14.5) mm - the SolidWorks part frame has
           the lid's outer face at z = 0; LOCAL_FROM_REF lifts it onto the web (placements.json cross-checked).
In the arm: x1 (j2_cap_1#1).

Conversion: build_cap_1(LEGACY) reproduces the reference (REFERENCE_BUILD); the model builds DEFAULT.
"""
import pathlib

from cadgen import step
from lib.forearm import DEFAULT, LEGACY
from lib.forearm.caps import build_cap_1

NAME = pathlib.Path(__file__).stem
REFERENCE = NAME              # reference/solidworks/<NAME>.step
CONVERTED = True
LOCAL_FROM_REF = ((0.0, 0.0, 33.5), (0.0, 0.0, 0.0))   # SolidWorks part frame -> j2_link's frame [REFERENCE]
REF_BBOX_TOL = 0.02


def REFERENCE_BUILD():
    """The configuration that reproduces the SolidWorks part (the reference-match lock)."""
    return build_cap_1(LEGACY)


@step
def j2_cap_1():
    """The lid in j2_link's frame; the assembly owns placement (through LOCAL_FROM_REF)."""
    part = build_cap_1(DEFAULT)
    part.label = NAME
    return part


if __name__ == "__main__":
    j2_cap_1()   # build: writes the sibling j2_cap_1.step
