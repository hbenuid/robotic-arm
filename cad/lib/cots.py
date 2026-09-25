"""The body every purchased (COTS) part module builds through: the vendor STEP when vendor/<name>.step exists,
else the module's envelope (parts/_templates/cots.py, parts/CLAUDE.md Purchased (COTS) parts).

Each module keeps the house COTS contract (COTS, MASS_G, VENDOR_STEP, VENDOR_TO_REF, _envelope, the `@step`
model) and returns hybrid(NAME, VENDOR_STEP, VENDOR_TO_REF, _envelope). pattern() is the multi-body form of the
drive's pins, bolts and nuts.
"""
from __future__ import annotations

from cadgen import build123d as bd
from cadgen import read_step

from lib.datum import to_location


def hybrid(name: str, vendor_step, vendor_to_ref, envelope):
    """Vendor geometry if vendor/<name>.step exists (moved by the VENDOR_TO_REF frame data), else the
    envelope - labelled."""
    part = read_step(vendor_step).moved(to_location(vendor_to_ref)) if vendor_step.exists() else envelope()
    part.label = name
    return part


def pattern(solids) -> bd.Compound:
    """A multi-body pattern part (pins, bolts, nuts) as one compound of separate solids."""
    return bd.Compound([s for shape in solids for s in shape.solids()])
