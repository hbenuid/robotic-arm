"""Shared body of the cycloidal drive's purchased-part modules (parts/bearing_*.py,
nema17_48mm.py, cycloidal_*_pins/bolts/nuts.py) - NOT a part (underscore: not discovered).

Each module keeps the house COTS contract (COTS, MASS_G, VENDOR_STEP, VENDOR_TO_REF, _envelope,
gen_step); the geometry of the envelopes is the drive repo's simplified purchased-part model
(src/purchased_parts.py at cycloidal_drive@2f1f67d), which is also the reference STEP.
"""
from __future__ import annotations

from build123d import Compound, Location, import_step


def hybrid(name: str, vendor_step, vendor_to_ref: Location, envelope):
    """Vendor geometry if vendor/<name>.step exists, else the envelope - labelled."""
    part = import_step(str(vendor_step)).moved(vendor_to_ref) if vendor_step.exists() else envelope()
    part.label = name
    return part


def pattern(solids) -> Compound:
    """A multi-body pattern part (pins, bolts, nuts) as one compound of separate solids."""
    return Compound([s for shape in solids for s in shape.solids()])
