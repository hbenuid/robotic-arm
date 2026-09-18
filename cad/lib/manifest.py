"""reference/manifest.json - what every reference (and vendor) STEP is: kind, origin, checksum,
units and geometry facts.

Two tools write it and each owns its entries, keeping the other's untouched:
tools/reference/import_solidworks.py (the SolidWorks exports: kinds "custom" / "cots") and
tools/cycloidal/import_cadquery.py (the cycloidal drive's CadQuery exports: "designed" / "cots").
tests/test_reference_match.py and tests/test_parts_convention.py read it to detect an edited
reference or a swapped vendor file. No part imports this module, so editing it never makes a
part stale.
"""
from __future__ import annotations

import json
import pathlib

from lib import reference as R

MANIFEST_PATH = R.REF_DIR / "manifest.json"


def read() -> dict:
    """The manifest as a dict ({"parts": {}} while the file does not exist yet)."""
    if not MANIFEST_PATH.exists():
        return {"parts": {}}
    return json.loads(MANIFEST_PATH.read_text(encoding="utf-8"))


def write(manifest: dict) -> None:
    """Write the manifest deterministically (sorted keys, one trailing newline)."""
    MANIFEST_PATH.write_text(json.dumps(manifest, indent=1, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")


def entry(kind: str, ref: pathlib.Path, *, product: str, source, vendor: pathlib.Path | None = None, **extra) -> dict:
    """The manifest entry of one part: its reference STEP `ref` (under reference/) and, for a
    purchased part with a vendor model, the `vendor` block describing the current vendor/ file.
    `extra` carries tool-specific fields (the drive's `origin`)."""
    record = {
        "kind": kind,
        "file": ref.relative_to(R.REF_DIR).as_posix(),
        "product": product,
        "source": source,
        "bytes": ref.stat().st_size,
        "sha256": R.sha256(ref),
        "units": R.step_units(ref),
        **R.describe(ref),
        **extra,
    }
    if vendor is not None:
        vendor_sha = R.sha256(vendor)
        record["vendor"] = {
            "bytes": vendor.stat().st_size,
            "sha256": vendor_sha,
            "same_as_reference": vendor_sha == record["sha256"],
            **R.describe(vendor),
        }
    return record
