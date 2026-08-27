"""Every custom part must match its SolidWorks reference geometry.

Wrappers pass trivially (they ARE the reference); a converted part is gated by volume and
bounding box (size + position in its local frame, after LOCAL_FROM_REF) so a conversion
that drifts from the original design fails here. Per-part tolerances: REF_VOL_TOL
(relative) and REF_BBOX_TOL (mm) module attributes.
"""
import hashlib
import importlib
import json
import pkgutil

import pytest
from build123d import Location

import parts
from lib import reference as R

CUSTOM_PARTS = sorted(
    mi.name for mi in pkgutil.iter_modules(parts.__path__) if not mi.name.startswith("_") and mi.name in R.CUSTOM
)


def _sha256(path):
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def test_reference_and_vendor_files_match_manifest():
    """reference/*.step and vendor/*.step are immutable inputs - detect edits/corruption."""
    manifest = json.loads((R.REF_DIR / "manifest.json").read_text(encoding="utf-8"))
    for name, entry in manifest["parts"].items():
        path = (R.REF_DIR if entry["kind"] == "reference" else R.VENDOR_DIR) / f"{name}.step"
        assert path.exists(), f"{path} listed in manifest.json but missing"
        assert _sha256(path) == entry["sha256"], f"{path} differs from reference/manifest.json"
    assert set(manifest["parts"]) == set(R.CUSTOM) | set(R.COTS)


@pytest.mark.slow
@pytest.mark.parametrize("name", CUSTOM_PARTS)
def test_part_matches_reference(name):
    mod = importlib.import_module(f"parts.{name}")
    ok, report = R.matches_reference(
        mod.gen_step(),
        mod.REFERENCE,
        local_from_ref=getattr(mod, "LOCAL_FROM_REF", None) or Location(),
        vol_tol=getattr(mod, "REF_VOL_TOL", 0.005),
        bbox_tol=getattr(mod, "REF_BBOX_TOL", 0.2),
    )
    assert ok, f"{name} deviates from reference/{mod.REFERENCE}.step (got, reference): {report}"
