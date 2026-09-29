"""Every custom part must match its reference geometry (the SolidWorks export, for the designed
cycloidal-drive parts the CadQuery export they were ported from, for a native part its accepted build).

A converted part - and a native one - is gated by volume and bounding box (size + position in its local
frame, after LOCAL_FROM_REF) so a conversion that drifts from the original design fails here. Two kinds are not
matched here: a wrapper IS its reference (its body loads that file), and the designed cycloidal-drive parts are
matched face for face by tests/cycloidal/test_port.py, stricter than this. Per-part tolerances: REF_VOL_TOL
(relative) and REF_BBOX_TOL (mm) module attributes. A converted part whose DEFAULT build deliberately
diverges from its reference (the forearm parts: the SolidWorks elbow end gave way to the roll joint)
declares REFERENCE_BUILD, a zero-arg callable returning the LEGACY configuration that reproduces the
reference - that build is matched here, the default one is locked by the part's own tests. A measured conversion
(lib/reference.py MEASURED) has no reference: its own tests hold the numbers measured on its export.
"""
import pytest

import parts
from lib import manifest as M
from lib import reference as R
from lib.datum import IDENTITY, to_location
from tests import built

CUSTOM_PARTS = [n for n in parts.names() if n in R.CUSTOM or n in R.DESIGNED or n in R.NATIVE]
MATCHED = [n for n in CUSTOM_PARTS if n in R.NATIVE or (n in R.CUSTOM and parts.load(n).CONVERTED)]


def test_some_custom_parts_were_discovered():
    """An empty parametrize list would only *skip* the reference match - guard the discovery."""
    assert len(CUSTOM_PARTS) == len(R.CUSTOM) + len(R.DESIGNED) + len(R.NATIVE)
    assert MATCHED


def test_reference_and_vendor_files_match_manifest():
    """reference/{solidworks,cycloidal}/*.step are immutable inputs and vendor/*.step must match what the manifest
    describes - detect edits/corruption or a vendor swap without re-running the import tools."""
    manifest = M.read()
    assert manifest["parts"], f"{M.MANIFEST_PATH} is missing or empty - run the import tools"
    for name, entry in manifest["parts"].items():
        ref = R.path_of(name)
        assert ref.exists(), f"{ref} listed in manifest.json but missing"
        assert entry.get("file") == ref.relative_to(R.REF_DIR).as_posix(), (
            f"{name}: manifest 'file' {entry.get('file')!r} != {ref.relative_to(R.REF_DIR).as_posix()!r}"
        )
        assert R.sha256(ref) == entry["sha256"], f"{ref} differs from reference/manifest.json"
        if entry["kind"] == "cots":
            vendor = R.VENDOR_DIR / f"{name}.step"
            if "vendor" in entry:
                assert vendor.exists(), f"{vendor} missing"
                assert R.sha256(vendor) == entry["vendor"]["sha256"], (
                    f"{vendor} differs from manifest.json - run tools/reference/import_solidworks.py "
                    f"(tools/cycloidal/import_cadquery.py for the drive) after replacing a vendor file"
                )
            else:   # envelope in use - a vendor file must not appear without being recorded
                assert not vendor.exists(), f"{vendor} exists but manifest.json has no vendor entry - re-run the import tool"
    assert set(manifest["parts"]) == set(R.CUSTOM) | set(R.COTS) | set(R.DESIGNED) | set(R.NATIVE)


@pytest.mark.slow
@pytest.mark.parametrize("name", MATCHED)
def test_part_matches_reference(name):
    mod = parts.load(name)
    ok, report = R.matches_reference(
        built.legacy(name) if hasattr(mod, "REFERENCE_BUILD") else built.part(name),   # a diverged conversion: its LEGACY build
        mod.REFERENCE,
        local_from_ref=to_location(getattr(mod, "LOCAL_FROM_REF", IDENTITY)),
        vol_tol=getattr(mod, "REF_VOL_TOL", 0.005),
        bbox_tol=getattr(mod, "REF_BBOX_TOL", 0.2),
    )
    assert ok, f"{name} deviates from reference/{mod.REFERENCE}.step (got, reference): {report}"
