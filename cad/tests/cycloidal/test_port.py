"""The ported (designed) cycloidal-drive parts reproduce their CadQuery exports - stricter than
the house reference-match: identical face sets and identical tessellations, plus the exact
analytic volume for the parts whose faces OCCT integrates reliably (everything but the two
spline discs, whose analytic volume wobbles by ~0.3 % on both sides of the comparison, and whose
tessellation is platform-sensitive: they match the reference lobe spline point for point instead)."""
import json

import pytest

import parts
from lib import reference as R
from tests.cycloidal.helpers import fingerprint, mesh_volume, spline_deviation
from tools.cycloidal import export_cadquery as EX  # stdlib-only at module level (runs in the CadQuery venv)

DESIGNED = sorted(R.DESIGNED)
SPLINE_PARTS = {"cycloidal_disc_1", "cycloidal_disc_2"}


@pytest.mark.slow
@pytest.mark.parametrize("name", DESIGNED)
def test_designed_part_reproduces_cadquery_export(name):
    mine, ref = parts.build(name), R.load(name)
    assert all(abs(a - b) <= 0.01 for a, b in zip(R.bbox_min(mine), R.bbox_min(ref), strict=True)), (R.bbox_min(mine), R.bbox_min(ref))
    assert all(abs(a - b) <= 0.01 for a, b in zip(R.bbox_size(mine), R.bbox_size(ref), strict=True)), (R.bbox_size(mine), R.bbox_size(ref))
    fm, fr = fingerprint(mine), fingerprint(ref)
    assert len(fm) == len(fr), f"{name}: {len(fm)} faces vs {len(fr)} in the reference"
    for (tm, am), (tr, ar) in zip(fm, fr, strict=True):
        assert tm == tr and abs(am - ar) <= 1e-3 * max(1.0, abs(ar)), f"{name}: face {tm} {am} vs {tr} {ar}"
    vm, cm, nm = mesh_volume(mine)
    vr, cr, nr = mesh_volume(ref)
    if name in SPLINE_PARTS:
        # How the mesher discretises the lobe spline depends on the platform's floating point (arm64 macOS,
        # 2026-09-21: +-6 of ~22 000 triangles, mesh volume 4e-5, centroid 2e-3 mm, on a profile that matches
        # to 1e-11 mm). So the discs lock the profile itself and hold the mesh to the chordal error.
        deviation = spline_deviation(mine, ref)
        assert deviation <= 1e-6, f"{name}: lobe profile is {deviation:.3g} mm off the reference spline"
        tri_tol, vol_tol, centroid_tol = 1e-3, 1e-4, 5e-3
    else:
        tri_tol, vol_tol, centroid_tol = 0.0, 1e-6, 1e-3
    assert abs(nm - nr) <= tri_tol * nr, f"{name}: tessellation differs ({nm} vs {nr} triangles)"
    assert abs(vm - vr) <= vol_tol * abs(vr), f"{name}: mesh volume {vm} vs {vr}"
    assert all(abs(a - b) <= centroid_tol for a, b in zip(cm, cr, strict=True)), f"{name}: centroid {cm} vs {cr}"
    if name not in SPLINE_PARTS:
        vol, ref_vol = R.solid_volume(mine), R.solid_volume(ref)
        assert abs(vol - ref_vol) <= 1e-6 * ref_vol, f"{name}: volume {vol} vs {ref_vol}"


def test_manifest_records_the_cadquery_origin():
    manifest = json.loads((R.REF_DIR / "manifest.json").read_text(encoding="utf-8"))["parts"]
    for kind, registry in (("designed", R.DESIGNED), ("cots", R.CYCLOIDAL_COTS)):
        for name, builder in registry.items():
            entry = manifest[name]
            assert entry["kind"] == kind, name
            assert entry["origin"] == f"cycloidal_drive@{R.CYCLOIDAL_REV}", name
            assert entry["source"] == builder, name


def test_exporter_name_map_matches_registry():
    """tools/cycloidal/export_cadquery.py (run in the CadQuery venv) and lib/reference.py must
    name the same parts and builders."""
    expected = {**R.DESIGNED, **R.CYCLOIDAL_COTS}
    assert set(EX.BUILDERS) == set(expected), set(EX.BUILDERS) ^ set(expected)
    for name, label in expected.items():
        assert EX.builder_label(name) == label, name
