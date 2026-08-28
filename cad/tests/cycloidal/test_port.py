"""The ported (designed) cycloidal-drive parts reproduce their CadQuery exports - stricter than
the house reference-match: identical face sets and identical tessellations, plus the exact
analytic volume for the parts whose faces OCCT integrates reliably (everything but the two
spline discs, whose analytic volume wobbles by ~0.3 % on both sides of the comparison)."""
import json

import pytest

import parts
from lib import reference as R
from tests.cycloidal.helpers import fingerprint, mesh_volume
from tools.cycloidal import export_cadquery as EX     # stdlib-only at module level (runs in the CadQuery venv)

DESIGNED = sorted(R.DESIGNED)
SPLINE_PARTS = {"cycloidal_disc_1", "cycloidal_disc_2"}


@pytest.mark.slow
@pytest.mark.parametrize("name", DESIGNED)
def test_designed_part_reproduces_cadquery_export(name):
    mod = parts.load(name)
    mine, ref = mod.gen_step(), R.load(name)
    assert all(abs(a - b) <= 0.01 for a, b in zip(R.bbox_min(mine), R.bbox_min(ref))), (R.bbox_min(mine), R.bbox_min(ref))
    assert all(abs(a - b) <= 0.01 for a, b in zip(R.bbox_size(mine), R.bbox_size(ref))), (R.bbox_size(mine), R.bbox_size(ref))
    fm, fr = fingerprint(mine), fingerprint(ref)
    assert len(fm) == len(fr), f"{name}: {len(fm)} faces vs {len(fr)} in the reference"
    for (tm, am), (tr, ar) in zip(fm, fr):
        assert tm == tr and abs(am - ar) <= 1e-3 * max(1.0, abs(ar)), f"{name}: face {tm} {am} vs {tr} {ar}"
    vm, cm, nm = mesh_volume(mine)
    vr, cr, nr = mesh_volume(ref)
    assert nm == nr, f"{name}: tessellation differs ({nm} vs {nr} triangles)"
    assert abs(vm - vr) <= 1e-6 * abs(vr), f"{name}: mesh volume {vm} vs {vr}"
    assert all(abs(a - b) <= 1e-3 for a, b in zip(cm, cr)), f"{name}: centroid {cm} vs {cr}"
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
