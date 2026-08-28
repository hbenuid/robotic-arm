"""Export the cycloidal_drive repo's CadQuery builders as house-named STEP files - the reference
geometry the build123d port is verified against (reference/<name>.step via
tools/cycloidal/import_reference.py).

RUNS IN THE cycloidal_drive REPO'S OWN VENV (CadQuery), never in cad/'s:

    cd ../cycloidal_drive && uv run python ../robotic-arm/cad/tools/cycloidal/export_cadquery.py [--out export/step/house]

Writes <out>/<name>.step for every entry of BUILDERS plus <out>/manifest.json (git rev, cadquery
version, per-part builder label / solids / volume). `export/` is git-ignored in that repo, so
this never changes it. The name map must equal lib/reference.py's DESIGNED | CYCLOIDAL_COTS
(tests/test_cycloidal_port.py checks the two agree) - keep them in step.
"""
from __future__ import annotations

import argparse
import datetime
import importlib
import json
import os
import pathlib
import subprocess
import sys

# house name -> (module, builder, kwargs); "disc2_phase" is resolved from the config at run time.
BUILDERS: dict[str, tuple[str, str, dict]] = {
    "cycloidal_disc_1":            ("src.cycloidal_disc",  "build_cycloidal_disc", {}),
    "cycloidal_disc_2":            ("src.cycloidal_disc",  "build_cycloidal_disc", {"phase_offset_deg": "disc2_phase"}),
    "cycloidal_eccentric_shaft":   ("src.eccentric_shaft", "build_eccentric_shaft", {}),
    "cycloidal_motor_plate":       ("src.motor_plate",     "build_motor_plate", {}),
    "cycloidal_ring_gear_body":    ("src.ring_gear_body",  "build_ring_gear_body", {}),
    "cycloidal_output_hub":        ("src.output_hub",      "build_output_hub", {}),
    "bearing_6003":                ("src.purchased_parts", "build_bearing_6003", {}),
    "bearing_6814":                ("src.purchased_parts", "build_bearing_6814", {}),
    "bearing_625":                 ("src.purchased_parts", "build_bearing_625", {}),
    "nema17_48mm":                 ("src.purchased_parts", "build_nema17_motor", {}),
    "cycloidal_ring_pins":         ("src.purchased_parts", "build_ring_pins", {}),
    "cycloidal_output_pins":       ("src.purchased_parts", "build_output_pins", {}),
    "cycloidal_shaft_support_pin": ("src.purchased_parts", "build_shaft_support_pin", {}),
    "cycloidal_motor_bolts":       ("src.purchased_parts", "build_motor_bolts", {}),
    "cycloidal_housing_bolts":     ("src.purchased_parts", "build_housing_bolts", {}),
    "cycloidal_housing_nuts":      ("src.purchased_parts", "build_housing_nuts", {}),
}


def builder_label(name: str) -> str:
    """`src/purchased_parts.py:build_bearing_6003()` - the label lib/reference.py records."""
    module, func, kwargs = BUILDERS[name]
    args = ", ".join(f"{k}={v}" for k, v in kwargs.items())
    return f"{module.replace('.', '/')}.py:{func}({args})"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", type=pathlib.Path, default=pathlib.Path("export/step/house"))
    ap.add_argument("--only", nargs="*", default=None, help="subset of house names")
    args = ap.parse_args(argv)

    root = pathlib.Path(os.getcwd())
    if not (root / "src" / "params.py").exists():
        print(f"run from the cycloidal_drive repo root (no src/params.py in {root})", file=sys.stderr)
        return 1
    sys.path.insert(0, str(root))
    import cadquery as cq  # noqa: E402  (the OLD venv)
    from src.params import DEFAULT_CONFIG  # noqa: E402

    rev = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=root).stdout.strip()
    dirty = subprocess.run(["git", "status", "--porcelain", "--", "src", "assembly.py", "export.py"],
                           capture_output=True, text=True, cwd=root).stdout.strip()
    args.out.mkdir(parents=True, exist_ok=True)
    manifest = {
        "repo": "cycloidal_drive", "rev": rev, "dirty": bool(dirty), "cadquery": cq.__version__,
        "exported": datetime.date.today().isoformat(), "parts": {},
    }
    for name, (module, func, kwargs) in BUILDERS.items():
        if args.only and name not in args.only:
            continue
        fn = getattr(importlib.import_module(module), func)
        kw = {k: (DEFAULT_CONFIG.gear.disc2_phase_deg if v == "disc2_phase" else v) for k, v in kwargs.items()}
        shape = fn(**kw)
        path = args.out / f"{name}.step"
        cq.exporters.export(shape, str(path))
        solids = shape.val().Solids() if hasattr(shape, "val") else shape.Solids()
        manifest["parts"][name] = {
            "builder": builder_label(name), "file": path.name, "solids": len(solids),
            "solid_volume": round(sum(s.Volume() for s in solids), 3),
        }
        print(f"  {name:28s} {manifest['parts'][name]['solids']:2d} solid(s)  {manifest['parts'][name]['solid_volume']:12.3f} mm^3  -> {path}")
    (args.out / "manifest.json").write_text(json.dumps(manifest, indent=1, sort_keys=True) + "\n")
    print(f"wrote {args.out / 'manifest.json'} (rev {rev}{' DIRTY' if dirty else ''})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
