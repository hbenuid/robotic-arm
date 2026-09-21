"""Export print/<name>.stl for every PRINTED part (parts.bought(name) is False - no COTS = True).

    ./cadtool python tools/export_printables.py [--parts cycloidal_disc_1 j1_link ...] [--tolerance 0.01] [--angular 0.1]

One binary STL per part, in MILLIMETRES, in the part's own local frame: orienting it on the bed is the
slicer's job (print notes - "print output-face-down" - live in each part's docstring). How many of each to
print is the print list of tools/bom.py, repeated at the end. The body is built in-process
(parts.build(name)): no STEP is written, no cadgen build starts. print/ is git-ignored (the root *.stl
rule) - the files are regenerable; an STL here whose part is gone or is now bought is deleted. The
tolerances are finer than the URDF link meshes' (tools/robot/export_link_meshes.py): these get printed.
"""
from __future__ import annotations

import argparse
import pathlib

from build123d import export_stl

import parts
from lib import reference as R
from tools import bom

PRINT_DIR = pathlib.Path(__file__).resolve().parents[1] / "print"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--parts", nargs="*", default=None, help="printed part names (default: all of them)")
    ap.add_argument("--tolerance", type=float, default=0.01, help="linear deflection, mm")
    ap.add_argument("--angular", type=float, default=0.1, help="angular deflection, rad")
    args = ap.parse_args(argv)
    printed = [name for name in parts.names() if not parts.bought(name)]
    names = args.parts or printed
    wrong = [name for name in names if name not in printed]
    if wrong:
        ap.error(f"not printed parts (bought, or unknown): {', '.join(wrong)}")
    PRINT_DIR.mkdir(exist_ok=True)
    for stale in sorted(PRINT_DIR.glob("*.stl")):
        if stale.stem not in printed:
            stale.unlink()
            print(f"removed {stale.name} (no printed part of that name)")
    qty = {row["part"]: row["qty"] for row in bom.print_rows()}
    for name in names:
        shape = parts.build(name)
        path = PRINT_DIR / f"{name}.stl"
        ok = export_stl(shape, str(path), tolerance=args.tolerance, angular_tolerance=args.angular)
        size = path.stat().st_size if path.exists() else 0
        print(f"{name:28s} x{qty.get(name, 0)}  solids={len(shape.solids()):2d} vol={R.solid_volume(shape):11.1f} mm3 "
              f"bbox={[round(v, 1) for v in R.bbox_size(shape)]} -> print/{path.name} {size / 1e6:.2f} MB {'OK' if ok else 'FAILED'}")
        if not ok:
            return 1
    print(f"\n{len(names)} STL(s) in {PRINT_DIR} - print quantities above (x N); the full lists: ./cadtool python tools/bom.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
