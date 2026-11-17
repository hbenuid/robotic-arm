"""Export print/<name>.step for every PRINTED part (parts.bought(name) is False - no COTS = True).

    ./cadtool python tools/export_printables.py [--parts cycloidal_disc_1 j1_link ...]

One STEP per part, in MILLIMETRES, in the part's own local frame: the exact geometry, which the slicer meshes
itself (OrcaSlicer, Bambu Studio and PrusaSlicer open STEP), so no mesh tolerance is chosen here; orienting it on
the bed is the slicer's job too (print notes - "print output-face-down" - live in each part's docstring). How many
of each to print, and at which layer height, is the print list of tools/bom.py, repeated on each part's line. The
body is built in-process (parts.build(name)): no cad build starts, and the part's own parts/<group>/<name>.step is
neither read nor written. print/ is git-ignored (/cad/print/ in the root .gitignore) - the files are regenerable; a
STEP here whose part is gone or is now bought is deleted.
"""
from __future__ import annotations

import argparse
import pathlib

from build123d import export_step

import parts
from lib import reference as R
from tools import bom

PRINT_DIR = pathlib.Path(__file__).resolve().parents[1] / "print"


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--parts", nargs="*", default=None, help="printed part names (default: all of them)")
    args = ap.parse_args(argv)
    printed = [name for name in parts.names() if not parts.bought(name)]
    names = args.parts or printed
    wrong = [name for name in names if name not in printed]
    if wrong:
        ap.error(f"not printed parts (bought, or unknown): {', '.join(wrong)}")
    PRINT_DIR.mkdir(exist_ok=True)
    for stale in sorted(PRINT_DIR.glob("*.step")):
        if stale.stem not in printed:
            stale.unlink()
            print(f"removed {stale.name} (no printed part of that name)")
    qty = {row["part"]: row["qty"] for row in bom.print_rows()}
    for name in names:
        shape = parts.build(name)
        path = PRINT_DIR / f"{name}.step"
        ok = export_step(shape, str(path))
        size = path.stat().st_size if path.exists() else 0
        print(f"{name:28s} x{qty.get(name, 0)}  {bom.layer_mm(name):.2f} mm layers  "
              f"solids={len(shape.solids()):2d} vol={R.solid_volume(shape):11.1f} mm3 "
              f"bbox={[round(v, 1) for v in R.bbox_size(shape)]} -> print/{path.name} {size / 1e6:.2f} MB {'OK' if ok else 'FAILED'}")
        if not ok:
            return 1
    print(f"\n{len(names)} STEP(s) in {PRINT_DIR} - print quantities above (x N); the full lists: ./cadtool python tools/bom.py")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
