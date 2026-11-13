"""Export the fit gauges (lib/gauges): print/gauges/plate_<n>.3mf, one file per bed plate, every coupon its own named
object, laid out as lib/gauges/layout.py packs them.

    ./cadtool python tools/export_gauges.py [--only seat42 m4hex ...] [--tolerance 0.01] [--angular 0.1]
    ./cadtool python tools/export_gauges.py --list        # the gauges, their sizes and the arm's (nothing built)

A gauge prints one kind of fit (lib/gauges/fits.py GAUGES) at the arm's own sizes and at steps either side of them,
every size debossed beside its hole, ring or clip, the arm's own with a bar under it. Print them as the arm prints
(PETG, flat as laid out, at the layer height of the parts each gauge tests - the print list's layer_mm, parts/AGENTS.md
"Printed vs. bought"), try the real part in every
step, and note the step that fits as its Use says (press, slide, line to line, clearance): that size is the constant's
value. The bodies are built in-process; print/ is git-ignored (/cad/print/ in the root .gitignore) - the files are
regenerable, and the plates of an earlier export are deleted first.
"""
from __future__ import annotations

import argparse
import locale
import pathlib

import build123d as bd

from lib.gauges import coupons as C
from lib.gauges.fits import GAUGES, GROUPS
from lib.gauges.layout import BED, Piece, pack
from lib.params import PETG_DENSITY

OUT_DIR = pathlib.Path(__file__).resolve().parents[1] / "print" / "gauges"


def build_all(keys=None) -> list[C.Coupon]:
    """Every coupon of the gauges named (default: all), in GAUGES order."""
    out = []
    for g in GAUGES:
        if keys is None or g.key in keys:
            out += C.build(g)
    return out


def plates(coupons) -> list[list[tuple]]:
    """(coupon, x, y) per plate: each coupon's footprint corner on the bed."""
    by_key = {c.key: c for c in coupons}
    pieces = [Piece(c.key, c.size[0], c.size[1], c.opening, c.centre) for c in coupons]
    return [[(by_key[p.key], p.x, p.y) for p in plate] for plate in pack(pieces)]


def write_plate(items, path: pathlib.Path, tolerance: float, angular: float) -> None:
    mesher = bd.Mesher()
    for coupon, x, y in items:
        shape = bd.Pos(x, y, 0.0) * coupon.shape
        shape.label = coupon.key
        mesher.add_shape(shape, linear_deflection=tolerance, angular_deflection=angular, part_number=coupon.key)
    # lib3mf's write leaves the process in the C locale - an ASCII default encoding for every later open() /
    # read_text() without one (the test suite's, after a plate is written in-process): put the caller's back
    saved = locale.setlocale(locale.LC_ALL)
    try:
        mesher.write(str(path))
    finally:
        locale.setlocale(locale.LC_ALL, saved)


def list_gauges() -> None:
    for group, title in GROUPS:
        print(f"\n{title}")
        for g in (g for g in GAUGES if g.group == group):
            sizes = " ".join(C.fmt(s) + ("*" if C.is_arm(s, g.values) else "") for s in g.sizes)
            print(f"  {g.key:10s} {g.name}\n             {sizes}")
            for u in g.uses:
                print(f"             {C.fmt(u.value)}  {u.where}")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--only", nargs="*", default=None, help="gauge keys (default: all of them)")
    ap.add_argument("--list", action="store_true", help="list the gauges and their sizes, build nothing")
    ap.add_argument("--tolerance", type=float, default=0.01, help="linear deflection, mm")
    ap.add_argument("--angular", type=float, default=0.1, help="angular deflection, rad")
    args = ap.parse_args(argv)
    if args.list:
        list_gauges()
        return 0
    known = {g.key for g in GAUGES}
    wrong = sorted(set(args.only or ()) - known)
    if wrong:
        ap.error(f"unknown gauges: {', '.join(wrong)} (--list names them)")
    coupons = build_all(set(args.only) if args.only else None)
    layout = plates(coupons)
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    for stale in sorted(OUT_DIR.glob("plate_*.3mf")):
        stale.unlink()
    total = 0.0
    for i, items in enumerate(layout, 1):
        path = OUT_DIR / f"plate_{i}.3mf"
        write_plate(items, path, args.tolerance, args.angular)
        grams = sum(c.shape.volume for c, _, _ in items) * PETG_DENSITY
        total += grams
        tallest = max(c.size[2] for c, _, _ in items)
        print(f"{path.relative_to(OUT_DIR.parents[1])}: {len(items)} pieces, {grams:.0f} g, tallest {tallest:.1f} mm")
    print(f"{len(coupons)} pieces on {len(layout)} plates ({BED:.0f} x {BED:.0f}), {total:.0f} g PETG solid")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
