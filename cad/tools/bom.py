"""The arm's print list and buy list, generated from the one make/buy label every part carries
(parts.bought(name): the module declares COTS = True -> bought, anything else -> printed).

    ./cadtool python tools/bom.py [--module cycloidal_drive|gripper] [--md | --json]

Counts come from the assembly tables - assemblies/arm.py OCCURRENCES, a module row expanding into that
module's own OCCURRENCES (gripper, cycloidal_drive) - so a part added to an assembly shows up here with no
second list to keep. A bought part says what to order itself (PURCHASE_SPEC / PURCHASE_QTY, the pieces
per occurrence - a whole pattern for the drive's pin and fastener parts / optional PURCHASE_NOTE, next
to its MASS_G). EXTRAS below is the one hand-kept table: purchased items with NO geometry - they are on
the buy list, not in the model, the totals or the inertials. No CAD kernel is loaded.
"""
from __future__ import annotations

import argparse
import json
from collections import Counter

import parts
from assemblies import arm
from assemblies._occurrences import module_rows
from lib import reference as R
from lib.forearm import DEFAULT as _FOREARM
from lib.params import (
    CYCLOIDAL_ARM_MOUNT_BOLT_COUNT, CYCLOIDAL_ARM_MOUNT_BOLT_DIA, FOREARM_ROLL_BELT_LENGTH, WRIST_BELT_LENGTH,
)

# (module or None = the arm itself, what to order, pieces, why it is not modelled). Seeded with what
# docs/cycloidal_drive.md states; the elbow belt, the arm's own fasteners and the electronics are not listed yet.
EXTRAS = [
    ("cycloidal_drive", f"M{CYCLOIDAL_ARM_MOUNT_BOLT_DIA:g} socket head cap screw (ISO 4762), 40-50 mm long - arm-mount bolts",
     CYCLOIDAL_ARM_MOUNT_BOLT_COUNT, "length = the arm link + the hub; settle it once j1_link is parametric"),
    ("cycloidal_drive", f"M{CYCLOIDAL_ARM_MOUNT_BOLT_DIA:g} hex nut (ISO 4032) - captive in the output hub's inner face",
     CYCLOIDAL_ARM_MOUNT_BOLT_COUNT, "drop them in before pressing the hub through the 6814s"),
    ("cycloidal_drive", "bearing grease", 1, "the output pins are a greased sliding fit through the discs"),
    ("forearm_roll_drive", f"{FOREARM_ROLL_BELT_LENGTH}-2GT closed belt, 6 mm - the roll belt (90T ring on the shaft, 20T on the motor)", 1,
     "belts are not modelled; the length sets the motor offset (lib/forearm/params.py roll_belt, [ESTIMATE])"),
    ("forearm_roll_drive", "M3 x 20 socket head cap screw (ISO 4762) - the roll shaft's flange to the forearm wall", _FOREARM.roll_end.bolt_count,
     "heads on the wall's wrist face, through the wall + the flange into the nuts captive in the flange"),
    ("forearm_roll_drive", "M3 hex nut (ISO 4032) - captive in the roll shaft's flange (from its elbow face)", _FOREARM.roll_end.bolt_count,
     "drop them in before the shaft goes into the block"),
    ("forearm_roll_drive", "M3 x 10 socket head cap screw - the bearing retainer to the elbow block's lugs (self-tapping in PETG)", 2, "or heat-set inserts"),
    ("forearm_roll_drive", "M3 x 8 socket head cap screw - the roll motor to the pad plate", 4, "through the plate's tension slots into the motor"),
    ("forearm_roll_drive", "M3 x 16 screw - the hard-stop pin, radial in the shaft's boss (self-tapping)", 1,
     "hits the retainer's post at +/- FOREARM_ROLL_LIMIT_DEG; the post + boss are modelled, the pin is not"),
    ("forearm_roll_drive", "home sensor (hall or optical) on the retainer's post + magnet / flag in the flange", 1,
     "wired to the MKS board's limit input; nothing modelled yet (docs/open_issues.md)"),
    ("forearm_roll_drive", "M4 x 25 socket head cap screw + M4 hex nut - the elbow block to j3_coupler#1 (the SolidWorks disc's 4x M4)", 4,
     "the same bolts j2_link's disc used; the nuts drop into the block's hex pockets from the top"),
    (None, f"{WRIST_BELT_LENGTH}-2GT closed belt, 6 mm - the wrist-pitch belt (90T at the wrist, 20T on the forearm motor)", 1,
     "belts are not modelled; the length sets J2_MOTOR_SLIDE_X (lib/forearm/params.py wrist_belt, [ESTIMATE])"),
]


def part_counts(module: str | None = None) -> Counter:
    """part name -> occurrences in the arm, or in one of its modules (arm.MODULES)."""
    counts: Counter = Counter()
    for name, *_ in (arm.OCCURRENCES if module is None else module_rows(module)):
        if name in arm.MODULES:
            counts.update(part_counts(name))
        else:
            counts[name] += 1
    return counts


def _state(name: str) -> str:
    if name in R.DESIGNED:
        return "designed"
    if name in R.NATIVE:
        return "native"
    return "parametric" if parts.load(name).CONVERTED else "wrapper"


def print_rows(module: str | None = None) -> list[dict]:
    """The printed parts: one row per part, with how many to print."""
    return [{"part": name, "group": parts.GROUPS[name], "qty": n, "state": _state(name)}
            for name, n in sorted(part_counts(module).items()) if not parts.bought(name)]


def buy_rows(module: str | None = None) -> list[dict]:
    """The bought parts: one row per part, pieces = occurrences x PURCHASE_QTY."""
    rows = []
    for name, n in sorted(part_counts(module).items()):
        if not parts.bought(name):
            continue
        mod = parts.load(name)
        rows.append({"part": name, "group": parts.GROUPS[name], "order": mod.PURCHASE_SPEC,
                     "pieces": n * mod.PURCHASE_QTY, "mass_g": round(n * float(mod.MASS_G), 1),
                     "geometry": "vendor" if mod.VENDOR_STEP.exists() else "envelope",
                     "note": getattr(mod, "PURCHASE_NOTE", "")})
    return rows


def extra_rows(module: str | None = None) -> list[dict]:
    """The purchased items that are not modelled (EXTRAS); `module` keeps that module's rows only."""
    return [{"module": owner or "arm", "order": spec, "pieces": pieces, "note": why}
            for owner, spec, pieces, why in EXTRAS if module is None or owner == module]


def _table(rows: list[dict], md: bool) -> str:
    if not rows:
        return "(none)"
    heads = list(rows[0])
    cells = [[str(r[h]) for h in heads] for r in rows]
    if md:
        return "\n".join(["| " + " | ".join(heads) + " |", "|" + "---|" * len(heads)] + ["| " + " | ".join(c) + " |" for c in cells])
    widths = [max(len(h), *(len(c[i]) for c in cells)) for i, h in enumerate(heads)]
    return "\n".join("  ".join(v.ljust(w) for v, w in zip(line, widths)).rstrip() for line in [heads] + cells)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--module", choices=sorted(arm.MODULES), default=None, help="one module of the arm instead of the whole arm")
    fmt = ap.add_mutually_exclusive_group()
    fmt.add_argument("--md", action="store_true", help="markdown tables")
    fmt.add_argument("--json", action="store_true")
    args = ap.parse_args(argv)
    printed, bought, extras = print_rows(args.module), buy_rows(args.module), extra_rows(args.module)
    if args.json:
        print(json.dumps({"scope": args.module or "arm", "print": printed, "buy": bought, "buy_not_modelled": extras}, indent=2))
        return 0
    head = "## " if args.md else ""
    print(f"{head}PRINT - {len(printed)} parts, {sum(r['qty'] for r in printed)} to print ({args.module or 'arm'})\n{_table(printed, args.md)}\n")
    print(f"{head}BUY - {len(bought)} parts, {sum(r['pieces'] for r in bought)} pieces, {sum(r['mass_g'] for r in bought):.0f} g\n{_table(bought, args.md)}\n")
    print(f"{head}BUY, NOT MODELLED - {len(extras)} items (tools/bom.py EXTRAS)\n{_table(extras, args.md)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
