# vendor/ — purchased-part STEP files (current best model per part)

**Purpose:** the geometry a `COTS = True` part in `parts/<group>/` imports when it has a vendor file (the others build
their `_envelope()`). Committed via the `!/cad/vendor/*.step` gitignore exception (not regenerable from Python). Unlike
the part's reference (`reference/<origin>/<name>.step`, which fixes its **frame and size**), a vendor file may be
**replaced** by a better catalog model. The rules and how to produce or swap one (Recipe D): [`AGENTS.md`](AGENTS.md).
This file is the record of the step.parts catalog searches, so a rejected model is not tried twice.

Where each file is described: what it is, its frame and where it came from - the part module's docstring
(`parts/<group>/<name>.py`; the MKS kit motors and boards: `tools/reference/split_mks_motor.py`); its bytes, bbox and
whether it is its own reference - `reference/manifest.json` (the entry's `vendor` block); what to **order** -
`PURCHASE_SPEC` / `PURCHASE_QTY` / `PURCHASE_NOTE` in the part module, printed by `./cadtool python tools/bom.py`, which
also marks each bought line `vendor` or `envelope`.

## Catalog searches (step.parts)

Parts with a vendor file:

| Part | Catalog outcome |
|---|---|
| `gt2_pulley_20t` | `gt2_pulley_20t_bore5_w6` **tried and rejected**: analytic simplified (10 faces, Ø18.2 flanges, 10 mm long — beyond `COTS_FRAME_TOL_MM`); the SolidWorks re-export (real teeth) stays |
| `mg996r_servo` | no `MG996R` / `towerpro` model, only generic servo envelopes — the third-party model stays |
| `nema17_pancake` | `stepper_motor_nema17_l0020_single_shaft` is analytic simplified — the SolidWorks sub-assembly stays |
| `bearing_625` | `bearing_625_2rs_sealed_simple` **adopted** |
| `nema17_40mm`, `nema17_48mm`, `mks_servo42d` | not searched: the user's own kit exports, split by `tools/reference/split_mks_motor.py` |

Parts without one (their envelope is the geometry; the pattern parts - pins, screw and nut sets - have no catalog model:
the catalog has single fasteners only):

| Part | Catalog outcome |
|---|---|
| `bearing_6003` | `bearing_6003_2rs_sealed_simple` **tried and rejected**: the file is a Ø24 × 8 bearing (a 628 size), not 17 × 35 × 10 |
| `bearing_6814` | no 6814 / 61814 entry (search and direct ids 404) |
| `bearing_6806` | no 6806 / 61806 / 30 x 42 x 7 entry (the catalog's bearings stop at a 20 mm bore) - a native COTS part |
| `gt2_idler_20t` | `gt2_smooth_idler_bore5_w6` / `gt2_flanged_smooth_idler_bore5_w6` **tried and rejected**: one generic model under both ids (Ø18.2 flanges on a Ø15.2 seat, 10 mm long, three loose solids, no bore) where the seller's drawing is a Ø12.1 seat, 9 mm long - a native COTS part built from that drawing (`lib/belts.py GT2_IDLER_*`) |
| `ky003_hall_sensor` | no KY-003 module and no A3144 ("KY-003", "KY003", "A3144", "hall sensor", "hall effect"; `to_92s` is a bare package) - a native COTS part, its dimensions `lib/sensors.py` |
