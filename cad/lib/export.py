"""Plugin-free STL/STEP export helper. Writes into cad/exports/ (git-ignored).

The primary, committed parts/<group>/<name>.step files are written by running the model
(`./cadtool gen parts/<group>/<name>.py`) and meshes by `./cadtool export <file.step> stl|3mf|glb`;
use this only for ad-hoc sidecars (build123d's own exporters, absolute tolerances).
"""
import pathlib

from build123d import export_step, export_stl

# lib/export.py -> lib/ -> cad/ ; exports live at cad/exports/
EXPORT_DIR = pathlib.Path(__file__).resolve().parent.parent / "exports"


def export(part, name, *, stl=True, step=True):
    """Write {name}.stl and/or {name}.step into cad/exports/. Returns {kind: path|None}."""
    EXPORT_DIR.mkdir(parents=True, exist_ok=True)
    written = {}
    if step:
        p = EXPORT_DIR / f"{name}.step"
        written["step"] = p if export_step(part, str(p)) else None
    if stl:
        p = EXPORT_DIR / f"{name}.stl"
        written["stl"] = p if export_stl(part, str(p)) else None
    return written
