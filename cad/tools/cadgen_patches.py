"""Re-applicable patches for the installed cadgen runtime (the viewer + snapshot browser bundles).

    ./cadtool patch                 # apply, idempotent   (= uv run --frozen python tools/cadgen_patches.py apply)
    ./cadtool patch check           # one line per bundle; rc 0 patched / 1 unpatched / 2 not applicable
    ./cadtool patch revert          # restore the <bundle>.orig backups

Why this exists - cadgen 0.5.0 / 0.5.1 draw every URDF / SRDF / SDF as a pile of huge shards:
  * the robot loader is right: packages/cadgen-js/src/lib/urdf/parseUrdf.js reads <mesh scale> and the
    joint origins, lib/urdf/kinematics.js poseUrdfMeshData composes linkWorld * visualOrigin * meshScale
    into every part.transform, and the camera frames the posed, metre-scale bounds;
  * the renderer then drops that transform: common/stepModuleEffects.js displayTransformForPart returns
    part.transform only when meshData.partTransformsBaked === false, and the URDF mesh data built by
    lib/urdf/kinematics.js buildUrdfMeshGeometry (the lightweight path lib/urdf/loadRobot.js uses) never
    sets the flag. Each link is drawn with an identity matrix at its raw STL coordinates (millimetres,
    unposed), so the links pile up 1000x too big and the joint sliders / --joint-values change nothing
    visible. cadgen 0.4.28 (text-to-cad `0e94cd1d`) still had the fallback branch
    (renderPartsIndividually -> part.transform) that 0.5.0 (`3eb1f9b8`) removed.
  Upstream fix (one line; github.com/earthtojake/text-to-cad, packages/cadgen-js/src/lib/urdf/kinematics.js,
  the lightweight return object of buildUrdfMeshGeometry): add `partTransformsBaked: false,`. Until that
  ships, this tool inserts the minified equivalent into the two built bundles under .venv (git-ignored;
  a `<bundle>.orig` backup sits beside each). `uv sync` / a cadgen reinstall reverts it: `./cadtool setup`
  re-applies, `./cadtool doctor` and tests/test_tooling.py check, `./cadtool viewer` / `snapshot` warn.
  The merged builder (buildUrdfMeshData, no `lightweight`) is not used by the viewer or the snapshots
  and is left alone.
"""
from __future__ import annotations

import argparse
import dataclasses
import glob
import pathlib


@dataclasses.dataclass(frozen=True)
class Patch:
    name: str
    files: tuple[str, ...]              # globs under cadgen/_runtime
    anchor: str                         # occurs exactly once in an unpatched bundle
    insert: str                         # inserted right before the anchor
    cadgen_versions: tuple[str, ...]    # releases known to need it (others need --force)


PATCHES = (
    Patch(
        name="urdf-part-transforms",
        files=("viewer/assets/index-*.js", "browser/snapshot-render.js"),
        anchor='lightweightGeometry:!0,geometrySource:{type:"urdf-source-parts"',
        insert="partTransformsBaked:!1,",
        cadgen_versions=("0.5.0", "0.5.1"),
    ),
)

PATCHED, UNPATCHED, NOT_APPLICABLE = "patched", "unpatched", "not-applicable"


def runtime_dir() -> pathlib.Path:
    import cadgen

    return pathlib.Path(cadgen.__file__).resolve().parent / "_runtime"


def cadgen_version() -> str:
    import cadgen

    return str(getattr(cadgen, "__version__", "?"))


def bundles(patch: Patch) -> list[pathlib.Path]:
    root = runtime_dir()
    found = [pathlib.Path(p) for pattern in patch.files for p in sorted(glob.glob(str(root / pattern)))]
    return [p for p in found if not p.name.endswith(".orig")]


def state(patch: Patch, bundle: pathlib.Path) -> str:
    text = bundle.read_text(encoding="utf-8")
    if patch.insert + patch.anchor in text:
        return PATCHED
    if text.count(patch.anchor) == 1:
        return UNPATCHED
    return NOT_APPLICABLE       # neither the patch nor its anchor: the bundle changed (or was fixed upstream)


def check() -> dict[str, dict[str, str]]:
    """{patch name: {bundle path: PATCHED | UNPATCHED | NOT_APPLICABLE}}."""
    return {p.name: {str(b): state(p, b) for b in bundles(p)} for p in PATCHES}


def apply(force: bool = False) -> int:
    version = cadgen_version()
    rc = 0
    for patch in PATCHES:
        if version not in patch.cadgen_versions and not force:
            print(f"{patch.name}: cadgen {version} is not a release known to need it ({', '.join(patch.cadgen_versions)}) - "
                  "check that `./cadtool snapshot robot/arm.urdf /tmp/x.png` renders an arm; if it is still a pile, "
                  "re-run with --force")
            rc = max(rc, 2)
            continue
        found = bundles(patch)
        if not found:
            print(f"{patch.name}: no bundle matches {patch.files} under {runtime_dir()}")
            rc = 2
            continue
        for bundle in found:
            s = state(patch, bundle)
            if s == PATCHED:
                print(f"{patch.name}: {bundle.name}: already patched")
            elif s == UNPATCHED:
                backup = bundle.with_name(bundle.name + ".orig")
                text = bundle.read_text(encoding="utf-8")
                if not backup.exists():
                    backup.write_text(text, encoding="utf-8")
                bundle.write_text(text.replace(patch.anchor, patch.insert + patch.anchor, 1), encoding="utf-8")
                print(f"{patch.name}: {bundle.name}: patched (backup {backup.name})")
            else:
                print(f"{patch.name}: {bundle.name}: anchor not found exactly once - the bundle changed; "
                      "re-verify the upstream code before patching (see the module docstring)")
                rc = max(rc, 2)
    return rc


def report() -> int:
    rc = 0
    for name, states in check().items():
        if not states:
            print(f"{name}: no bundle found under {runtime_dir()}")
            rc = max(rc, 2)
            continue
        for bundle, s in states.items():
            print(f"{name}: {pathlib.Path(bundle).name}: {s}")
            if s == UNPATCHED:
                rc = max(rc, 1)
            elif s == NOT_APPLICABLE:
                rc = max(rc, 2)
    if rc == 1:
        print("run ./cadtool patch - robots render as a pile in the viewer and in snapshots until then")
    return rc


def revert() -> int:
    rc = 0
    for patch in PATCHES:
        for bundle in bundles(patch):
            backup = bundle.with_name(bundle.name + ".orig")
            if backup.exists():
                bundle.write_text(backup.read_text(encoding="utf-8"), encoding="utf-8")
                backup.unlink()
                print(f"{patch.name}: {bundle.name}: reverted")
            else:
                print(f"{patch.name}: {bundle.name}: no .orig backup, nothing to revert")
                rc = 1
    return rc


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", nargs="?", choices=("apply", "check", "revert"), default="apply")
    ap.add_argument("--force", action="store_true", help="apply on a cadgen release not in the known list")
    args = ap.parse_args(argv)
    print(f"cadgen {cadgen_version()} runtime: {runtime_dir()}")
    return {"apply": lambda: apply(args.force), "check": report, "revert": revert}[args.action]()


if __name__ == "__main__":
    raise SystemExit(main())
