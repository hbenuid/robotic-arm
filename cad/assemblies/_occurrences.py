"""Shared helpers for assemblies/*.py: place part occurrences from reference/placements.json.

An OCCURRENCES table row is (part_or_module_name, role_or_None, placement_key), in the
SolidWorks document order. Roles make duplicate parts' labels unique (`j3_coupler:j2`);
they are positional/ordinal for now - rename them when the joint semantics are modelled.
"""
from __future__ import annotations

import importlib
import pathlib
import sys

from build123d import Location

from lib import placements as P

CAD_DIR = str(pathlib.Path(__file__).resolve().parent.parent)


def place(part_name: str, key: str):
    """A fresh copy of parts/<part_name>.gen_step() placed at occurrence `key`.

    The part is modelled in its LOCAL frame (= LOCAL_FROM_REF * reference frame); the
    extracted placement maps the REFERENCE frame into the parent frame, so compose
    rel * LOCAL_FROM_REF^-1 and MOVE (compose) rather than locate (replace)."""
    # The plugin's generator runner (scripts/gen) restores sys.path after importing the generator
    # module and does not seed the cwd, so this lazy import must re-assert the cad/ root itself.
    if CAD_DIR not in sys.path:
        sys.path.insert(0, CAD_DIR)
    mod = importlib.import_module(f"parts.{part_name}")
    local_from_ref = getattr(mod, "LOCAL_FROM_REF", None) or Location()
    return mod.gen_step().moved(P.location(key, "rel") * local_from_ref.inverse())


def add_occurrences(asm, rows, modules: dict | None = None) -> None:
    """Add every row of an OCCURRENCES table to AssemblyHelper `asm`.

    `modules` maps a module name to a zero-arg builder returning the module Compound in
    its own frame; it is located in place at the module's placement key."""
    modules = modules or {}
    for name, role, key in rows:
        if name in modules:
            module = modules[name]()
            module.locate(P.location(key, "rel"))   # in place: .located() would deep-copy the tree
            asm.add(module, name)
        elif role is None:
            asm.add(place(name, key), name)
        else:
            asm.add(place(name, key), name, role)
