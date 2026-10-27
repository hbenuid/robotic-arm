"""Native build123d assembly nodes - what cadgen writes as the STEP component tree.

An assembly is a labelled `Compound(children=[...])`: `assembly(name, children)` builds one node
(the root of a model, or a group / module inside it), `label_shape(shape, name, *details)` labels a
child and `label_text` gives the compact STEP-friendly label (`gt2_pulley_90t:j3`): each token stripped,
its whitespace runs and colons turned into `_`, the tokens joined by `:` (cadgen 0.7's `cadgen.assembly`
rule, which 0.8 dropped: 0.8 reads a plain `shape.label`). The arm declares no joints (`robot/` is the
mechanism).
"""

from __future__ import annotations

import build123d as bd

__all__ = ["assembly", "label_shape", "label_text"]


def label_text(name: str, *details: object) -> str:
    """`name:detail:...` - each token stripped, whitespace and `:` inside it made `_`; an empty token is an error."""
    tokens = []
    for field, value in (("name", name), *(("detail", d) for d in details)):
        token = str(value).strip()
        if not token:
            raise ValueError(f"label {field} must be non-empty")
        tokens.append("_".join(token.replace(":", "_").split()))
    return ":".join(tokens)


def label_shape(shape, name: str, *details: object):
    """`shape` labelled `label_text(name, *details)`, returned."""
    shape.label = label_text(name, *details)
    return shape


def assembly(name: str, children, *details) -> bd.Compound:
    """`Compound(label=label_text(name, *details), children=children)`. No colour on the node: the colours are
    the leaves' (`assemblies/_occurrences.py _tint`)."""
    return bd.Compound(label=label_text(name, *details), children=list(children))
