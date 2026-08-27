"""AssemblyHelper shim.

Uses the cad@text-to-cad plugin's `cadpy.assembly.AssemblyHelper` when it is importable
(it is on PYTHONPATH under ./cadtool), else a minimal drop-in with the same
add()/add_module()/build() semantics - a labelled build123d Compound tree, which is exactly
what cadpy's build() returns when no joints are declared. So plain `uv run pytest` and
`uv run python -m assemblies.arm` work without the plugin.

Source-level joints/mates (rigid_frame, revolute_frame, face_to_face, ...) need the real
cadpy: run through ./cadtool.
"""
from __future__ import annotations

try:
    from cadpy.assembly import AssemblyHelper, label_shape, label_text  # type: ignore  # noqa: F401

    HAVE_CADPY = True
except ModuleNotFoundError:
    HAVE_CADPY = False
    from build123d import Compound

    def label_text(name, *details) -> str:
        """Compact STEP-friendly label such as `j3_coupler:j2` (mirrors cadpy.assembly)."""
        tokens = []
        for value in (name, *details):
            token = "_".join(str(value).strip().replace(":", "_").split())
            if not token:
                raise ValueError("label tokens must be non-empty")
            tokens.append(token)
        return ":".join(tokens)

    def label_shape(shape, name, *details, color=None):
        shape.label = label_text(name, *details)
        if color is not None:
            shape.color = color
        return shape

    class AssemblyHelper:
        """Minimal stand-in for cadpy.assembly.AssemblyHelper (no joints/mates)."""

        def __init__(self, name: str) -> None:
            self.label = label_text(name)
            self.children: list = []
            self.relations: list = []

        def add(self, shape, name, *details, color=None):
            label_shape(shape, name, *details, color=color)
            self.children.append(shape)
            return shape

        def add_module(self, name, children, *details, color=None):
            module = Compound(label=label_text(name, *details), children=list(children))
            if color is not None:
                module.color = color
            self.children.append(module)
            return module

        def build(self):
            return Compound(label=self.label, children=list(self.children))

        def __getattr__(self, item):  # rigid_frame, revolute_frame, face_to_face, ...
            raise AttributeError(
                f"AssemblyHelper.{item} needs the cad@text-to-cad plugin's cadpy - run via ./cadtool"
            )
