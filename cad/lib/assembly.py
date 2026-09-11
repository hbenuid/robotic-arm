"""AssemblyHelper - re-exported from cadgen (a locked dependency of this project).

`AssemblyHelper(name)` collects labelled children (`add(shape, name, *details, color=None)`,
`add_module(name, children, *details, color=None)`) and `build()` returns the labelled
build123d Compound tree cadgen packages as an assembly. `label_shape` / `label_text` give the
compact STEP-friendly labels (`j3_coupler:j2`). Source-level joints (`rigid_frame`,
`revolute_frame`, `connect`, ...) live on the same class; persisted motion is `@step(kinematics=...)`.
"""
from cadgen.assembly import AssemblyHelper, label_shape, label_text  # noqa: F401

__all__ = ["AssemblyHelper", "label_shape", "label_text"]
