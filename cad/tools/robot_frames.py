"""Joint origins, link inertials and URDF/SDF drafts for the arm, from robot/frames.py + the CAD.

    ./cadtool python tools/robot_frames.py                  # print joint origins + link inertials
    ./cadtool python tools/robot_frames.py --urdf-draft     # complete URDF draft on stdout
    ./cadtool python tools/robot_frames.py --sdf-draft      # model-level SDF draft on stdout (derived the same way)
    ./cadtool python tools/robot_frames.py --check robot/arm.urdf [robot/arm.sdf]   # compare the checked-in files

The checked-in robot/arm.urdf / arm.sdf are the source of truth (hand-edited ledger, tuned
limits); this tool is scaffolding: it recomputes every number from the CAD so `--check` catches
drift after a part is converted or a frame changes.

Units: URDF/SDF metres, kilograms, radians. Rotations are URDF fixed-axis roll/pitch/yaw
(R = Rz(yaw) Ry(pitch) Rx(roll)). Inertials: OCP BRepGProp volume properties of every part
occurrence in the link frame; printed parts use PETG_DENSITY, COTS parts their MASS_G;
per-occurrence inertia (about its own COM) is parallel-axis-shifted to the link origin, summed,
and re-centred on the link COM.
"""
from __future__ import annotations

import argparse
import importlib
import math
import pathlib
import sys
import xml.etree.ElementTree as ET

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent))

from OCP.BRepGProp import BRepGProp  # noqa: E402
from OCP.GProp import GProp_GProps  # noqa: E402

from assemblies._occurrences import place_world  # noqa: E402
from lib import params as PARAMS  # noqa: E402
from lib import placements as P  # noqa: E402
from robot import frames as F  # noqa: E402

MM = 1e-3
ROBOT_DIR = pathlib.Path(__file__).resolve().parent.parent / "robot"


# --- rotations -----------------------------------------------------------------------------
def matrix(loc):
    t = loc.wrapped.Transformation()
    return [[t.Value(i, j) for j in (1, 2, 3, 4)] for i in (1, 2, 3)]


def rpy_from_matrix(R):
    """URDF roll/pitch/yaw (fixed axes) from a 3x3 rotation: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    sy = math.hypot(R[0][0], R[1][0])
    if sy > 1e-9:
        roll = math.atan2(R[2][1], R[2][2])
        pitch = math.atan2(-R[2][0], sy)
        yaw = math.atan2(R[1][0], R[0][0])
    else:  # gimbal lock: pitch = +/-90 deg, put everything in roll
        roll = math.atan2(-R[1][2], R[1][1])
        pitch = math.atan2(-R[2][0], sy)
        yaw = 0.0
    return roll, pitch, yaw


def matrix_from_rpy(roll, pitch, yaw):
    cr, sr, cp, sp, cy, sy = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch), math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def joint_origin(j: F.Joint):
    """(xyz in m, rpy in rad) of joint `j`'s frame expressed in its parent link's frame."""
    rel = F.link_frame_world(j.parent).inverse() * F.joint_frame_world(j.name)
    M = matrix(rel)
    xyz = (M[0][3] * MM, M[1][3] * MM, M[2][3] * MM)
    rpy = rpy_from_matrix(M)
    R2 = matrix_from_rpy(*rpy)
    err = max(abs(R2[i][k] - M[i][k]) for i in range(3) for k in range(3))
    assert err < 1e-9, f"{j.name}: rpy round-trip error {err}"
    return xyz, rpy


# --- inertials -----------------------------------------------------------------------------
def occurrence_props(link: str, key: str):
    """(mass g, COM mm in link frame, inertia about COM in g.mm^2, volume mm^3) of one occurrence."""
    part = P.OCCURRENCES[key]["part"]
    mod = importlib.import_module(f"parts.{part}")
    shape = place_world(part, key, into=F.link_frame_world(link))
    props = GProp_GProps()
    BRepGProp.VolumeProperties_s(shape.wrapped, props)
    volume = props.Mass()
    c = props.CentreOfMass()
    Mi = props.MatrixOfInertia()   # about the centre of mass (verified), volume-weighted
    if getattr(mod, "COTS", False):
        mass = float(mod.MASS_G)
        rho = mass / volume
    else:
        rho = PARAMS.PETG_DENSITY
        mass = rho * volume
    inertia = [[rho * Mi.Value(i, k) for k in (1, 2, 3)] for i in (1, 2, 3)]
    return mass, (c.X(), c.Y(), c.Z()), inertia, volume


def _shift(inertia, mass, c, sign):
    """Parallel-axis term: sign=+1 moves an about-COM tensor to a point at offset c; -1 back."""
    c2 = sum(v * v for v in c)
    out = [[0.0] * 3 for _ in range(3)]
    for i in range(3):
        for k in range(3):
            out[i][k] = inertia[i][k] + sign * mass * ((c2 if i == k else 0.0) - c[i] * c[k])
    return out


def link_inertial(link: str):
    """(mass kg, COM m, inertia kg.m^2 about the COM, all in the link frame) + a report dict."""
    M = 0.0
    S = [0.0, 0.0, 0.0]
    I_o = [[0.0] * 3 for _ in range(3)]
    rows = []
    for key in F.LINKS[link]:
        m, c, I_c, v = occurrence_props(link, key)
        M += m
        for i in range(3):
            S[i] += m * c[i]
        I_at_o = _shift(I_c, m, c, +1)
        for i in range(3):
            for k in range(3):
                I_o[i][k] += I_at_o[i][k]
        rows.append((key, m, v))
    C = [s / M for s in S]
    I_C = _shift(I_o, M, C, -1)
    mass_kg = M * 1e-3
    com_m = tuple(x * MM for x in C)
    I_kgm2 = [[I_C[i][k] * 1e-9 for k in range(3)] for i in range(3)]
    gates = []
    d = [I_kgm2[0][0], I_kgm2[1][1], I_kgm2[2][2]]
    if min(d) <= 0:
        gates.append("non-positive diagonal")
    if not (d[0] + d[1] >= d[2] * 0.999 and d[0] + d[2] >= d[1] * 0.999 and d[1] + d[2] >= d[0] * 0.999):
        gates.append("triangle inequality violated")
    return mass_kg, com_m, I_kgm2, {"occurrences": rows, "gates": gates}


# --- drafts ----------------------------------------------------------------------------------
def fmt(v, nd=6):
    s = f"{v:.{nd}f}"
    return "0" if float(s) == 0 else s.rstrip("0").rstrip(".")


def fmt_i(v):
    return f"{v:.6e}"


def urdf_draft() -> str:
    out = ['<?xml version="1.0"?>',
           '<!-- DRAFT generated by tools/robot_frames.py from robot/frames.py + the CAD. The checked-in',
           '     robot/arm.urdf is the source of truth: keep its ledger comment and re-run the check mode. -->',
           f'<robot name="{F.ROBOT_NAME}">']
    for link in F.LINK_ORDER:
        if not F.LINKS[link]:
            out.append(f'  <link name="{link}" />')
            continue
        mass, com, I, _ = link_inertial(link)
        out += [f'  <link name="{link}">',
                '    <inertial>',
                f'      <origin xyz="{fmt(com[0])} {fmt(com[1])} {fmt(com[2])}" rpy="0 0 0" />',
                f'      <mass value="{fmt(mass)}" />',
                f'      <inertia ixx="{fmt_i(I[0][0])}" ixy="{fmt_i(I[0][1])}" ixz="{fmt_i(I[0][2])}" '
                f'iyy="{fmt_i(I[1][1])}" iyz="{fmt_i(I[1][2])}" izz="{fmt_i(I[2][2])}" />',
                '    </inertial>']
        for tag in ("visual", "collision"):
            out += [f'    <{tag}>', '      <origin xyz="0 0 0" rpy="0 0 0" />', '      <geometry>',
                    f'        <mesh filename="meshes/{link}.stl" scale="0.001 0.001 0.001" />',
                    '      </geometry>', f'    </{tag}>']
        out.append('  </link>')
    for j in F.JOINTS:
        xyz, rpy = joint_origin(j)
        out += [f'  <joint name="{j.name}" type="{j.type}">',
                f'    <parent link="{j.parent}" />', f'    <child link="{j.child}" />',
                f'    <origin xyz="{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])}" rpy="{fmt(rpy[0])} {fmt(rpy[1])} {fmt(rpy[2])}" />']
        if j.type != "fixed":
            out += ['    <axis xyz="0 0 1" />',
                    f'    <limit lower="{fmt(j.lower)}" upper="{fmt(j.upper)}" effort="{fmt(j.effort)}" velocity="{fmt(j.velocity)}" />']
        if j.mimic:
            out.append(f'    <mimic joint="{j.mimic[0]}" multiplier="{fmt(j.mimic[1])}" offset="{fmt(j.mimic[2])}" />')
        out.append('  </joint>')
    out.append('</robot>')
    return "\n".join(out) + "\n"


def sdf_draft() -> str:
    out = ['<?xml version="1.0"?>',
           '<!-- DRAFT generated by tools/robot_frames.py; derived mechanically from robot/arm.urdf\'s',
           '     numbers (same links, joints, limits, inertials, meshes). robot/arm.sdf is canonical. -->',
           '<sdf version="1.12">', f'  <model name="{F.ROBOT_NAME}">']
    joint_of_child = {j.child: j for j in F.JOINTS}
    for link in F.LINK_ORDER:
        j = joint_of_child.get(link)
        pose_rel = f' relative_to="{j.name}"' if j else ''
        out += [f'    <link name="{link}">', f'      <pose{pose_rel}>0 0 0 0 0 0</pose>']
        if F.LINKS[link]:
            mass, com, I, _ = link_inertial(link)
            out += ['      <inertial>',
                    f'        <pose>{fmt(com[0])} {fmt(com[1])} {fmt(com[2])} 0 0 0</pose>',
                    f'        <mass>{fmt(mass)}</mass>',
                    '        <inertia>',
                    f'          <ixx>{fmt_i(I[0][0])}</ixx><ixy>{fmt_i(I[0][1])}</ixy><ixz>{fmt_i(I[0][2])}</ixz>',
                    f'          <iyy>{fmt_i(I[1][1])}</iyy><iyz>{fmt_i(I[1][2])}</iyz><izz>{fmt_i(I[2][2])}</izz>',
                    '        </inertia>', '      </inertial>']
            for tag in ("visual", "collision"):
                out += [f'      <{tag} name="{link}_{tag}">', '        <geometry>', '          <mesh>',
                        f'            <uri>meshes/{link}.stl</uri>', '            <scale>0.001 0.001 0.001</scale>',
                        '          </mesh>', '        </geometry>', f'      </{tag}>']
        out.append('    </link>')
    for j in F.JOINTS:
        xyz, rpy = joint_origin(j)
        out += [f'    <joint name="{j.name}" type="{j.type}">',
                f'      <pose relative_to="{j.parent}">{fmt(xyz[0])} {fmt(xyz[1])} {fmt(xyz[2])} {fmt(rpy[0])} {fmt(rpy[1])} {fmt(rpy[2])}</pose>',
                f'      <parent>{j.parent}</parent>', f'      <child>{j.child}</child>']
        if j.type != "fixed":
            out += ['      <axis>', f'        <xyz expressed_in="{j.name}">0 0 1</xyz>',
                    '        <limit>', f'          <lower>{fmt(j.lower)}</lower><upper>{fmt(j.upper)}</upper>',
                    f'          <effort>{fmt(j.effort)}</effort><velocity>{fmt(j.velocity)}</velocity>',
                    '        </limit>']
            if j.mimic:
                out += [f'        <mimic joint="{j.mimic[0]}">', f'          <multiplier>{fmt(j.mimic[1])}</multiplier>',
                        f'          <offset>{fmt(j.mimic[2])}</offset>', '        </mimic>']
            out.append('      </axis>')
        out.append('    </joint>')
    out += ['  </model>', '</sdf>']
    return "\n".join(out) + "\n"


# --- check -------------------------------------------------------------------------------------
def _floats(text):
    return [float(v) for v in text.split()]


def check_urdf(path: pathlib.Path, tol_m=1e-6, tol_rad=1e-6, rel_inertia=1e-3) -> list[str]:
    root = ET.parse(path).getroot()
    problems = []
    if root.get("name") != F.ROBOT_NAME:
        problems.append(f"robot name {root.get('name')!r} != {F.ROBOT_NAME!r}")
    joints = {j.get("name"): j for j in root.findall("joint")}
    links = {l.get("name"): l for l in root.findall("link")}
    if set(links) != set(F.LINK_ORDER):
        problems.append(f"links {sorted(links)} != {sorted(F.LINK_ORDER)}")
    if set(joints) != {j.name for j in F.JOINTS}:
        problems.append(f"joints {sorted(joints)} != {sorted(j.name for j in F.JOINTS)}")
    for j in F.JOINTS:
        el = joints.get(j.name)
        if el is None:
            continue
        xyz, rpy = joint_origin(j)
        o = el.find("origin")
        got_xyz, got_rpy = _floats(o.get("xyz")), _floats(o.get("rpy"))
        if max(abs(a - b) for a, b in zip(got_xyz, xyz)) > tol_m:
            problems.append(f"{j.name}: origin xyz {got_xyz} != {[round(v, 6) for v in xyz]}")
        drpy = max(abs(math.remainder(a - b, 2 * math.pi)) for a, b in zip(got_rpy, rpy))
        if drpy > tol_rad:
            problems.append(f"{j.name}: origin rpy {got_rpy} != {[round(v, 6) for v in rpy]}")
        if el.get("type") != j.type:
            problems.append(f"{j.name}: type {el.get('type')} != {j.type}")
        if el.find("parent").get("link") != j.parent or el.find("child").get("link") != j.child:
            problems.append(f"{j.name}: parent/child mismatch")
        if j.type != "fixed":
            ax = el.find("axis")
            if ax is None or _floats(ax.get("xyz")) != [0.0, 0.0, 1.0]:
                problems.append(f"{j.name}: axis must be 0 0 1 (frames put Z on the axis)")
            lim = el.find("limit")
            for attr, want in (("lower", j.lower), ("upper", j.upper), ("effort", j.effort), ("velocity", j.velocity)):
                if lim is None or abs(float(lim.get(attr)) - want) > 1e-6:
                    problems.append(f"{j.name}: limit {attr} != {want}")
        mim = el.find("mimic")
        if (mim is None) != (j.mimic is None):
            problems.append(f"{j.name}: mimic presence mismatch")
    for link in F.LINK_ORDER:
        el = links.get(link)
        if el is None or not F.LINKS[link]:
            continue
        mass, com, I, _ = link_inertial(link)
        inertial = el.find("inertial")
        if inertial is None:
            problems.append(f"{link}: missing inertial")
            continue
        got_mass = float(inertial.find("mass").get("value"))
        if abs(got_mass - mass) > rel_inertia * mass:
            problems.append(f"{link}: mass {got_mass} != {mass:.6f}")
        got_com = _floats(inertial.find("origin").get("xyz"))
        if max(abs(a - b) for a, b in zip(got_com, com)) > 1e-5:
            problems.append(f"{link}: COM {got_com} != {[round(v, 6) for v in com]}")
        ie = inertial.find("inertia")
        for attr, want in (("ixx", I[0][0]), ("iyy", I[1][1]), ("izz", I[2][2]), ("ixy", I[0][1]), ("ixz", I[0][2]), ("iyz", I[1][2])):
            got = float(ie.get(attr))
            if abs(got - want) > rel_inertia * max(abs(want), 1e-9):
                problems.append(f"{link}: {attr} {got:.6e} != {want:.6e}")
        for tag in ("visual", "collision"):
            mesh = el.find(f"{tag}/geometry/mesh")
            if mesh is None or mesh.get("filename") != f"meshes/{link}.stl":
                problems.append(f"{link}: {tag} mesh should be meshes/{link}.stl")
            elif not (path.parent / mesh.get("filename")).exists():
                problems.append(f"{link}: mesh file missing: {mesh.get('filename')}")
    return problems


def check_sdf(path: pathlib.Path, tol=1e-6) -> list[str]:
    """The SDF must carry the same joint poses (parent-relative) and limits as the frames."""
    root = ET.parse(path).getroot()
    model = root.find("model")
    problems = []
    if model is None or model.get("name") != F.ROBOT_NAME:
        problems.append("model name mismatch")
        return problems
    joints = {j.get("name"): j for j in model.findall("joint")}
    for j in F.JOINTS:
        el = joints.get(j.name)
        if el is None:
            problems.append(f"{j.name}: missing")
            continue
        xyz, rpy = joint_origin(j)
        pose = el.find("pose")
        if pose is None or pose.get("relative_to") != j.parent:
            problems.append(f"{j.name}: pose must be relative_to the parent link")
            continue
        vals = _floats(pose.text)
        if max(abs(a - b) for a, b in zip(vals[:3], xyz)) > tol or max(abs(math.remainder(a - b, 2 * math.pi)) for a, b in zip(vals[3:], rpy)) > tol:
            problems.append(f"{j.name}: pose {vals} != {[round(v, 6) for v in (*xyz, *rpy)]}")
        if j.type != "fixed":
            lim = el.find("axis/limit")
            for tag, want in (("lower", j.lower), ("upper", j.upper), ("effort", j.effort), ("velocity", j.velocity)):
                if lim is None or abs(float(lim.find(tag).text) - want) > 1e-6:
                    problems.append(f"{j.name}: limit {tag} != {want}")
    return problems


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf-draft", action="store_true")
    ap.add_argument("--sdf-draft", action="store_true")
    ap.add_argument("--check", nargs="+", type=pathlib.Path, metavar="FILE")
    args = ap.parse_args(argv)
    if args.urdf_draft:
        sys.stdout.write(urdf_draft())
        return 0
    if args.sdf_draft:
        sys.stdout.write(sdf_draft())
        return 0
    if args.check:
        rc = 0
        for f in args.check:
            probs = check_urdf(f) if f.suffix == ".urdf" else check_sdf(f)
            print(f"{f}: {'OK' if not probs else f'{len(probs)} problem(s)'}")
            for p in probs:
                print("  -", p)
            rc |= bool(probs)
        return rc
    print("joints (origin in the parent link frame; axis = joint-frame Z):")
    for j in F.JOINTS:
        xyz, rpy = joint_origin(j)
        print(f"  {j.name:12s} {j.type:9s} {j.parent:16s}-> {j.child:16s} xyz={[round(v, 4) for v in xyz]} m  "
              f"rpy={[round(v, 4) for v in rpy]} rad  limits=[{j.lower:.3f}, {j.upper:.3f}]")
    print("links (mass, COM in the link frame, principal inertia diagonal):")
    for link in F.LINK_ORDER:
        if not F.LINKS[link]:
            print(f"  {link:16s} frame-only")
            continue
        mass, com, I, rep = link_inertial(link)
        print(f"  {link:16s} m={mass:7.4f} kg  com={[round(v, 4) for v in com]} m  "
              f"I=({I[0][0]:.3e}, {I[1][1]:.3e}, {I[2][2]:.3e})  {'GATES: ' + ', '.join(rep['gates']) if rep['gates'] else 'gates ok'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
