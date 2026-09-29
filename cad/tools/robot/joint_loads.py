"""Worst-case static load per joint of robot/arm.urdf: what each joint's drive has to hold.

    ./cadtool python tools/robot/joint_loads.py                  # 0.2 kg payload at tool0
    ./cadtool python tools/robot/joint_loads.py --payload 0      # the arm alone
    ./cadtool python tools/robot/joint_loads.py --step 15        # a finer search (slower)

For each revolute joint: everything beyond it (its subtree's links + the payload, a point mass at
tool0) has mass M and its centre of mass lies r from the joint's axis. The joints beyond it are
searched on a grid over their limits for the largest M * r; the static load M * g * r is what the
drive holds when the joints before it turn that offset level (every joint of this arm but base_yaw
can be turned so). base_yaw's axis stays vertical - gravity puts no torque on it -, so its row is
the pose with the largest inertia about its axis instead. The inertia column is the subtree's
about the axis, in the worst pose.

The masses are the URDF's inertials (tools/robot/derive.py: printed parts at PETG_DENSITY, i.e.
solid), so a printed arm at normal infill loads its joints less. Compare a load with a drive's
motor torque x its reduction (robot/CLAUDE.md "the reductions") x its efficiency.
"""
from __future__ import annotations

import argparse
import itertools
import math
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

G = 9.81
ROBOT_DIR = pathlib.Path(__file__).resolve().parents[2] / "robot"


def _floats(text):
    return [float(v) for v in text.split()]


def _rpy(roll, pitch, yaw):
    """URDF fixed-axis roll / pitch / yaw: R = Rz(yaw) Ry(pitch) Rx(roll)."""
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch),
                              math.cos(yaw), math.sin(yaw))
    rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return rz @ ry @ rx


def _about(axis, angle):
    """Rotation by `angle` about the unit vector `axis` (Rodrigues)."""
    k = np.array([[0, -axis[2], axis[1]], [axis[2], 0, -axis[0]], [-axis[1], axis[0], 0]])
    return np.eye(3) + math.sin(angle) * k + (1 - math.cos(angle)) * k @ k


def _tf(xyz, rot):
    t = np.eye(4)
    t[:3, :3], t[:3, 3] = rot, xyz
    return t


class Robot:
    """The links' inertials and the joint tree of a URDF (metres, kilograms)."""

    def __init__(self, path: pathlib.Path):
        root = ET.parse(path).getroot()
        self.inertials = {}
        for link in root.findall("link"):
            inertial = link.find("inertial")
            if inertial is None:
                continue
            i = {k: float(v) for k, v in inertial.find("inertia").attrib.items()}
            tensor = np.array([[i["ixx"], i["ixy"], i["ixz"]],
                               [i["ixy"], i["iyy"], i["iyz"]],
                               [i["ixz"], i["iyz"], i["izz"]]])
            self.inertials[link.get("name")] = (float(inertial.find("mass").get("value")),
                                                np.array(_floats(inertial.find("origin").get("xyz"))), tensor)
        self.joints, self.children = {}, {}
        for j in root.findall("joint"):
            origin, axis, limit = j.find("origin"), j.find("axis"), j.find("limit")
            parent = j.find("parent").get("link")
            self.joints[j.get("name")] = {
                "type": j.get("type"), "parent": parent, "child": j.find("child").get("link"),
                "origin": _tf(_floats(origin.get("xyz")), _rpy(*_floats(origin.get("rpy")))),
                "axis": np.array(_floats(axis.get("xyz")) if axis is not None else [0.0, 0.0, 1.0]),
                "range": (float(limit.get("lower")), float(limit.get("upper"))) if limit is not None else (0.0, 0.0)}
            self.children.setdefault(parent, []).append(j.get("name"))
        self.base = next(iter(set(self.children) - {j["child"] for j in self.joints.values()}))

    def revolute(self) -> list[str]:
        """The revolute joints, base to tip."""
        order, stack = [], [self.base]
        while stack:
            for name in self.children.get(stack.pop(0), []):
                order += [name] if self.joints[name]["type"] == "revolute" else []
                stack.append(self.joints[name]["child"])
        return order

    def poses(self, q: dict[str, float]):
        """World frame of every link, and of every joint (its origin, before its own rotation)."""
        links, joints, stack = {self.base: np.eye(4)}, {}, [self.base]
        while stack:
            parent = stack.pop()
            for name in self.children.get(parent, []):
                j = self.joints[name]
                joints[name] = links[parent] @ j["origin"]
                turn = _about(j["axis"], q.get(name, 0.0)) if j["type"] == "revolute" else np.eye(3)
                links[j["child"]] = joints[name] @ _tf([0, 0, 0], turn)
                stack.append(j["child"])
        return links, joints

    def subtree(self, joint: str) -> list[str]:
        out, stack = [], [self.joints[joint]["child"]]
        while stack:
            link = stack.pop()
            out.append(link)
            stack += [self.joints[c]["child"] for c in self.children.get(link, [])]
        return out

    def load(self, joint: str, q: dict[str, float], payload: float, tip: str = "tool0"):
        """(M kg, r m, inertia kg m^2 about the axis) of everything beyond `joint` in pose `q`."""
        links, joints = self.poses(q)
        point = joints[joint][:3, 3]
        axis = joints[joint][:3, :3] @ self.joints[joint]["axis"]
        bodies = []
        for link in self.subtree(joint):
            if link in self.inertials:
                m, com, tensor = self.inertials[link]
                rot = links[link][:3, :3]
                bodies.append((m, rot @ com + links[link][:3, 3], rot @ tensor @ rot.T))
            if link == tip and payload:
                bodies.append((payload, links[link][:3, 3], np.zeros((3, 3))))
        mass = sum(m for m, _, _ in bodies)
        inertia = 0.0
        for m, c, tensor in bodies:
            d = c - point
            d -= axis * (d @ axis)
            inertia += axis @ tensor @ axis + m * (d @ d)
        d = sum(m * c for m, c, _ in bodies) / mass - point
        return mass, float(np.linalg.norm(d - axis * (d @ axis))), inertia

    def vertical(self, joint: str) -> bool:
        """True when `joint`'s axis is world-vertical in every pose: it and every joint before it turn about Z."""
        _, joints = self.poses({})
        chain, link = [], self.joints[joint]["parent"]
        while link != self.base:
            name = next(n for n, j in self.joints.items() if j["child"] == link)
            chain.append(name)
            link = self.joints[name]["parent"]
        return all(abs((joints[n][:3, :3] @ self.joints[n]["axis"])[2]) > 1 - 1e-9
                   for n in [joint, *chain] if self.joints[n]["type"] == "revolute")


def worst(robot: Robot, joint: str, payload: float, step: float):
    """The worst pose of the joints beyond `joint`: (M, r, inertia, pose, vertical)."""
    order = robot.revolute()
    beyond = order[order.index(joint) + 1:]
    grids = [np.arange(*robot.joints[b]["range"], step).tolist() + [robot.joints[b]["range"][1]] for b in beyond]
    vertical = robot.vertical(joint)
    best = None
    for combo in itertools.product(*grids):
        q = dict(zip(beyond, combo, strict=True))
        mass, r, inertia = robot.load(joint, q, payload)
        key = inertia if vertical else mass * r
        if best is None or key > best[0]:
            best = (key, mass, r, inertia, q)
    return (*best[1:], vertical)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--payload", type=float, default=0.2, help="kg at tool0 (default 0.2)")
    ap.add_argument("--step", type=float, default=30.0, help="grid step of the search, degrees (default 30)")
    ap.add_argument("--urdf", type=pathlib.Path, default=ROBOT_DIR / "arm.urdf")
    args = ap.parse_args(argv)
    robot = Robot(args.urdf)
    print(f"{args.urdf.name}, payload {args.payload} kg at tool0, search step {args.step:g} deg")
    print(f"  {'joint':15s} {'M kg':>6s} {'r mm':>6s} {'load N m':>9s} {'I kg m2':>8s}  worst pose of the joints beyond")
    for joint in robot.revolute():
        mass, r, inertia, q, vertical = worst(robot, joint, args.payload, math.radians(args.step))
        load = "vertical" if vertical else f"{mass * G * r:9.2f}"
        pose = ", ".join(f"{n} {math.degrees(v):.0f}" for n, v in q.items())
        print(f"  {joint:15s} {mass:6.3f} {r * 1000:6.1f} {load:>9s} {inertia:8.4f}  {pose}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
