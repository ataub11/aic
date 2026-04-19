#!/usr/bin/env python3

#
#  Copyright (C) 2026 Intrinsic Innovation LLC
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.
#

"""Produce an aic_world variant with a prismatic-chain cable instead of the
MuJoCo elasticity plugin.

Motivation (Bug 86, ANT_PROJECT_STATUS.md):
    The MuJoCo ``mujoco.elasticity.cable`` plugin enforces a rigid-axial
    constraint between cable links (``twist``/``bend`` stiffness knobs are
    the only elasticity; there is no ``stretch`` or axial-compliance key).
    In sim, any axial pull on the plug end transmits instantly to the fixed
    base at cable_end_0 - Stage 4 compliant insertion therefore cannot make
    real forward progress under feedforward force; the policy must keep
    escalating feedforward_fz (Bug 83 chain: -3N -> -6N -> -9N) and still
    stalls. Real cable has 1-3 mm of axial give under the typical 5-15 N
    insertion forces, so the sim and real physics diverge.

    This script swaps the elasticity plugin for a discrete chain of rigid
    segments connected by four joints each:

        * slide along local +Z      (axial stretch/compression)
        * hinge about local +Z      (twist)
        * hinge about local +X      (bend in ZX plane)
        * hinge about local +Y      (bend in ZY plane)

    Each joint has explicit ``stiffness`` / ``damping``, giving genuine
    axial compliance while still behaving like a thin, bendable cable.

Usage:
    python3 add_cable_prismatic.py \\
        --input  aic_world_final.xml \\
        --output aic_world_prismatic.xml \\
        [--scene-output scene_prismatic.xml]

    The input is expected to already be the plugin-based world XML (output
    of add_cable_plugin.py). This keeps the post-processor small and
    focused; the heavy asset partitioning work is not duplicated.
"""

from __future__ import annotations

import argparse
import os
import re
import sys
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from typing import Iterable, List, Optional


# --- Tuning parameters (reasonable starting values; override via CLI) ---

DEFAULTS = dict(
    # Axial (slide) stiffness per segment [N/m]. Target: 1 N load -> ~2 mm
    # total stretch across 20 segments, giving real-like cable compliance.
    axial_stiffness=1.0e4,
    axial_damping=5.0,
    # Allowed axial travel per segment [m]; -1 mm compression, +4 mm stretch.
    axial_range_lo=-0.001,
    axial_range_hi=0.004,
    # Twist stiffness per segment [N.m/rad]. Mirrors plugin twist=1e2 spread
    # over 20 segments -> ~5 per segment, scaled up slightly because hinge
    # stiffness acts only about the joint axis.
    twist_stiffness=6.0,
    twist_damping=0.05,
    # Bend stiffness per segment [N.m/rad]. Mirrors plugin bend=2e1 spread
    # over 20 segments; keep slightly soft so cable drapes naturally.
    bend_stiffness=1.2,
    bend_damping=0.05,
    # Soft angular limits keep the sim stable; the cable should not need
    # more than +/- 45 deg per segment under realistic loads.
    bend_range_rad=0.78539816,   # +/- 45 deg
    twist_range_rad=0.52359878,  # +/- 30 deg
)


@dataclass
class PrismaticCableConfig:
    axial_stiffness: float
    axial_damping: float
    axial_range_lo: float
    axial_range_hi: float
    twist_stiffness: float
    twist_damping: float
    bend_stiffness: float
    bend_damping: float
    bend_range_rad: float
    twist_range_rad: float

    @classmethod
    def from_args(cls, args: argparse.Namespace) -> "PrismaticCableConfig":
        return cls(
            axial_stiffness=args.axial_stiffness,
            axial_damping=args.axial_damping,
            axial_range_lo=args.axial_range_lo,
            axial_range_hi=args.axial_range_hi,
            twist_stiffness=args.twist_stiffness,
            twist_damping=args.twist_damping,
            bend_stiffness=args.bend_stiffness,
            bend_damping=args.bend_damping,
            bend_range_rad=args.bend_range_rad,
            twist_range_rad=args.twist_range_rad,
        )


# --- XML manipulation helpers ---

def iter_cable_link_bodies(root: ET.Element) -> Iterable[ET.Element]:
    """Yield every body element whose name is ``link_N`` with N in [1..20].

    The cable plugin output uses ``link_1..link_20`` plus ``cable_end_0``
    and ``cable_connection_0/1``; only the numbered links need the full
    four-joint replacement.
    """
    for body in root.iter("body"):
        name = body.get("name", "")
        m = re.fullmatch(r"link_(\d+)", name)
        if m and 1 <= int(m.group(1)) <= 20:
            yield body


def find_ball_joint(body: ET.Element) -> Optional[ET.Element]:
    for j in body.findall("joint"):
        if j.get("type") == "ball":
            return j
    return None


def strip_plugin_tags(root: ET.Element) -> int:
    """Remove every ``<plugin instance="..."/>`` from every body."""
    removed = 0
    for body in root.iter("body"):
        for plugin in list(body.findall("plugin")):
            body.remove(plugin)
            removed += 1
    return removed


def strip_cable_extension(root: ET.Element) -> bool:
    """Drop the ``<extension><plugin plugin="mujoco.elasticity.cable"/>`` entry.

    If the extension section becomes empty, remove it entirely.
    """
    extension = root.find("extension")
    if extension is None:
        return False
    changed = False
    for plugin in list(extension.findall("plugin")):
        if plugin.get("plugin", "").startswith("mujoco.elasticity"):
            extension.remove(plugin)
            changed = True
    if len(list(extension)) == 0:
        root.remove(extension)
    return changed


def replace_ball_with_prismatic_chain(
    body: ET.Element, idx: int, cfg: PrismaticCableConfig
) -> bool:
    """Replace the body's single ball joint with four analytic joints.

    The four joints share the ball joint's ``pos`` so the segment still
    articulates at the same anchor point; they are inserted in the same
    location in the child order to preserve the joint-before-geom
    convention MuJoCo prefers.
    """
    ball = find_ball_joint(body)
    if ball is None:
        return False

    pos_attr = ball.get("pos", "0 0 0")

    # Build the four replacement joints. Insertion order matches kinematic
    # meaning: stretch (translation) first, then twist, then the two bends.
    def mk_joint(name: str, jtype: str, axis: str, stiffness: float,
                 damping: float, jrange: Optional[str]) -> ET.Element:
        attrs = dict(
            name=name,
            pos=pos_attr,
            type=jtype,
            axis=axis,
            stiffness=f"{stiffness:g}",
            damping=f"{damping:g}",
        )
        if jrange is not None:
            attrs["limited"] = "true"
            attrs["range"] = jrange
        el = ET.Element("joint", attrs)
        return el

    stretch = mk_joint(
        f"stretch_{idx}", "slide", "0 0 1",
        cfg.axial_stiffness, cfg.axial_damping,
        f"{cfg.axial_range_lo:g} {cfg.axial_range_hi:g}",
    )
    twist = mk_joint(
        f"twist_{idx}", "hinge", "0 0 1",
        cfg.twist_stiffness, cfg.twist_damping,
        f"{-cfg.twist_range_rad:g} {cfg.twist_range_rad:g}",
    )
    bend_x = mk_joint(
        f"bend_x_{idx}", "hinge", "1 0 0",
        cfg.bend_stiffness, cfg.bend_damping,
        f"{-cfg.bend_range_rad:g} {cfg.bend_range_rad:g}",
    )
    bend_y = mk_joint(
        f"bend_y_{idx}", "hinge", "0 1 0",
        cfg.bend_stiffness, cfg.bend_damping,
        f"{-cfg.bend_range_rad:g} {cfg.bend_range_rad:g}",
    )

    # Insert replacements at the ball's position, then remove the ball.
    children = list(body)
    ball_index = children.index(ball)
    for offset, new_joint in enumerate((stretch, twist, bend_x, bend_y)):
        body.insert(ball_index + offset, new_joint)
    body.remove(ball)
    return True


def retune_connection_joints(root: ET.Element, cfg: PrismaticCableConfig) -> int:
    """Give the cable_connection_{0,1} ball joints explicit damping.

    Those ball joints connect the cable ends to the plug bodies. Without
    the plugin, they have only their existing (weak) damping; we bump
    damping to match the bend joints so the plug doesn't wobble freely.
    """
    touched = 0
    for body_name in ("cable_connection_0", "cable_connection_1"):
        body = _find_body_by_name(root, body_name)
        if body is None:
            continue
        for j in body.findall("joint"):
            if j.get("type") == "ball":
                j.set("damping", f"{max(cfg.bend_damping * 4, 0.2):g}")
                touched += 1
    return touched


def _find_body_by_name(root: ET.Element, name: str) -> Optional[ET.Element]:
    for body in root.iter("body"):
        if body.get("name") == name:
            return body
    return None


def remove_cable_default_block(root: ET.Element) -> bool:
    """Delete the ``<default class="cable_default">`` entry and childclass refs.

    The class only carried ``joint damping=0.1`` which is irrelevant once
    every cable joint specifies its own damping.
    """
    defaults = root.find("default")
    changed = False
    if defaults is not None:
        for d in list(defaults.findall("default")):
            if d.get("class") == "cable_default":
                defaults.remove(d)
                changed = True

    # Scrub childclass="cable_default" from any remaining body.
    for body in root.iter("body"):
        if body.get("childclass") == "cable_default":
            del body.attrib["childclass"]
            changed = True
    return changed


# --- Main pipeline ---

def build_prismatic_cable_world(
    input_path: str,
    output_path: str,
    cfg: PrismaticCableConfig,
) -> None:
    tree = ET.parse(input_path)
    root = tree.getroot()

    if root.tag != "mujoco":
        raise ValueError(f"Expected <mujoco> root, got <{root.tag}>")

    ext_stripped = strip_cable_extension(root)
    plugin_tag_count = strip_plugin_tags(root)

    link_count = 0
    for body in iter_cable_link_bodies(root):
        idx_match = re.fullmatch(r"link_(\d+)", body.get("name", ""))
        if idx_match is None:
            continue
        if replace_ball_with_prismatic_chain(body, int(idx_match.group(1)), cfg):
            link_count += 1

    conn_count = retune_connection_joints(root, cfg)
    cls_removed = remove_cable_default_block(root)

    _pretty_indent(root)
    tree.write(output_path, encoding="utf-8", xml_declaration=False)

    print(
        f"[prismatic-cable] extension_stripped={ext_stripped} "
        f"plugin_tags_removed={plugin_tag_count} "
        f"links_converted={link_count} "
        f"connection_joints_retuned={conn_count} "
        f"cable_default_removed={cls_removed}"
    )


def _pretty_indent(elem: ET.Element, level: int = 0) -> None:
    """Minimal in-place indenter (stdlib ET.indent exists on 3.9+ but we
    avoid depending on it to keep this script portable)."""
    indent = "\n" + level * "  "
    child_indent = indent + "  "
    if len(elem):
        if not elem.text or not elem.text.strip():
            elem.text = child_indent
        for child in elem:
            _pretty_indent(child, level + 1)
            if not child.tail or not child.tail.strip():
                child.tail = child_indent
        if not elem[-1].tail or not elem[-1].tail.strip():
            elem[-1].tail = indent
    else:
        if level and (not elem.tail or not elem.tail.strip()):
            elem.tail = indent


def write_scene(
    scene_path: str, robot_xml_rel: str, world_xml_rel: str
) -> None:
    scene_xml = (
        '<mujoco model="Scene-Prismatic">\n'
        '  <option integrator="implicitfast"/>\n'
        f'  <include file="{robot_xml_rel}"/>\n'
        f'  <include file="{world_xml_rel}"/>\n'
        '</mujoco>\n'
    )
    with open(scene_path, "w") as f:
        f.write(scene_xml)


def parse_args(argv: List[str]) -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description=(
            "Rewrite an aic_world XML to replace the mujoco.elasticity.cable "
            "plugin with a prismatic-chain cable that has true axial "
            "compliance. Addresses ANT project Bug 86 (sim rigid axial)."
        )
    )
    p.add_argument(
        "--input", required=True,
        help="Path to the plugin-based world XML (e.g. aic_world_final.xml)",
    )
    p.add_argument(
        "--output", required=True,
        help="Destination path for the prismatic-chain world XML",
    )
    p.add_argument(
        "--scene-output", default=None,
        help="Optional scene XML path. If set, a minimal scene is written "
             "that includes aic_robot.xml alongside the new world.",
    )
    p.add_argument(
        "--robot-xml", default="aic_robot.xml",
        help="Robot XML filename to reference from the scene (if --scene-output given)",
    )
    for k, v in DEFAULTS.items():
        flag = "--" + k.replace("_", "-")
        p.add_argument(flag, type=float, default=v, dest=k)
    return p.parse_args(argv)


def main(argv: List[str]) -> int:
    if "BUILD_WORKSPACE_DIRECTORY" in os.environ:
        os.chdir(os.environ["BUILD_WORKSPACE_DIRECTORY"])
    args = parse_args(argv)
    cfg = PrismaticCableConfig.from_args(args)

    input_path = os.path.abspath(args.input)
    output_path = os.path.abspath(args.output)
    if not os.path.exists(input_path):
        print(f"error: input not found: {input_path}", file=sys.stderr)
        return 2

    build_prismatic_cable_world(input_path, output_path, cfg)

    if args.scene_output:
        scene_path = os.path.abspath(args.scene_output)
        scene_dir = os.path.dirname(scene_path) or "."
        world_rel = os.path.relpath(output_path, scene_dir)
        robot_rel = args.robot_xml
        # If the user passed an absolute robot path, make it relative to scene dir.
        if os.path.isabs(robot_rel):
            robot_rel = os.path.relpath(robot_rel, scene_dir)
        write_scene(scene_path, robot_rel, world_rel)
        print(f"[prismatic-cable] wrote scene: {scene_path}")

    print(f"[prismatic-cable] wrote world: {output_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
