#!/usr/bin/env python3
"""Create the deterministic pick-demo MJCF without changing the formal model."""

from __future__ import annotations

import argparse
import hashlib
import json
import xml.etree.ElementTree as ET
from pathlib import Path

OBJECT_POSITION = (0.11697, -0.15350, 0.52183)
OBJECT_HALF_SIZE = (0.015, 0.020, 0.015)
GRAVITY = (0.0, 0.0, -9.81)
TABLE_TOP_Z = 0.42
OBJECT_BOTTOM_Z = OBJECT_POSITION[2] - OBJECT_HALF_SIZE[2]
SUPPORT_HALF_SIZE = (0.040, 0.040, (OBJECT_BOTTOM_Z - TABLE_TOP_Z) / 2.0)
SUPPORT_POSITION = (
    OBJECT_POSITION[0],
    OBJECT_POSITION[1],
    TABLE_TOP_Z + SUPPORT_HALF_SIZE[2],
)
GUIDE_HALF_HEIGHT = 0.001
GUIDE_THICKNESS = 0.001
GUIDE_CLEARANCE = 0.002
GUIDE_CENTER_Z = SUPPORT_HALF_SIZE[2] + GUIDE_HALF_HEIGHT


def build_scene(source: Path, output: Path) -> dict[str, object]:
    source = source.expanduser().resolve()
    output = output.expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"formal MJCF is missing: {source}")
    tree = ET.parse(source)
    root = tree.getroot()
    root.set("model", "openarm_pick_demo_v8")
    option = root.find("option")
    if option is None:
        option = ET.Element("option")
        root.insert(1, option)
    option.set("gravity", "0 0 -9.81")
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise ValueError("formal MJCF has no worldbody")
    if root.find(".//body[@name='pick_demo_block']") is not None:
        raise ValueError("formal MJCF unexpectedly already contains pick_demo_block")
    if root.find(".//body[@name='pick_demo_support']") is not None:
        raise ValueError("formal MJCF unexpectedly already contains pick_demo_support")
    support = ET.Element(
        "body",
        {
            "name": "pick_demo_support",
            "pos": " ".join(f"{value:.6f}" for value in SUPPORT_POSITION),
        },
    )
    ET.SubElement(
        support,
        "geom",
        {
            "name": "pick_demo_support_geom",
            "type": "box",
            "size": " ".join(f"{value:.6f}" for value in SUPPORT_HALF_SIZE),
            "rgba": "0.25 0.28 0.32 1",
            "friction": "2.0 0.5 0.1",
            "condim": "4",
        },
    )
    guide_specs = (
        (
            "pick_demo_cradle_x_negative",
            (
                -OBJECT_HALF_SIZE[0] - GUIDE_CLEARANCE - GUIDE_THICKNESS,
                0.0,
                GUIDE_CENTER_Z,
            ),
            (
                GUIDE_THICKNESS,
                OBJECT_HALF_SIZE[1] + GUIDE_CLEARANCE,
                GUIDE_HALF_HEIGHT,
            ),
        ),
        (
            "pick_demo_cradle_x_positive",
            (
                OBJECT_HALF_SIZE[0] + GUIDE_CLEARANCE + GUIDE_THICKNESS,
                0.0,
                GUIDE_CENTER_Z,
            ),
            (
                GUIDE_THICKNESS,
                OBJECT_HALF_SIZE[1] + GUIDE_CLEARANCE,
                GUIDE_HALF_HEIGHT,
            ),
        ),
        (
            "pick_demo_cradle_y_negative",
            (
                0.0,
                -OBJECT_HALF_SIZE[1] - GUIDE_CLEARANCE - GUIDE_THICKNESS,
                GUIDE_CENTER_Z,
            ),
            (
                OBJECT_HALF_SIZE[0] + GUIDE_CLEARANCE,
                GUIDE_THICKNESS,
                GUIDE_HALF_HEIGHT,
            ),
        ),
        (
            "pick_demo_cradle_y_positive",
            (
                0.0,
                OBJECT_HALF_SIZE[1] + GUIDE_CLEARANCE + GUIDE_THICKNESS,
                GUIDE_CENTER_Z,
            ),
            (
                OBJECT_HALF_SIZE[0] + GUIDE_CLEARANCE,
                GUIDE_THICKNESS,
                GUIDE_HALF_HEIGHT,
            ),
        ),
    )
    for name, position, half_size in guide_specs:
        ET.SubElement(
            support,
            "geom",
            {
                "name": name,
                "type": "box",
                "pos": " ".join(f"{value:.6f}" for value in position),
                "size": " ".join(f"{value:.6f}" for value in half_size),
                "rgba": "0.35 0.38 0.42 1",
                "friction": "2.0 0.5 0.1",
                "condim": "4",
            },
        )
    block = ET.Element(
        "body",
        {
            "name": "pick_demo_block",
            "pos": " ".join(f"{value:.5f}" for value in OBJECT_POSITION),
        },
    )
    ET.SubElement(
        block,
        "joint",
        {
            "name": "pick_demo_block_freejoint",
            "type": "free",
            "damping": "0.2",
        },
    )
    ET.SubElement(
        block,
        "geom",
        {
            "name": "pick_demo_block_geom",
            "type": "box",
            "size": " ".join(f"{value:.5f}" for value in OBJECT_HALF_SIZE),
            "mass": "0.025",
            "rgba": "0.95 0.75 0.05 1",
            "friction": "2.0 0.5 0.1",
            "condim": "4",
        },
    )
    first_robot = next(
        (index for index, body in enumerate(worldbody) if body.get("name") == "openarm_base_wrapper"),
        len(worldbody),
    )
    worldbody.insert(first_robot, support)
    worldbody.insert(first_robot + 1, block)
    ET.indent(tree, space="  ")
    output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output, encoding="utf-8", xml_declaration=True)
    encoded = output.read_bytes()
    return {
        "schema_version": "openarm.pick_demo.scene.v8",
        "formal_model": str(source),
        "formal_sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
        "scene_model": str(output),
        "scene_sha256": hashlib.sha256(encoded).hexdigest(),
        "object_id": "pick_demo_block",
        "object_position_m": list(OBJECT_POSITION),
        "object_half_size_m": list(OBJECT_HALF_SIZE),
        "object_free_joint_damping": 0.2,
        "gravity_mps2": list(GRAVITY),
        "support_id": "pick_demo_support",
        "support_position_m": list(SUPPORT_POSITION),
        "support_half_size_m": list(SUPPORT_HALF_SIZE),
        "passive_cradle_guide_count": len(guide_specs),
        "passive_cradle_guide_height_m": 2.0 * GUIDE_HALF_HEIGHT,
        "passive_cradle_clearance_m": GUIDE_CLEARANCE,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    print(json.dumps(build_scene(args.source, args.output), indent=2, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
