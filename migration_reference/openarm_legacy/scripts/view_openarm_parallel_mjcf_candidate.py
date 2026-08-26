#!/usr/bin/env python3
"""Animate a parallel-gripper MJCF candidate in the native MuJoCo viewer."""

from __future__ import annotations

import argparse
import math
import time
from pathlib import Path

import mujoco
from mujoco import viewer


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("model", type=Path)
    parser.add_argument("--period", type=float, default=6.0)
    args = parser.parse_args()

    model = mujoco.MjModel.from_xml_path(str(args.model.resolve()))
    data = mujoco.MjData(model)
    qpos_addresses = []
    for side in ("left", "right"):
        for number in (1, 2):
            name = f"openarm_{side}_finger_joint{number}"
            joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise ValueError(f"missing joint: {name}")
            qpos_addresses.append(int(model.jnt_qposadr[joint_id]))

    with viewer.launch_passive(model, data) as handle:
        handle.cam.lookat[:] = (0.0, 0.0, 0.78)
        handle.cam.distance = 1.35
        handle.cam.azimuth = 135.0
        handle.cam.elevation = -18.0
        start = time.monotonic()
        previous_bucket = -1
        while handle.is_running():
            elapsed = time.monotonic() - start
            phase = 0.5 - 0.5 * math.cos(2.0 * math.pi * elapsed / args.period)
            position = 0.044 * phase
            for address in qpos_addresses:
                data.qpos[address] = position
            data.qvel[:] = 0.0
            mujoco.mj_forward(model, data)
            handle.sync()
            bucket = int(elapsed * 2.0)
            if bucket != previous_bucket:
                state = "CLOSING" if math.sin(2.0 * math.pi * elapsed / args.period) >= 0 else "OPENING"
                print(f"{state}: driver_joint={position * 1000.0:.2f} mm")
                previous_bucket = bucket
            time.sleep(1.0 / 60.0)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
