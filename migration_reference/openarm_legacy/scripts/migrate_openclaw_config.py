#!/usr/bin/env python3
"""Remove the retired OpenArm provenance plugin from OpenClaw config."""

from __future__ import annotations

import argparse
import json
import os
import tempfile
from pathlib import Path
from typing import Any

STALE_PLUGIN_ID = "openarm-provenance-diagnostic"
STALE_PLUGIN_PATH = (
    "/home/hkz/openarm_rosclaw/openclaw_plugins/openarm-provenance-diagnostic"
)


def migrate_config(config_path: Path) -> bool:
    data: Any = json.loads(config_path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("OpenClaw config root must be a JSON object")

    changed = False
    plugins = data.get("plugins")
    if plugins is None:
        return False
    if not isinstance(plugins, dict):
        raise ValueError("OpenClaw plugins config must be a JSON object")

    load = plugins.get("load")
    if load is not None:
        if not isinstance(load, dict):
            raise ValueError("OpenClaw plugins.load config must be a JSON object")
        paths = load.get("paths")
        if paths is not None:
            if not isinstance(paths, list):
                raise ValueError("OpenClaw plugins.load.paths must be a JSON array")
            retained_paths = [path for path in paths if path != STALE_PLUGIN_PATH]
            if retained_paths != paths:
                load["paths"] = retained_paths
                changed = True

    entries = plugins.get("entries")
    if entries is not None:
        if not isinstance(entries, dict):
            raise ValueError("OpenClaw plugins.entries config must be a JSON object")
        if STALE_PLUGIN_ID in entries:
            del entries[STALE_PLUGIN_ID]
            changed = True

    if not changed:
        return False

    mode = config_path.stat().st_mode
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        dir=config_path.parent,
        prefix=f".{config_path.name}.",
        suffix=".tmp",
        delete=False,
    ) as handle:
        temporary_path = Path(handle.name)
        json.dump(data, handle, ensure_ascii=False, indent=2)
        handle.write("\n")
        handle.flush()
        os.fsync(handle.fileno())

    try:
        os.chmod(temporary_path, mode)
        os.replace(temporary_path, config_path)
    finally:
        temporary_path.unlink(missing_ok=True)
    return True


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, required=True)
    args = parser.parse_args()

    changed = migrate_config(args.config)
    status = "removed" if changed else "already absent"
    print(f"Retired OpenArm provenance plugin config: {status}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
