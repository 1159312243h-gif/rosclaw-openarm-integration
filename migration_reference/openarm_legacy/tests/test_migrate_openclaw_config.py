"""Tests for the retired OpenClaw plugin config migration."""

from __future__ import annotations

import importlib.util
import json
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
SCRIPT = PROJECT_ROOT / "scripts/migrate_openclaw_config.py"
SPEC = importlib.util.spec_from_file_location("migrate_openclaw_config", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_migration_removes_only_retired_openarm_plugin(tmp_path: Path) -> None:
    config_path = tmp_path / "openclaw.json"
    config_path.write_text(
        json.dumps(
            {
                "plugins": {
                    "load": {
                        "paths": [
                            "/opt/openclaw/keep-plugin",
                            MODULE.STALE_PLUGIN_PATH,
                        ]
                    },
                    "entries": {
                        "qwen": {"enabled": True},
                        "memory-core": {},
                        MODULE.STALE_PLUGIN_ID: {"enabled": False},
                    },
                },
                "mcp": {"servers": {"keep": {"command": "example"}}},
            }
        ),
        encoding="utf-8",
    )

    assert MODULE.migrate_config(config_path) is True
    migrated = json.loads(config_path.read_text(encoding="utf-8"))

    assert migrated["plugins"]["load"]["paths"] == [
        "/opt/openclaw/keep-plugin"
    ]
    assert migrated["plugins"]["entries"] == {
        "qwen": {"enabled": True},
        "memory-core": {},
    }
    assert migrated["mcp"] == {"servers": {"keep": {"command": "example"}}}
    assert MODULE.migrate_config(config_path) is False
