"""Tests for structured doctor JSON output."""

from __future__ import annotations

import json
import sys


class TestDoctorJson:
    def test_doctor_bootstrap_json_schema(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "doctor", "--bootstrap", "--json"]
        code = main()
        captured = capsys.readouterr()
        data = json.loads(captured.out)

        assert "status" in data
        assert "exit_code" in data
        assert "checks" in data
        assert data["exit_code"] == code

        for check in data["checks"]:
            assert "id" in check
            assert "name" in check
            assert "status" in check
            assert "required" in check
            assert "message" in check
            assert "fix" in check

        check_ids = {c["id"] for c in data["checks"]}
        assert "core.cli" in check_ids
        assert "core.python" in check_ids
        assert "core.workspace" in check_ids
        assert "core.config_dir" in check_ids

    def test_doctor_full_json_contains_optional_checks(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "doctor", "--full", "--json"]
        code = main()
        captured = capsys.readouterr()
        data = json.loads(captured.out)

        assert data["exit_code"] == code
        statuses = {c["status"] for c in data["checks"]}
        assert statuses.issubset({"PASS", "WARN", "FAIL", "SKIP"})

        check_ids = {c["id"] for c in data["checks"]}
        assert any("optional." in cid for cid in check_ids)

    def test_doctor_fix_creates_missing_dirs(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "doctor", "--full", "--fix", "--json"]
        code = main()
        captured = capsys.readouterr()
        data = json.loads(captured.out)

        assert data["exit_code"] == code
        config_dir_check = next((c for c in data["checks"] if c["id"] == "core.config_dir"), None)
        if config_dir_check is not None:
            assert config_dir_check["status"] == "PASS"
            assert config_dir_check["message"] == "created"
        assert (home / "config").exists()

    def test_doctor_fix_regenerates_default_configs_and_path_shim(
        self, tmp_path, monkeypatch, capsys
    ):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        # Force the shim branch by pretending rosclaw is not on PATH.
        monkeypatch.setattr(
            "rosclaw.firstboot.doctor.shutil.which",
            lambda cmd: None if cmd == "rosclaw" else f"/usr/bin/{cmd}",
        )
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "doctor", "--full", "--fix", "--json"]
        code = main()
        captured = capsys.readouterr()
        data = json.loads(captured.out)

        assert data["exit_code"] == code
        assert (home / "config" / "rosclaw.yaml").exists()
        assert (home / "config" / "mcp.json").exists()
        assert (home / "config" / "telemetry.yaml").exists()
        assert (home / "state" / "install.json").exists()
        assert (home / "state" / "workspace.json").exists()
        assert (home / "bin" / "rosclaw").exists()

        cli_check = next((c for c in data["checks"] if c["id"] == "core.cli"), None)
        assert cli_check is not None
        assert cli_check["status"] == "WARN"
        assert "shim created" in cli_check["message"]

        install_check = next((c for c in data["checks"] if c["id"] == "core.install_json"), None)
        assert install_check is not None
        assert install_check["status"] == "PASS"
        assert install_check["message"] == "regenerated"
