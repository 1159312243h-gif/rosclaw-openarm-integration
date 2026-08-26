"""Tests for config and profile CLI commands."""

from __future__ import annotations

import sys

import yaml


class TestConfigCommands:
    def test_config_path(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "firstboot", "--yes", "--profile", "offline", "--no-telemetry"]
        main()

        sys.argv = ["rosclaw", "config", "path"]
        code = main()
        captured = capsys.readouterr()
        assert code == 0
        assert "rosclaw.yaml" in captured.out

    def test_config_show(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "firstboot", "--yes", "--profile", "offline", "--no-telemetry"]
        main()

        sys.argv = ["rosclaw", "config", "show"]
        code = main()
        captured = capsys.readouterr()
        assert code == 0
        assert "workspace:" in captured.out
        assert "runtime:" in captured.out

    def test_config_validate_pass(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "firstboot", "--yes", "--profile", "offline", "--no-telemetry"]
        main()

        sys.argv = ["rosclaw", "config", "validate"]
        code = main()
        captured = capsys.readouterr()
        assert code == 0
        assert "valid" in captured.out.lower()

    def test_config_validate_fail_without_config(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "config", "validate"]
        code = main()
        assert code == 1


class TestProfileCommands:
    def test_profile_current(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "firstboot", "--yes", "--profile", "offline", "--no-telemetry"]
        main()

        sys.argv = ["rosclaw", "profile", "current"]
        code = main()
        captured = capsys.readouterr()
        assert code == 0
        assert "offline" in captured.out

    def test_profile_use_changes_config(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "firstboot", "--yes", "--profile", "offline", "--no-telemetry"]
        main()

        sys.argv = ["rosclaw", "profile", "use", "cloud"]
        code = main()
        captured = capsys.readouterr()
        assert code == 0
        assert "cloud" in captured.out

        cfg = yaml.safe_load((home / "config" / "rosclaw.yaml").read_text(encoding="utf-8"))
        assert cfg["workspace"]["profile"] == "cloud"

        sys.argv = ["rosclaw", "profile", "current"]
        main()
        captured = capsys.readouterr()
        assert captured.out.strip() == "cloud"

    def test_profile_list(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "firstboot", "--yes", "--profile", "offline", "--no-telemetry"]
        main()

        sys.argv = ["rosclaw", "profile", "list"]
        code = main()
        assert code == 0

    def test_profile_use_no_config_fails(self, tmp_path, monkeypatch, capsys):
        home = tmp_path / ".rosclaw"
        monkeypatch.setenv("ROSCLAW_HOME", str(home))
        from rosclaw.cli import main

        sys.argv = ["rosclaw", "profile", "use", "cloud"]
        code = main()
        assert code == 1
