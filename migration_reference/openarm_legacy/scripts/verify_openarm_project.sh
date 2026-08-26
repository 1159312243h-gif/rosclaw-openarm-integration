#!/usr/bin/env bash
set -euo pipefail

MODE="${1:-}"
if [[ -n "$MODE" && "$MODE" != "--cutover" ]]; then
  echo "Usage: verify_openarm_project.sh [--cutover]" >&2
  exit 2
fi

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
VENV="${VENV:-/home/hkz/rosclaw_env}"
PYTHON="$VENV/bin/python"

[[ -x "$PYTHON" ]] || { echo "Missing Python: $PYTHON" >&2; exit 1; }
source "$VENV/bin/activate"
export PYTHONPATH="$PROJECT_ROOT/src${PYTHONPATH:+:$PYTHONPATH}"
export ROSCLAW_HOME="${ROSCLAW_HOME:-$PROJECT_ROOT/.rosclaw}"
export ROSCLAW_EURDF_ZOO="$PROJECT_ROOT/models"
export PROJECT_ROOT

cd "$PROJECT_ROOT"
mkdir -p "$ROSCLAW_HOME"

(cd models/openarm_dual_mujoco_01 && sha256sum -c model.sha256)

"$PYTHON" - <<'PY'
import inspect
import os
from pathlib import Path
import rosclaw

project = Path(os.environ["PROJECT_ROOT"]).resolve()
package = Path(inspect.getfile(rosclaw)).resolve()
print("rosclaw package:", package)
if project not in package.parents:
    raise SystemExit(f"ROSClaw is not loaded from {project}")

skill = project / ".agents/skills/openarm-move-relative/SKILL.md"
text = skill.read_text(encoding="utf-8")
frontmatter = text.split("---", 2)[1]
description = next(
    line.split(":", 1)[1].strip()
    for line in frontmatter.splitlines()
    if line.startswith("description:")
)
if len(description) > 160:
    raise SystemExit("Agent Skill description exceeds 160 characters")
for signal in (
    "OpenArm",
    "左臂",
    "右臂",
    "move",
    "向上",
    "毫米",
    "X/Y/Z",
    "MoveIt",
    "MuJoCo",
):
    if signal.casefold() not in description.casefold():
        raise SystemExit(f"Agent Skill routing signal missing: {signal}")
print("OpenArm Agent Skill format: PASS")

import yaml

profile = yaml.safe_load((project / "runtime.yaml").read_text(encoding="utf-8"))
moveit = profile.get("sandbox", {}).get("moveit", {})
if moveit.get("required") is not True:
    raise SystemExit("runtime profile must require MoveIt")
worker = project / str(moveit.get("worker_command", ""))
if not worker.is_file():
    raise SystemExit(f"MoveIt worker command is missing: {worker}")
print("OpenArm MoveIt runtime profile: PASS")
PY

"$PYTHON" -m rosclaw.entrypoint agent install openclaw \
  --project-root "$PROJECT_ROOT" \
  --profile "$PROJECT_ROOT/runtime.yaml" \
  --skip-secrets

if [[ ! -f "$ROSCLAW_HOME/body/body.yaml" ]]; then
  "$PYTHON" -m rosclaw.entrypoint body link-eurdf openarm_dual_mujoco_01 \
    --workspace "$ROSCLAW_HOME" \
    --instance-id openarm_dual_mujoco_01 \
    --nickname openarm-sim
fi

"$PYTHON" -m rosclaw.entrypoint body inspect --json
"$PYTHON" -m rosclaw.entrypoint skill validate \
  "$PROJECT_ROOT/skills/openarm-move-relative" --json

"$PYTHON" -m ruff check \
  scripts/migrate_openclaw_config.py \
  src/rosclaw/sandbox/openarm.py \
  src/rosclaw/sandbox/openarm_moveit.py \
  src/rosclaw/sandbox/runtime_adapter.py \
  src/rosclaw/sandbox/sandbox_api.py \
  src/rosclaw/mcp/adapters/runtime_client.py \
  tests/sandbox/test_openarm.py \
  tests/sandbox/test_openarm_moveit.py \
  tests/test_migrate_openclaw_config.py \
  tests/mcp/adapters/test_runtime_client.py \
  tests/integration/test_openarm_official_acceptance.py

"$PYTHON" -m ruff check \
  ros_ws/src/rosclaw_openarm_bringup/scripts/mujoco_moveit_bridge.py \
  ros_ws/src/rosclaw_openarm_bringup/scripts/rosclaw_moveit_action_worker.py \
  ros_ws/src/rosclaw_openarm_bringup/launch/openarm_sim.launch.py

"$PYTHON" -m pytest \
  tests/sandbox/test_openarm.py \
  tests/sandbox/test_openarm_moveit.py \
  tests/test_migrate_openclaw_config.py \
  tests/mcp/adapters/test_runtime_client.py \
  tests/mcp/test_schemas.py \
  -q

chmod +x scripts/run_openarm_moveit_worker.sh scripts/verify_openarm_moveit.sh
scripts/verify_openarm_moveit.sh

if [[ "$MODE" == "--cutover" ]]; then
  OPENCLAW_CONFIG="${OPENCLAW_CONFIG:-/home/hkz/.openclaw/openclaw.json}"
  STALE_PLUGIN_BACKUP="${OPENCLAW_CONFIG}.before-retired-openarm-plugin-$(date +%Y%m%d_%H%M%S)"
  cp -a "$OPENCLAW_CONFIG" "$STALE_PLUGIN_BACKUP"
  echo "OpenClaw config backup before retired-plugin migration: $STALE_PLUGIN_BACKUP"
  "$PYTHON" scripts/migrate_openclaw_config.py --config "$OPENCLAW_CONFIG"
fi

chmod +x scripts/install_openclaw_skills.sh
scripts/install_openclaw_skills.sh --install --force
scripts/install_openclaw_skills.sh --check

"$PYTHON" -m rosclaw.entrypoint agent doctor openclaw \
  --project-root "$PROJECT_ROOT"
# The official generic MCP probe executes the bundled UR5e product demo.
# OpenArm execution is verified above; canonical tool discovery is probed below.
"$PYTHON" -m rosclaw.entrypoint agent test openclaw \
  --project-root "$PROJECT_ROOT" --quick

if [[ "$MODE" == "--cutover" ]]; then
  export PATH="${OPENCLAW_NODE_BIN:-/home/hkz/.nvm/versions/node/v24.18.0/bin}:$PATH"
  command -v openclaw >/dev/null || {
    echo "openclaw is not available on PATH" >&2
    exit 1
  }

  "$PYTHON" -m pip install --no-deps -e "$PROJECT_ROOT"
  INSTALLED_PACKAGE="$(
    env -u PYTHONPATH "$PYTHON" -c \
      'import inspect, rosclaw; print(inspect.getfile(rosclaw))'
  )"
  case "$INSTALLED_PACKAGE" in
    "$PROJECT_ROOT"/src/rosclaw/*) ;;
    *)
      echo "Editable ROSClaw install resolved outside $PROJECT_ROOT: $INSTALLED_PACKAGE" >&2
      exit 1
      ;;
  esac
  echo "Editable ROSClaw install: $INSTALLED_PACKAGE"

  BACKUP="$OPENCLAW_CONFIG.before-official-openarm-$(date +%Y%m%d_%H%M%S)"
  cp -a "$OPENCLAW_CONFIG" "$BACKUP"
  echo "OpenClaw config backup: $BACKUP"

  openclaw mcp unset rosclaw >/dev/null || true
  if ! openclaw mcp add rosclaw \
    --command "$PYTHON" \
    --arg=-m \
    --arg=rosclaw.entrypoint \
    --arg=mcp \
    --arg=serve \
    --arg=--profile \
    --arg="$PROJECT_ROOT/runtime.yaml" \
    --arg=--project \
    --arg="$PROJECT_ROOT" \
    --arg=--log-level \
    --arg=ERROR \
    --cwd "$PROJECT_ROOT" \
    --env ROSCLAW_AGENT_CLIENT=openclaw \
    --env ROSCLAW_HOME="$ROSCLAW_HOME" \
    --env ROSCLAW_MCP_AUDIT=1 \
    --connect-timeout 20 \
    --timeout 300; then
    cp -a "$BACKUP" "$OPENCLAW_CONFIG"
    echo "MCP cutover failed; restored $OPENCLAW_CONFIG" >&2
    exit 1
  fi

  openclaw mcp probe rosclaw
  openclaw gateway restart
  openclaw gateway status
fi

echo "OPENARM ROSCLAW PROJECT: PASS"
