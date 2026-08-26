#!/usr/bin/env bash
set -euo pipefail

PROJECT="${OPENARM_PROJECT:-/home/hkz/openarm_rosclaw}"
RUNTIME="${1:?usage: activate_openarm_candidate_mcp.sh CANDIDATE_RUNTIME_YAML}"
PYTHON="${OPENARM_PYTHON:-/home/hkz/rosclaw_env/bin/python}"
OPENCLAW_CONFIG="${OPENCLAW_CONFIG:-/home/hkz/.openclaw/openclaw.json}"
BACKUP_RECORD="$PROJECT/.rosclaw/candidate_openclaw_backup.txt"

RUNTIME="$(readlink -f "$RUNTIME")"
[[ -f "$RUNTIME" ]] || { echo "Missing candidate runtime: $RUNTIME" >&2; exit 1; }
[[ -f "$OPENCLAW_CONFIG" ]] || { echo "Missing OpenClaw config: $OPENCLAW_CONFIG" >&2; exit 1; }
command -v openclaw >/dev/null || { echo "openclaw is not on PATH" >&2; exit 1; }
CANDIDATE_HOME="$($PYTHON - "$RUNTIME" <<'PY'
import sys
from pathlib import Path

import yaml

profile = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="ascii"))
print(Path(profile["workspace"]["home"]).resolve())
PY
)"
CANDIDATE_ZOO="$($PYTHON - "$RUNTIME" <<'PY'
import sys
from pathlib import Path

import yaml

profile = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="ascii"))
print(Path(profile["robot"]["zoo_path"]).resolve())
PY
)"
BODY_ID="$($PYTHON - "$RUNTIME" <<'PY'
import sys
from pathlib import Path

import yaml

profile = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="ascii"))
print(profile["robot"]["id"])
PY
)"
mkdir -p "$CANDIDATE_HOME"
[[ -d "$CANDIDATE_ZOO/$BODY_ID" ]] || {
  echo "Missing candidate Body profile: $CANDIDATE_ZOO/$BODY_ID" >&2
  exit 1
}

if [[ ! -f "$CANDIDATE_HOME/body/body.yaml" ]]; then
  ROSCLAW_HOME="$CANDIDATE_HOME" \
  ROSCLAW_EURDF_ZOO="$CANDIDATE_ZOO" \
  PYTHONPATH="$PROJECT/src${PYTHONPATH:+:$PYTHONPATH}" \
    "$PYTHON" -m rosclaw.entrypoint body link-eurdf "$BODY_ID" \
      --workspace "$CANDIDATE_HOME" \
      --instance-id "$BODY_ID" \
      --nickname openarm-candidate-sim
fi

ROSCLAW_HOME="$CANDIDATE_HOME" \
ROSCLAW_EURDF_ZOO="$CANDIDATE_ZOO" \
PYTHONPATH="$PROJECT/src${PYTHONPATH:+:$PYTHONPATH}" \
  "$PYTHON" -m rosclaw.entrypoint body inspect --json >/dev/null

BACKUP="$OPENCLAW_CONFIG.before-openarm-candidate-$(date +%Y%m%d_%H%M%S)"
cp -a "$OPENCLAW_CONFIG" "$BACKUP"
printf '%s\n' "$BACKUP" >"$BACKUP_RECORD"

restore_on_error() {
  cp -a "$BACKUP" "$OPENCLAW_CONFIG"
  openclaw gateway restart >/dev/null 2>&1 || true
  echo "Candidate MCP activation failed; restored $OPENCLAW_CONFIG" >&2
}
trap restore_on_error ERR

openclaw mcp unset rosclaw >/dev/null || true
openclaw mcp add rosclaw \
  --command "$PYTHON" \
  --arg=-m \
  --arg=rosclaw.entrypoint \
  --arg=mcp \
  --arg=serve \
  --arg=--profile \
  --arg="$RUNTIME" \
  --arg=--project \
  --arg="$PROJECT" \
  --arg=--log-level \
  --arg=ERROR \
  --cwd "$PROJECT" \
  --env PYTHONPATH="$PROJECT/src" \
  --env ROSCLAW_AGENT_CLIENT=openclaw \
  --env ROSCLAW_HOME="$CANDIDATE_HOME" \
  --env ROSCLAW_EURDF_ZOO="$CANDIDATE_ZOO" \
  --env ROSCLAW_MCP_AUDIT=1 \
  --connect-timeout 20 \
  --timeout 300
openclaw gateway restart
openclaw mcp probe rosclaw
trap - ERR

echo "Candidate MCP active: $RUNTIME"
echo "Candidate ROSCLAW_HOME: $CANDIDATE_HOME"
echo "Candidate Body linked: $BODY_ID"
echo "Restore backup: $BACKUP"
