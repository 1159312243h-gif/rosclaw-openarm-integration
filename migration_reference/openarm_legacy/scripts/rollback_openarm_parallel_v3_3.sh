#!/usr/bin/env bash
set -euo pipefail

PROJECT="${OPENARM_PROJECT:-/home/hkz/openarm_rosclaw}"
PYTHON="${OPENARM_PYTHON:-/home/hkz/rosclaw_env/bin/python}"
RECORD="${1:-$PROJECT/.rosclaw/promotions/openarm_parallel_v3_3_latest.json}"
MODE="${2:-}"

ARGS=(
  "$PROJECT/scripts/promote_openarm_parallel_v3_3.py"
  --project "$PROJECT"
  --rollback
  --record "$RECORD"
)

if [[ "$MODE" == "--apply" ]]; then
  ARGS+=(--apply)
elif [[ -n "$MODE" ]]; then
  echo "Usage: rollback_openarm_parallel_v3_3.sh [RECORD] [--apply]" >&2
  exit 2
fi

exec "$PYTHON" "${ARGS[@]}"
