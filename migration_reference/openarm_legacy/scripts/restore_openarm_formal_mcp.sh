#!/usr/bin/env bash
set -euo pipefail

PROJECT="${OPENARM_PROJECT:-/home/hkz/openarm_rosclaw}"
OPENCLAW_CONFIG="${OPENCLAW_CONFIG:-/home/hkz/.openclaw/openclaw.json}"
BACKUP_RECORD="$PROJECT/.rosclaw/candidate_openclaw_backup.txt"

[[ -f "$BACKUP_RECORD" ]] || { echo "No candidate MCP backup record: $BACKUP_RECORD" >&2; exit 1; }
BACKUP="$(head -n 1 "$BACKUP_RECORD")"
[[ -f "$BACKUP" ]] || { echo "Recorded OpenClaw backup is missing: $BACKUP" >&2; exit 1; }

cp -a "$BACKUP" "$OPENCLAW_CONFIG"
openclaw gateway restart
openclaw mcp probe rosclaw
echo "Formal MCP configuration restored from: $BACKUP"
