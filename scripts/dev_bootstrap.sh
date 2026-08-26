#!/usr/bin/env bash
# ROSClaw Developer Bootstrap
# Run from the repository root after cloning.

set -euo pipefail

python3 -m venv .venv
# shellcheck disable=SC1091
. .venv/bin/activate

python -m pip install --upgrade pip wheel setuptools
python -m pip install -e ".[dev]"

export ROSCLAW_HOME="${ROSCLAW_HOME:-$PWD/.rosclaw}"
rosclaw firstboot --dev --workspace "$ROSCLAW_HOME" --profile offline --no-telemetry --yes
rosclaw doctor --full
