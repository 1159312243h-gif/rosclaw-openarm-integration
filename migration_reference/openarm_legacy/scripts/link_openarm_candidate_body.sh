#!/usr/bin/env bash
set -euo pipefail

PROJECT="${OPENARM_PROJECT:-/home/hkz/openarm_rosclaw}"
RUNTIME="${1:?usage: link_openarm_candidate_body.sh CANDIDATE_RUNTIME_YAML}"
PYTHON="${OPENARM_PYTHON:-/home/hkz/rosclaw_env/bin/python}"

RUNTIME="$(readlink -f "$RUNTIME")"
[[ -f "$RUNTIME" ]] || { echo "Missing candidate runtime: $RUNTIME" >&2; exit 1; }
[[ -x "$PYTHON" ]] || { echo "Missing Python: $PYTHON" >&2; exit 1; }

readarray -t PROFILE_VALUES < <("$PYTHON" - "$RUNTIME" <<'PY'
import sys
from pathlib import Path

import yaml

profile = yaml.safe_load(Path(sys.argv[1]).read_text(encoding="ascii"))
print(Path(profile["workspace"]["home"]).resolve())
print(Path(profile["robot"]["zoo_path"]).resolve())
print(profile["robot"]["id"])
PY
)

CANDIDATE_HOME="${PROFILE_VALUES[0]}"
CANDIDATE_ZOO="${PROFILE_VALUES[1]}"
BODY_ID="${PROFILE_VALUES[2]}"
BODY_DIR="$CANDIDATE_ZOO/$BODY_ID"

[[ "$BODY_ID" == "openarm_dual_mujoco_01" ]] || {
  echo "Unexpected candidate Body ID: $BODY_ID" >&2
  exit 1
}
[[ -d "$BODY_DIR" ]] || { echo "Missing candidate Body profile: $BODY_DIR" >&2; exit 1; }
(cd "$BODY_DIR" && sha256sum -c model.sha256)

mkdir -p "$CANDIDATE_HOME"
export ROSCLAW_HOME="$CANDIDATE_HOME"
export ROSCLAW_EURDF_ZOO="$CANDIDATE_ZOO"
export PYTHONPATH="$PROJECT/src${PYTHONPATH:+:$PYTHONPATH}"

if [[ ! -f "$CANDIDATE_HOME/body/body.yaml" ]]; then
  "$PYTHON" -m rosclaw.entrypoint body link-eurdf "$BODY_ID" \
    --workspace "$CANDIDATE_HOME" \
    --instance-id "$BODY_ID" \
    --nickname openarm-candidate-sim
fi

"$PYTHON" -m rosclaw.entrypoint body inspect --json

echo "OPENARM CANDIDATE BODY LINK: PASS"
echo "Candidate runtime: $RUNTIME"
echo "Candidate ROSCLAW_HOME: $CANDIDATE_HOME"
echo "Candidate model zoo: $CANDIDATE_ZOO"
echo "Candidate Body: $BODY_ID"
