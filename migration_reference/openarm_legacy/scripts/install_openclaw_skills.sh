#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="${PROJECT_ROOT:-$(cd -- "$SCRIPT_DIR/.." && pwd)}"
OPENCLAW_WORKSPACE="${OPENCLAW_WORKSPACE:-/home/hkz/.openclaw/workspace}"
MODE=""
FORCE=0

usage() {
  cat <<'EOF'
Usage: install_openclaw_skills.sh --install [--force]
       install_openclaw_skills.sh --check

Install or verify all project Agent Skills from the official ROSClaw checkout
in the OpenClaw workspace. Existing differing targets require --force.
EOF
}

while (($#)); do
  case "$1" in
    --install|--check)
      [[ -z "$MODE" ]] || { usage >&2; exit 2; }
      MODE="$1"
      ;;
    --force)
      FORCE=1
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

[[ -n "$MODE" ]] || { usage >&2; exit 2; }
[[ "$MODE" == "--install" || "$FORCE" -eq 0 ]] || {
  echo "--force is valid only with --install" >&2
  exit 2
}

SOURCE_ROOT="$PROJECT_ROOT/.agents/skills"
TARGET_ROOT="$OPENCLAW_WORKSPACE/skills"
SKILLS=(rosclaw rosclaw-simforge openarm-move-relative)

for skill in "${SKILLS[@]}"; do
  source_dir="$SOURCE_ROOT/$skill"
  [[ -f "$source_dir/SKILL.md" ]] || {
    echo "Missing source Skill: $source_dir/SKILL.md" >&2
    echo "Run the official ROSClaw agent installer in $PROJECT_ROOT first." >&2
    exit 1
  }
done

if [[ "$MODE" == "--install" ]]; then
  mkdir -p "$TARGET_ROOT"
  TARGET_ROOT="$(cd -- "$TARGET_ROOT" && pwd -P)"
  for skill in "${SKILLS[@]}"; do
    source_dir="$SOURCE_ROOT/$skill"
    target_dir="$TARGET_ROOT/$skill"
    target_differs=0
    if [[ -e "$target_dir" || -L "$target_dir" ]]; then
      if [[ ! -d "$target_dir" ]] \
        || ! diff -qr "$source_dir" "$target_dir" >/dev/null; then
        target_differs=1
      fi
    fi
    if [[ "$target_differs" -eq 1 ]]; then
      if [[ "$FORCE" -ne 1 ]]; then
        echo "Target differs: $target_dir" >&2
        echo "Review it, then rerun with --force if replacement is intended." >&2
        exit 1
      fi
      case "$target_dir" in
        "$TARGET_ROOT"/*) ;;
        *)
          echo "Refusing to replace Skill outside target root: $target_dir" >&2
          exit 1
          ;;
      esac
      rm -rf -- "$target_dir"
    fi
    mkdir -p "$target_dir"
    cp -a "$source_dir/." "$target_dir/"
  done
fi

for skill in "${SKILLS[@]}"; do
  source_dir="$SOURCE_ROOT/$skill"
  target_dir="$TARGET_ROOT/$skill"
  [[ -f "$target_dir/SKILL.md" ]] || {
    echo "OpenClaw workspace Skill missing: $target_dir/SKILL.md" >&2
    exit 1
  }
  diff -qr "$source_dir" "$target_dir" >/dev/null || {
    echo "OpenClaw workspace Skill is stale: $target_dir" >&2
    exit 1
  }
done

export PATH="/home/hkz/.nvm/versions/node/v24.18.0/bin:$PATH"
command -v openclaw >/dev/null || {
  echo "openclaw is not available on PATH" >&2
  exit 1
}

skill_list="$(openclaw skills list)"
printf '%s\n' "$skill_list"
grep -Eq '(^|[[:space:]])rosclaw([[:space:]]|$)' <<<"$skill_list"
grep -Eq '(^|[[:space:]])rosclaw-simforge([[:space:]]|$)' <<<"$skill_list"
grep -Eq '(^|[[:space:]])openarm-move-relative([[:space:]]|$)' <<<"$skill_list"
openclaw skills check

echo "OPENCLAW OFFICIAL ROSCLAW PROJECT SKILLS: PASS"
