# ROSClaw v1.0 发布后全面审计报告

**Report Date**: 2026-06-07
**Auditor**: Claude Opus 4.8
**Scope**: README.md, README.zh.md, CLI commands, pyproject.toml, file system, actual code behavior
**Repository**: https://github.com/ros-claw/rosclaw (main branch)
**Latest Commit**: 9039cc8 + 8bff799 (issue fixes)

---

## Executive Summary

After the v1.0 release, community user @coolbeevip immediately reported 3 issues + 1 PR. A comprehensive audit revealed 11 additional documentation/code inconsistencies between README promises and actual CLI behavior. These are categorized as:

| Severity | Count | Category |
|----------|-------|----------|
| 🔴 Critical | 5 | README commands/args that do not exist in CLI |
| 🟠 Medium | 2 | Files referenced in README that do not exist |
| 🟡 Minor | 4 | Documentation inaccuracies |

---

## Part 1: Community-Reported Issues (Fixed)

### Issue #3 — rosclaw doctor reports pytest missing after install

**Reporter**: @coolbeevip
**Status**: ✅ Fixed in commit 8bff799
**Root Cause**: scripts/install.sh installed only base package (pip install -e ".") but rosclaw doctor checks pytest as a required dependency.
**Fix**: Changed install.sh line 95 to pip install -e ".[dev]" -q, installing dev dependencies by default.
**Verification**:
```bash
# Before fix: pytest MISSING warning
# After fix: pytest 8.4.2 ✅
```

### Issue #4 — sandbox run --world argument unsupported

**Reporter**: @coolbeevip
**Status**: ✅ Fixed in commit 8bff799
**Root Cause**: CLI argparse did not define --world for sandbox run subcommand.
**Fix**: Added --world argument to sandbox_run_parser (default "empty"), passed world_id to SandboxRuntimeAdapter config.
**Verification**:
```bash
$ rosclaw sandbox run --robot ur5e --world tabletop --task reach
[ROSClaw] Running sandbox episode: robot=ur5e, world=tabletop, task=reach
...
World:      tabletop
Backend:    mujoco
Status:     success
Exit code: 0 ✅
```

### Issue #6 — firewall check --world argument unsupported

**Reporter**: @coolbeevip
**Status**: ✅ Fixed in commit 8bff799
**Root Cause**: CLI argparse did not define --world for firewall check subcommand.
**Fix**: Added --world argument to firewall_check_parser (default "empty"), used in cmd_firewall_check.
**Verification**:
```bash
$ rosclaw firewall check --robot ur5e --world tabletop --action grasp
[ROSClaw] Firewall check: robot=ur5e, world=tabletop, action=grasp
Decision:   ALLOW
Exit code: 0 ✅
```

### PR #5 — Fix sandbox run --world argument

**Author**: @coolbeevip
**Status**: ✅ Superseded by commit 8bff799
**Action**: Credited in commit message. Equivalent fix merged directly.

---

## Part 2: Comprehensive Audit Findings (11 New Issues)

### 🔴 Critical Issue 1: rosclaw auto command does not exist

**Location**: README.md line 540, README.zh.md line 565
**What README says**:
```bash
./rosclaw auto run --task pick_cube --episodes 50
./rosclaw skill champions list
./rosclaw skill lineage pick_cube
./rosclaw skill rollback pick_cube --to v1.0.0
```

**What actually happens**:
```text
$ rosclaw auto
rosclaw: error: argument command: invalid choice: 'auto'

$ rosclaw skill
usage: rosclaw skill [-h] {list,invoke} ...
# No champions/lineage/rollback subcommands
```

**Root Cause**:
- src/rosclaw/auto/cli.py exists as a standalone CLI (rosclaw-auto), not integrated into main rosclaw CLI
- src/rosclaw/skill_manager/registry.py has list_champions(), list_lineage(), rollback() methods, but these are not exposed in rosclaw/cli.py

**Correct commands**:
```bash
rosclaw-auto run --task pick_cube --rounds 50   # NOT --episodes
rosclaw-auto champion --task pick_cube          # NOT skill champions
# list_lineage/rollback have NO CLI entry point at all
```

**Impact**: High. README prominently advertises auto evolution and skill management as CLI features, but users cannot execute them.

---

### 🔴 Critical Issue 2: rosclaw demo tabletop_pick demo name wrong

**Location**: README.md line 418, README.zh.md line 490
**What README says**:
```bash
./rosclaw demo tabletop_pick --robot ur5e
```

**What actually happens**:
```text
$ rosclaw demo
usage: rosclaw demo [-h] {mobile-pid,tabletop-grasp} ...
rosclaw: error: argument demo_command: invalid choice: 'tabletop_pick'
```

**Root Cause**: The demo is named tabletop-grasp, not tabletop_pick. Additionally, --robot is not a valid argument; the correct argument is --robot-id.

**Correct command**:
```bash
rosclaw demo tabletop-grasp --robot-id ur5e
```

**Impact**: High. This is a featured demo example in README.

---

### 🔴 Critical Issue 3: rosclaw doctor --architecture flag does not exist

**Location**: README.md line 760, README.zh.md line 783
**What README says**:
```bash
./rosclaw doctor --architecture
```

**What actually happens**:
```text
$ rosclaw doctor --architecture
rosclaw: error: unrecognized arguments: --architecture
```

**Root Cause**: cmd_doctor only defines --ros2 flag. No --architecture flag exists.

**Correct command**:
```bash
rosclaw doctor          # Basic health check
rosclaw doctor --ros2   # With ROS2 environment check
```

**Impact**: Medium. This is presented as a development/architecture validation command.

---

### 🔴 Critical Issue 4: rosclaw forge sdk-to-mcp wrong arguments

**Location**: README.md lines 431-436, README.zh.md lines 591-594
**What README says**:
```bash
./rosclaw forge sdk-to-mcp \
  --robot unitree_go2 \
  --sdk-docs ./docs/unitree_go2_sdk.md \
  --eurdf ./e-urdf-zoo/unitree_go2 \
  --output ./generated/unitree_go2_bundle
```

**What actually happens**:
```text
$ rosclaw forge sdk-to-mcp --robot unitree_go2
rosclaw: error: unrecognized arguments: --robot
```

**Root Cause**: forge sdk-to-mcp only accepts --name, --sdk-docs, and --output. No --robot or --eurdf parameters.

**Correct command**:
```bash
rosclaw forge sdk-to-mcp \
  --name unitree_go2 \
  --sdk-docs ./docs/unitree_go2_sdk.md \
  --output ./generated/unitree_go2_bundle
```

**Impact**: Medium. Featured in the SDK-to-MCP / Asset Forge section.

---

### 🔴 Critical Issue 5: rosclaw auto run --episodes is wrong parameter name

**Location**: README.md line 540, README.zh.md line 565
**What README says**:
```bash
./rosclaw auto run --task pick_cube --episodes 50
```

**Root Cause**: Even if rosclaw auto existed, the parameter is --rounds, not --episodes.

**Verification from rosclaw-auto run --help**:
```text
usage: rosclaw-auto run [-h] --task TASK [--rounds ROUNDS] [--dry-run] [--policy POLICY]
```

**Correct command**:
```bash
rosclaw-auto run --task pick_cube --rounds 50
```

**Impact**: Medium. Parameter name mismatch.

---

### 🟠 Medium Issue 6: examples/actions/unsafe_reach.json does not exist

**Location**: README.md line 438, README.zh.md line 458
**What README says**:
```bash
./rosclaw firewall check \
  --robot ur5e \
  --world tabletop \
  --action examples/actions/unsafe_reach.json
```

**File check**:
```bash
$ ls examples/actions/unsafe_reach.json
ls: cannot access 'examples/actions/unsafe_reach.json': No such file or directory
```

**Root Cause**: examples/actions/ directory does not exist. Only examples/ contains .py demo scripts.

**Impact**: Medium. Firewall check example references a non-existent file.

---

### 🟠 Medium Issue 7: docs/unitree_go2_sdk.md does not exist

**Location**: README.md line 433, README.zh.md line 592
**What README says**:
```bash
--sdk-docs ./docs/unitree_go2_sdk.md
```

**File check**:
```bash
$ ls docs/unitree_go2_sdk.md
ls: cannot access 'docs/unitree_go2_sdk.md': No such file or directory
```

**Root Cause**: This file was never created. The forge example references a placeholder path.

**Impact**: Medium. Featured in Asset Forge section.

---

### 🟡 Minor Issue 8: MCP configuration example points to non-existent module

**Location**: README.md lines 450-458, README.zh.md lines 472-480
**What README says**:
```json
{
  "mcpServers": {
    "rosclaw": {
      "command": "python3",
      "args": ["-m", "rosclaw.mcp.server"],
      "env": { "PYTHONPATH": "src" }
    }
  }
}
```

**File check**:
```bash
$ ls src/rosclaw/mcp/server.py
ls: cannot access 'src/rosclaw/mcp/server.py': No such file or directory
```

**Root Cause**: rosclaw.mcp.server module does not exist. The actual modules are:
- rosclaw.mcp.minimal_server (generic MCP server)
- rosclaw.mcp.ur5_server (UR5-specific MCP server)

**pyproject.toml scripts**:
```toml
rosclaw-mcp = "rosclaw.mcp.minimal_server:main"
rosclaw-ur5-mcp = "rosclaw.mcp.ur5_server:main"
```

**Correct configuration**:
```json
{
  "mcpServers": {
    "rosclaw": {
      "command": "rosclaw-mcp"
    }
  }
}
```

**Impact**: Low-Medium. Users copying this config will get a ModuleNotFoundError.

---

### 🟡 Minor Issue 9: MCP tool names listed in README are fictional

**Location**: README.md line 459
**What README says**:
> Exposes tools such as: move_joints, grasp, get_robot_state, validate_trajectory, emergency_stop, query_world_objects, get_scene_graph, cognitive_search, system.list_robots, system.run_sandbox_task, system.query_practice, system.query_memory

**Actual tools exposed** (from src/rosclaw/mcp/minimal_server.py):

| # | Tool Name | In README? |
|---|-----------|------------|
| 1 | system.list_robots | ✅ Correct |
| 2 | system.list_providers | ❌ Missing |
| 3 | system.run_sandbox_task | ✅ Correct |
| 4 | system.query_memory | ✅ Correct |
| 5 | system.explain_failure | ❌ Missing |
| 6 | system.compile_asset_bundle | ❌ Missing |
| 7 | system.get_version | ❌ Missing |

**Fictional tools in README**:
- move_joints — does not exist
- grasp — does not exist
- get_robot_state — does not exist
- validate_trajectory — does not exist
- emergency_stop — does not exist
- query_world_objects — does not exist
- get_scene_graph — does not exist
- cognitive_search — does not exist
- system.query_practice — does not exist

**Impact**: Medium. 9 out of 12 listed tools are fictional.

---

### 🟡 Minor Issue 10: rosclaw.yaml config example structure does not match actual config

**Location**: README.md lines 627-660, README.zh.md lines 650-699

**What README shows**:
```yaml
runtime:
  robot_id: ur5e
  safety_level: strict
event_bus:
  backend: local
knowledge_plane:
  backend: seekdb
  path: .rosclaw/seekdb
object_store:
  backend: local
  path: .rosclaw/artifacts
sandbox:
  enabled: true
  backend: mujoco
  firewall_mode: true
provider:
  enabled: true
practice:
  enabled: true
  mcap: true
memory:
  enabled: true
how:
  enabled: true
  cooldown_window: 3
  evidence_trace_enabled: true
auto:
  enabled: true
  allow_code_patch: false
  require_human_approval: true
  trigger_failure_threshold: 3
darwin:
  enabled: true
  seeds: [0, 1, 2]
  episodes: 50
  metrics: [success_rate, collision_rate, completion_time]
```

**Actual rosclaw.yaml**:
```yaml
robot_id: rosclaw_bot
safety_level: MODERATE
robot_model_path: ""
enable_firewall: true
enable_memory: true
enable_practice: true
enable_auto: true
enable_darwin: true
enable_know: true
enable_how: true
enable_swarm: false

llm:
  provider: deepseek
  model: deepseek-chat

practice:
  output_dir: ./practice_data
  auto_record: true

memory:
  backend: memory

auto:
  enabled: true
  trigger:
    repeated_failure_threshold: 3
    min_failure_severity: medium
  patch_policy:
    allow_config_patch: true
    allow_code_patch: false
    require_human_approval_for_code: true
  store_path: ./.rosclaw_auto

darwin:
  enabled: true
  default_seeds: 10
  default_episodes: 50
  metrics:
    - success_rate
```

**Key inconsistencies**:

| README Key | Actual Key | Status |
|-----------|-----------|--------|
| runtime: robot_id: | robot_id: (top-level) | ❌ Different nesting |
| runtime: safety_level: | safety_level: (top-level) | ❌ Different nesting |
| event_bus: backend: | Does not exist | ❌ Missing |
| knowledge_plane: | Does not exist | ❌ Missing |
| object_store: | Does not exist | ❌ Missing |
| sandbox: enabled: | enable_firewall: | ❌ Different key name |
| sandbox: backend: | Does not exist | ❌ Missing |
| sandbox: firewall_mode: | enable_firewall: | ❌ Different key |
| provider: enabled: | Does not exist | ❌ Missing |
| practice: enabled: | practice: output_dir: | ❌ Different structure |
| practice: mcap: | Does not exist | ❌ Missing |
| memory: enabled: | memory: backend: | ❌ Different structure |
| how: cooldown_window: | Does not exist | ❌ Missing |
| how: evidence_trace_enabled: | Does not exist | ❌ Missing |
| auto: allow_code_patch: | auto: patch_policy: allow_code_patch: | ❌ Different nesting |
| auto: require_human_approval: | auto: patch_policy: require_human_approval_for_code: | ❌ Different nesting |
| auto: trigger_failure_threshold: | auto: trigger: repeated_failure_threshold: | ❌ Different nesting |
| darwin: seeds: | darwin: default_seeds: | ❌ Different key name |
| darwin: episodes: | darwin: default_episodes: | ❌ Different key name |

**Impact**: Medium. Users copying the README config will get an invalid rosclaw.yaml.

---

### 🟡 Minor Issue 11: Test execution guidance inconsistent

**Location**: README.md lines 748-760, README.zh.md lines 765-780
**What README says**:
```bash
PYTHONPATH=src pytest tests -v
PYTHONPATH=src pytest tests/test_e2e_full_pipeline.py -v
```

**Issue**: README earlier recommends bash scripts/install.sh which creates a virtual environment. After installation, users should use the venv's pytest, not PYTHONPATH=src. This creates confusion about whether the venv is needed.

**Correct guidance** (after scripts/install.sh):
```bash
source venv/bin/activate
pytest tests/ -v
```

**Impact**: Low. Functional but inconsistent user guidance.

---

## Part 3: Additional Observations

### Commit Message Pollution

Commit 8bff799 has a corrupted commit message. When the commit was created, the --world text in the message was interpreted as a shell command argument, causing the rosclaw doctor output to be embedded inside the commit message.

Note: This does not affect functionality, but pollutes git history.

### Auto Module is Isolated

The auto module (src/rosclaw/auto/) has its own standalone CLI (rosclaw-auto) with 7 subcommands (init, run, status, champion, deadends, report, repair), but none of these are wired into the main rosclaw CLI entry point. This is a design decision, but README incorrectly presents them as part of the main CLI.

### Skill Registry Methods Exist But Not Wired to CLI

SkillRegistry has promote(), get_champion(), list_champions(), list_lineage(), rollback() methods. These work at the code level but have no CLI entry points.

---

## Part 4: Recommended Fix Priority

### Priority P0 — Fix immediately (user-facing broken examples)

| # | Issue | Fix Approach | Effort |
|---|-------|-------------|--------|
| 1 | rosclaw auto does not exist | Replace README examples with rosclaw-auto correct syntax | Low |
| 2 | rosclaw demo tabletop_pick wrong | Fix to tabletop-grasp --robot-id | Low |
| 3 | rosclaw skill champions/lineage/rollback missing | Either remove from README OR add CLI subcommands | Medium |
| 4 | rosclaw doctor --architecture | Remove from README | Low |
| 5 | rosclaw forge --robot/--eurdf | Fix to --name only | Low |

### Priority P1 — Fix soon (missing files)

| # | Issue | Fix Approach | Effort |
|---|-------|-------------|--------|
| 6 | examples/actions/unsafe_reach.json missing | Create a sample unsafe action JSON | Low |
| 7 | docs/unitree_go2_sdk.md missing | Create a sample SDK doc OR fix example path | Low |
| 8 | MCP config rosclaw.mcp.server | Fix to rosclaw-mcp or rosclaw.mcp.minimal_server | Low |

### Priority P2 — Fix when convenient (documentation polish)

| # | Issue | Fix Approach | Effort |
|---|-------|-------------|--------|
| 9 | MCP tool names fictional | Replace with actual 7 tool names | Low |
| 10 | rosclaw.yaml structure mismatch | Rewrite README example to match actual config | Medium |
| 11 | Test guidance inconsistent | Standardize to venv-based pytest commands | Low |

---

## Part 5: Verification Commands

To verify any fix, use these commands:

```bash
# Verify auto CLI exists
rosclaw-auto --help

# Verify sandbox --world
rosclaw sandbox run --robot ur5e --world tabletop --task reach

# Verify firewall --world
rosclaw firewall check --robot ur5e --world tabletop --action grasp

# Verify skill subcommands
rosclaw skill --help

# Verify demo subcommands
rosclaw demo --help

# Verify forge args
rosclaw forge sdk-to-mcp --help

# Verify doctor args
rosclaw doctor --help

# Verify MCP server module
python3 -m rosclaw.mcp.minimal_server --help

# Check file existence
ls examples/actions/unsafe_reach.json
ls docs/unitree_go2_sdk.md
```

---

**Report compiled by**: Claude Opus 4.8
**Date**: 2026-06-07
**Repository**: ros-claw/rosclaw
**Commit audited**: 8bff799 (post-issue-fix state)
