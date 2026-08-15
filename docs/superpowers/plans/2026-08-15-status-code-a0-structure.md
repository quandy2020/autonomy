# StatusCode A0 Structure Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Restructure `StatusCode` into thin / fine / DEPRECATED sections, unify `CANCELLED` spelling, and sync docs — without changing numeric values or migrating call sites.

**Architecture:** Single enum in `status_msgs.proto`; declaration order becomes (1) all thin codes, (2) all fine codes, (3) deprecated aliases. Canonical cancel names use `CANCELLED`; `TASK_CANCELED=9003` becomes `TASK_BT_CANCELLED` with old name kept as alias. Docs state RPC uses thin codes only.

**Tech Stack:** Protocol Buffers 3, existing `automsgs` CMake proto generation, Markdown specs under `docs/superpowers/`.

**Spec:** `docs/superpowers/specs/2026-08-15-status-code-structure-design.md`

---

## File map

| File | Role |
|------|------|
| `automsgs/proto/msgs/status_msgs/status_msgs.proto` | Sole enum definition; A0 restructure + renames |
| `automsgs/proto/rpcs/README.md` | Thin-code usage + layer note |
| `docs/superpowers/specs/2026-08-15-status-code-unify-design.md` | Cross-link structure spec; cancel spelling |
| `docs/superpowers/specs/2026-08-15-status-code-structure-design.md` | Already written; touch only if drift found |

Out of scope: `autonomy/**` call-site migration (A1), deleting aliases (A2), fine→thin gateway (A3).

---

### Task 1: Header comment + thin-section markers

**Files:**
- Modify: `automsgs/proto/msgs/status_msgs/status_msgs.proto` (file header + enum opening through thin codes)

- [ ] **Step 1: Replace the file-level StatusCode comment block**

Replace lines 19–30 with:

```protobuf
// Unified status / result codes for msgs (StatusPb) and RPCs (rpcs.common.Status).
//
// Layout (declaration order):
//   1) Thin codes  — for rpcs.common.Status (and OK/UNKNOWN)
//   2) Fine codes  — for internal StatusPb diagnostics
//   3) DEPRECATED  — Nav2/MBF aliases; do not add new ones (phase A2 removes)
//
// Naming: no global CODE_/RPC_ prefix; <DOMAIN>_*; cancel spelling CANCELLED.
// Ranges & phases: docs/superpowers/specs/2026-08-15-status-code-structure-design.md
```

- [ ] **Step 2: Mark the start of the thin section**

Immediately after `option allow_alias = true;` and before `OK = 0;`, ensure a section banner:

```protobuf
  // ===========================================================================
  // (1) THIN CODES — use in rpcs.common.Status
  // ===========================================================================

  // Success / unset (RPC Status.code: success = 0).
  OK = 0;
  UNKNOWN = 1;
```

Keep existing thin blocks 2–99 … 900–999 as they are (values unchanged). Update the localization thin comment to:

```protobuf
  // ---------------------------------------------------------------------------
  // Thin: localization RPC (800-899) — distinct from fine LOCALIZATION_* (3000+)
  // LOCALIZATION_NOT_READY remains 3002 (fine); thin uses UNAVAILABLE at 801.
  // ---------------------------------------------------------------------------
```

- [ ] **Step 3: Commit**

```bash
git add automsgs/proto/msgs/status_msgs/status_msgs.proto
git commit -m "$(cat <<'EOF'
docs(status): mark StatusCode thin-layer header

EOF
)"
```

---

### Task 2: Move interleaved thin codes above fine CONTROL

**Files:**
- Modify: `automsgs/proto/msgs/status_msgs/status_msgs.proto`

Today `MAPPING_*` / `TELEOP_*` / `EXPLORATION_*` / `SYSTEM_*` sit after `CONTROL_*`. They must move into section (1) so all thin codes appear before any fine codes.

- [ ] **Step 1: Cut the four thin blocks out from between CONTROL and LOCALIZATION fine**

Remove these blocks from their current place (currently after `CONTROL_UNKNOWN = 1099`):

- `MAPPING_BUSY` … `MAPPING_FAILED` (1100–1102)
- `TELEOP_BUSY` … `TELEOP_NOT_READY` (1200–1205)
- `EXPLORATION_BUSY` … `EXPLORATION_NOT_RUNNING` (1300–1305)
- `SYSTEM_NOT_READY` … `SYSTEM_ESTOP_CLEAR_DENIED` (2000–2003)

- [ ] **Step 2: Paste them after `FOLLOW_CANCELLED = 903` (end of follow thin)**

Order and comments:

```protobuf
  // ---------------------------------------------------------------------------
  // Thin: mapping process (1100-1199) — distinct from MAP_* storage 600 / fine 7000
  // ---------------------------------------------------------------------------
  MAPPING_BUSY = 1100;
  MAPPING_NOT_RUNNING = 1101;
  MAPPING_FAILED = 1102;

  // ---------------------------------------------------------------------------
  // Thin: teleop (1200-1299)
  // ---------------------------------------------------------------------------
  TELEOP_BUSY = 1200;
  TELEOP_REJECTED = 1201;
  TELEOP_TIMEOUT = 1202;
  TELEOP_CANCELLED = 1203;
  TELEOP_COLLISION = 1204;
  TELEOP_NOT_READY = 1205;

  // ---------------------------------------------------------------------------
  // Thin: exploration (1300-1399)
  // ---------------------------------------------------------------------------
  EXPLORATION_BUSY = 1300;
  EXPLORATION_CANCELLED = 1301;
  EXPLORATION_FAILED = 1302;
  EXPLORATION_NO_AREA = 1303;
  EXPLORATION_SAVE_FAILED = 1304;
  EXPLORATION_NOT_RUNNING = 1305;

  // ---------------------------------------------------------------------------
  // Thin: system (2000-2099)
  // ---------------------------------------------------------------------------
  SYSTEM_NOT_READY = 2000;
  SYSTEM_ESTOP = 2001;
  SYSTEM_HARDWARE_FAULT = 2002;
  SYSTEM_ESTOP_CLEAR_DENIED = 2003;
```

- [ ] **Step 3: Verify no duplicate enum names and values unchanged**

Run:

```bash
python3 - <<'PY'
from pathlib import Path
import re
text = Path("automsgs/proto/msgs/status_msgs/status_msgs.proto").read_text()
# names -> list of values
pairs = re.findall(r"^\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*;", text, re.M)
from collections import defaultdict
by_name, by_val_name = {}, defaultdict(set)
for n, v in pairs:
    v = int(v)
    if n in by_name and by_name[n] != v:
        raise SystemExit(f"CONFLICT name {n}: {by_name[n]} vs {v}")
    by_name[n] = v
    by_val_name[v].add(n)
# spot-check thin moves
expect = {
    "FOLLOW_CANCELLED": 903,
    "MAPPING_BUSY": 1100,
    "TELEOP_CANCELLED": 1203,
    "EXPLORATION_BUSY": 1300,
    "SYSTEM_NOT_READY": 2000,
    "CONTROL_ERROR": 1000,
    "LOCALIZATION_ERROR": 3000,
}
for k, v in expect.items():
    assert by_name[k] == v, (k, by_name.get(k), v)
# CONTROL must appear after SYSTEM in file order
c = text.index("CONTROL_ERROR = 1000")
s = text.index("SYSTEM_NOT_READY = 2000")
assert s < c, "SYSTEM thin must be declared before CONTROL fine"
print(f"OK: {len(by_name)} unique names")
PY
```

Expected: `OK: … unique names` (no CONFLICT).

- [ ] **Step 4: Commit**

```bash
git add automsgs/proto/msgs/status_msgs/status_msgs.proto
git commit -m "$(cat <<'EOF'
refactor(status): group thin StatusCodes before fine codes

EOF
)"
```

---

### Task 3: Fine section banner + CANCELLED renames

**Files:**
- Modify: `automsgs/proto/msgs/status_msgs/status_msgs.proto`

- [ ] **Step 1: Insert fine-section banner before CONTROL**

```protobuf
  // ===========================================================================
  // (2) FINE CODES — internal StatusPb diagnostics (not for third-party RPC)
  // ===========================================================================

  // ---------------------------------------------------------------------------
  // Fine: control (1000-1099)
  // ---------------------------------------------------------------------------
  CONTROL_ERROR = 1000;
  ...
  CONTROL_CANCELED = 1010;   # will become CANCELLED in next step
```

Update remaining fine section comments from “module error codes start from” to `Fine: <domain> (…)`.

- [ ] **Step 2: Apply canonical CANCELLED renames in the fine section**

In the fine section only:

| Old canonical line | New canonical line |
|--------------------|--------------------|
| `CONTROL_CANCELED = 1010;` | `CONTROL_CANCELLED = 1010;` |
| `RECOVERY_CANCELED = 4001;` | `RECOVERY_CANCELLED = 4001;` |
| `PLANNING_CANCELED = 6002;` | `PLANNING_CANCELLED = 6002;` |
| `TASK_CANCELED = 9003;` | `TASK_BT_CANCELLED = 9003;` |

Do **not** rename thin `TASK_CANCELLED = 302` or other thin `*_CANCELLED`.

- [ ] **Step 3: Add deprecated spelling aliases (same values)**

Place these inside section (3) once that banner exists (Task 4), or temporarily at the end of the fine TASK block until Task 4 — prefer adding in Task 4 with the other aliases. For this step, only change the four canonical names above.

- [ ] **Step 4: Verify rename map**

```bash
python3 - <<'PY'
from pathlib import Path
import re
text = Path("automsgs/proto/msgs/status_msgs/status_msgs.proto").read_text()
pairs = dict(re.findall(r"^\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*;", text, re.M))
assert pairs["CONTROL_CANCELLED"] == "1010"
assert pairs["RECOVERY_CANCELLED"] == "4001"
assert pairs["PLANNING_CANCELLED"] == "6002"
assert pairs["TASK_BT_CANCELLED"] == "9003"
assert pairs["TASK_CANCELLED"] == "302"
# old names may be absent until Task 4 adds aliases
for gone in ("CONTROL_CANCELED", "RECOVERY_CANCELED", "PLANNING_CANCELED", "TASK_CANCELED"):
    if gone in pairs:
        print(f"note: {gone} still present as alias OK")
print("OK renames")
PY
```

Expected: `OK renames`

- [ ] **Step 5: Commit**

```bash
git add automsgs/proto/msgs/status_msgs/status_msgs.proto
git commit -m "$(cat <<'EOF'
refactor(status): unify fine cancel spelling to CANCELLED

EOF
)"
```

---

### Task 4: DEPRECATED section + spelling aliases

**Files:**
- Modify: `automsgs/proto/msgs/status_msgs/status_msgs.proto`

- [ ] **Step 1: Banner before legacy aliases**

Replace the current “Legacy nav2 / MBF aliases” header (success aliases start) with:

```protobuf
  // ===========================================================================
  // (3) DEPRECATED ALIASES — Nav2/MBF + old spellings; do not add; remove in A2
  // ===========================================================================

  // Old cancel spellings (canonical names are *_CANCELLED / TASK_BT_CANCELLED).
  CONTROL_CANCELED = 1010;
  RECOVERY_CANCELED = 4001;
  PLANNING_CANCELED = 6002;
  TASK_CANCELED = 9003;

  // Legacy nav2 / MBF aliases (success = OK).
  ERROR_CODE_NONE = 0;
  ...
```

Keep all existing `MOVE_BASE_*`, `GET_PATH_*`, `EXE_PATH_*`, `FOLLOW_PATH_*`, `NAV_TO_POSE_*`, etc. unchanged beneath this banner.

- [ ] **Step 2: Verify aliases resolve to same values as canonicals**

```bash
python3 - <<'PY'
from pathlib import Path
import re
text = Path("automsgs/proto/msgs/status_msgs/status_msgs.proto").read_text()
pairs = dict(re.findall(r"^\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*;", text, re.M))
checks = [
    ("CONTROL_CANCELLED", "CONTROL_CANCELED", "1010"),
    ("RECOVERY_CANCELLED", "RECOVERY_CANCELED", "4001"),
    ("PLANNING_CANCELLED", "PLANNING_CANCELED", "6002"),
    ("TASK_BT_CANCELLED", "TASK_CANCELED", "9003"),
    ("TASK_CANCELLED", None, "302"),
]
for canon, alias, val in checks:
    assert pairs[canon] == val, (canon, pairs.get(canon))
    if alias:
        assert pairs[alias] == val, (alias, pairs.get(alias))
assert "DEPRECATED ALIASES" in text
assert text.index("TASK_BT_CANCELLED") < text.index("TASK_CANCELED")
print("OK aliases")
PY
```

Expected: `OK aliases`

- [ ] **Step 3: Commit**

```bash
git add automsgs/proto/msgs/status_msgs/status_msgs.proto
git commit -m "$(cat <<'EOF'
refactor(status): quarantine StatusCode aliases as DEPRECATED

EOF
)"
```

---

### Task 5: Sync RPC README and unify design spec

**Files:**
- Modify: `automsgs/proto/rpcs/README.md` (section `## 状态码`)
- Modify: `docs/superpowers/specs/2026-08-15-status-code-unify-design.md`

- [ ] **Step 1: Update README 状态码 section**

After the existing naming paragraph, add:

```markdown
**分层：** `Status.code` 只使用**薄码**（0–999 中的 RPC 域、1100–1399、2000–2099 及通用码）。  
`CONTROL_*` / `PLANNING_*` / `TASK_*`（9000+）等为**细码**，供内部 `StatusPb`；第三方勿依赖。  
取消拼写统一为 `CANCELLED`；BT 任务取消细码为 `TASK_BT_CANCELLED`（9003）。  
结构与去别名阶段见 `docs/superpowers/specs/2026-08-15-status-code-structure-design.md`。
```

In the range table, prefix layer hints where helpful:

| 区间 | 层 | 模块 |
|------|----|------|
| 0–1 | 薄 | `OK` / `UNKNOWN` |
| … | 薄 | … |
| 1000–1099 | 细 | `CONTROL_*` |
| 1100–1199 | 薄 | 建图过程 |
| … | | |
| 3000+ | 细 | localization / recovery / planning / … |

- [ ] **Step 2: Patch unify design naming table**

In `2026-08-15-status-code-unify-design.md`, append under 命名约定:

```markdown
结构整理（薄/细/DEPRECATED、分阶段去别名）见
`2026-08-15-status-code-structure-design.md`。取消拼写：`CANCELLED`；
`TASK_CANCELED`（9003）规范名为 `TASK_BT_CANCELLED`。
```

- [ ] **Step 3: Commit**

```bash
git add automsgs/proto/rpcs/README.md docs/superpowers/specs/2026-08-15-status-code-unify-design.md
git commit -m "$(cat <<'EOF'
docs(status): document StatusCode thin/fine layers

EOF
)"
```

---

### Task 6: Final A0 verification

**Files:** none (read-only checks)

- [ ] **Step 1: Full invariant script**

```bash
python3 - <<'PY'
from pathlib import Path
import re
text = Path("automsgs/proto/msgs/status_msgs/status_msgs.proto").read_text()
assert "(1) THIN CODES" in text and "(2) FINE CODES" in text and "(3) DEPRECATED" in text
# order: thin SYSTEM before fine CONTROL before deprecated MOVE_BASE
assert text.index("SYSTEM_NOT_READY = 2000") < text.index("CONTROL_ERROR = 1000")
assert text.index("CONTROL_ERROR = 1000") < text.index("MOVE_BASE_SUCCESS = 0")
pairs = re.findall(r"^\s+([A-Z][A-Z0-9_]*)\s*=\s*(\d+)\s*;", text, re.M)
by_name = {}
for n, v in pairs:
    v = int(v)
    if n in by_name and by_name[n] != v:
        raise SystemExit(f"bad alias {n}")
    by_name[n] = v
# numeric freeze samples
frozen = {
    "OK": 0, "UNKNOWN": 1, "NAVIGATION_BUSY": 105, "TASK_CANCELLED": 302,
    "MAP_NOT_FOUND": 600, "LOCALIZATION_UNAVAILABLE": 801,
    "CONTROL_CANCELLED": 1010, "MAPPING_BUSY": 1100, "TELEOP_BUSY": 1200,
    "EXPLORATION_BUSY": 1300, "SYSTEM_ESTOP": 2001,
    "LOCALIZATION_NOT_READY": 3002, "TASK_BT_CANCELLED": 9003,
    "FOLLOW_PATH_INVALID_PATH": 1012, "COMPUTE_PATH_TO_POSE_TIMEOUT": 6008,
}
for k, v in frozen.items():
    assert by_name[k] == v, (k, by_name.get(k), v)
print(f"A0 OK ({len(by_name)} names)")
PY
```

Expected: `A0 OK (… names)`

- [ ] **Step 2: Optional protoc (if available on PATH)**

```bash
command -v protoc >/dev/null && protoc -I automsgs/proto \
  --descriptor_set_out=/tmp/status_msgs.pb \
  automsgs/proto/msgs/status_msgs/status_msgs.proto \
  && echo protoc_ok || echo protoc_skip
```

Expected: `protoc_ok` or `protoc_skip` (skip is acceptable if toolchain missing).

- [ ] **Step 3: Commit only if Step 1–2 caused doc/proto fixes; otherwise done**

If the script failed, fix proto/docs and commit the fix; if it passed with no edits, no empty commit.

---

## Spec coverage checklist

| Spec requirement | Task |
|------------------|------|
| Three-section layout | 1–4 |
| Thin before fine declaration order | 2 |
| CANCELLED spelling + TASK_BT_CANCELLED | 3–4 |
| Old spellings kept as aliases | 4 |
| No numeric changes | 6 frozen map |
| No call-site migration | (out of scope) |
| README / unify doc sync | 5 |

## Handoff

Plan complete and saved to `docs/superpowers/plans/2026-08-15-status-code-a0-structure.md`.

**Two execution options:**

1. **Subagent-Driven (recommended)** — fresh subagent per task, review between tasks  
2. **Inline Execution** — execute in this session with executing-plans checkpoints  

Which approach?
