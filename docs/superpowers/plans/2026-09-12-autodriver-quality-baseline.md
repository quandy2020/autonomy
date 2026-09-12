# AutoDriver Quality Baseline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Establish an auditable source/build inventory and a reproducible, hardware-independent AutoDriver configure-build-test-sanitizer path on Ubuntu.

**Architecture:** Keep the first increment deliberately narrow: repository-quality checks are small Python and shell programs under `scripts/`, while AutoDriver remains a standalone CMake project that resolves installed Autolink and Automsgs packages when embedded targets are unavailable. GitHub Actions calls the same presets and scripts used locally; no robot hardware or vendor SDK is part of this profile.

**Tech Stack:** C++17, CMake 3.20+, CMake Presets v6, Ninja, CTest, Python 3 `unittest`, Bash, GitHub Actions, AddressSanitizer, UndefinedBehaviorSanitizer

**Spec:** `docs/superpowers/specs/2026-09-11-autonomy-engineering-maturity-design.md`

## Global Constraints

- This plan implements Phase 0 and only the first `autodriver-minimal` vertical slice of Phase 1. `autonomy-minimal`, target decomposition, workflow regression, hardware qualification, and release governance require separate plans.
- Do not redesign algorithms, replace Autolink, migrate to ROS 2, or make functional-safety claims.
- CI must not require physical hardware, private SDKs, credentials, or developer-specific paths.
- Use out-of-source builds exclusively; generated headers belong under the binary directory.
- Preserve the evidence labels `observed`, `executed`, and `unverified`; source presence is not runtime proof.
- Before implementation, create an isolated worktree from commit `d96b9508` using `superpowers:using-git-worktrees`. Do not implement this plan in the current dirty checkout.
- Before every task, record `git rev-parse HEAD` and `git status --short`. Stop if a task file is already modified in the isolated worktree.
- Stage only the files listed by the current task. Never stage, format, restore, or rewrite unrelated files.
- Supported first CI platform: Ubuntu 22.04 x86-64 with GCC and Clang. macOS and ARM remain explicitly unverified in this increment.

---

## File Map

- `docs/quality/engineering-baseline.md`: human-readable targets, dependencies, tests, profiles, generated outputs, and evidence state.
- `docs/quality/source-provenance.yaml`: machine-readable ownership and upstream-origin inventory.
- `scripts/quality/provenance.py`: dependency-free provenance parser and validator.
- `scripts/quality/test_provenance.py`: validator contract tests.
- `scripts/quality/portable_paths.py`: detects host-specific absolute paths in build inputs.
- `scripts/quality/test_portable_paths.py`: portability-check fixtures.
- `scripts/quality/generated_outputs.py`: verifies configured generated files remain inside a build root.
- `scripts/quality/test_generated_outputs.py`: generated-output boundary tests.
- `autodriver/autodriver/CMakeLists.txt`: resolves embedded or installed Autolink/Automsgs targets.
- `autodriver/CMakePresets.json`: shared minimal and sanitizer profiles.
- `scripts/ci/bootstrap_autodriver_dependencies.sh`: installs Autolink and Automsgs into an isolated prefix.
- `scripts/ci/check_changed_cpp_format.sh`: checks only changed first-party C/C++ files.
- `.github/workflows/cpp-quality.yml`: portability, GCC, Clang, and sanitizer jobs.
- `autodriver/docs/source/guide/testing.md`: exact developer commands and evidence limitations.
- `CMakeLists.txt`: removes the personal OR-Tools fallback and emits `config.hpp` below the binary directory.

---

### Task 1: Source provenance contract and baseline inventory

**Files:**
- Create: `scripts/quality/provenance.py`
- Create: `scripts/quality/test_provenance.py`
- Create: `docs/quality/source-provenance.yaml`
- Create: `docs/quality/engineering-baseline.md`

**Interfaces:**
- Produces: `validate_manifest(path: pathlib.Path, repository_root: pathlib.Path) -> list[str]`; an empty list means valid.
- Produces: command `python3 scripts/quality/provenance.py docs/quality/source-provenance.yaml` with exit code 0 on valid input and 1 on validation errors.
- Manifest keys: `schema_version: 1`; `components[]` entries with `path`, `classification`, `upstream`, `license`, and `verification`.
- Allowed classifications: `first_party`, `adapted`, `generated`, `submodule`, `vendor`.
- Allowed verification values: `observed`, `executed`, `unverified`.

- [ ] **Step 1: Write contract tests**

Use `unittest`, `tempfile.TemporaryDirectory`, and JSON-compatible YAML so the validator needs no PyYAML dependency. Add tests that prove: the repository manifest is valid; an unknown classification is rejected; a missing path is rejected; a path that escapes the repository is rejected.

```python
class ProvenanceTest(unittest.TestCase):
    def test_rejects_unknown_classification(self):
        errors = validate_manifest(
            self.write_manifest({
                "schema_version": 1,
                "components": [{
                    "path": "autodriver",
                    "classification": "mystery",
                    "upstream": None,
                    "license": "Apache-2.0",
                    "verification": "observed",
                }],
            }),
            self.root,
        )
        self.assertIn("components[0].classification: unsupported value 'mystery'", errors)
```

- [ ] **Step 2: Run the tests and confirm the intended failure**

Run: `python3 -m unittest scripts.quality.test_provenance -v`

Expected: import failure because `scripts/quality/provenance.py` does not exist.

- [ ] **Step 3: Implement the dependency-free validator**

Parse the manifest with `json.loads`; JSON is valid YAML 1.2. Validate the exact keys, enum values, repository-relative paths, path existence, and duplicate paths. Print one `path: message` line per error.

```python
ALLOWED_CLASSIFICATIONS = {"first_party", "adapted", "generated", "submodule", "vendor"}
ALLOWED_VERIFICATIONS = {"observed", "executed", "unverified"}
REQUIRED_KEYS = {"path", "classification", "upstream", "license", "verification"}

def validate_manifest(path: pathlib.Path, repository_root: pathlib.Path) -> list[str]:
    errors: list[str] = []
    data = json.loads(path.read_text(encoding="utf-8"))
    if data.get("schema_version") != 1:
        errors.append("schema_version: expected 1")
    seen: set[str] = set()
    for index, component in enumerate(data.get("components", [])):
        prefix = f"components[{index}]"
        missing = REQUIRED_KEYS - component.keys()
        for key in sorted(missing):
            errors.append(f"{prefix}.{key}: missing")
        relative = pathlib.PurePosixPath(component.get("path", ""))
        if relative.is_absolute() or ".." in relative.parts:
            errors.append(f"{prefix}.path: must be repository-relative")
        elif not (repository_root / relative).exists():
            errors.append(f"{prefix}.path: does not exist")
        if str(relative) in seen:
            errors.append(f"{prefix}.path: duplicate")
        seen.add(str(relative))
        classification = component.get("classification")
        if classification not in ALLOWED_CLASSIFICATIONS:
            errors.append(f"{prefix}.classification: unsupported value {classification!r}")
        verification = component.get("verification")
        if verification not in ALLOWED_VERIFICATIONS:
            errors.append(f"{prefix}.verification: unsupported value {verification!r}")
    return errors
```

- [ ] **Step 4: Populate the inventories**

The manifest must contain entries for at least `autodriver`, `autolink`, `automsgs`, `autonomy`, `autonomy/localization/cartographer`, `autonomy/localization/atlas`, `thirdparty`, and every Git submodule reported by `git submodule status --recursive`. Use `null` for no upstream URL; do not guess an upstream or license. Mark code as `executed` only after a command in this plan has run successfully.

The Markdown baseline must contain these tables with one row per current item:

1. supported and unverified platforms;
2. top-level CMake targets and direct link dependencies;
3. all 17 AutoDriver CTest registrations and whether hardware is required;
4. optional SDK option, package name, enabled feature, and missing-package behavior;
5. generated file, template, and output location;
6. build profiles and exact commands;
7. evidence legend for `observed`, `executed`, and `unverified`.

Generate target facts from `add_library`, `add_executable`, and `target_link_libraries` in source; do not describe a target as build-tested yet.

- [ ] **Step 5: Run focused verification**

Run:

```bash
python3 -m unittest scripts.quality.test_provenance -v
python3 scripts/quality/provenance.py docs/quality/source-provenance.yaml
git diff --check -- docs/quality scripts/quality
```

Expected: all tests pass, validator exits 0, and `git diff --check` produces no output.

- [ ] **Step 6: Commit only this task**

```bash
git add docs/quality/engineering-baseline.md docs/quality/source-provenance.yaml scripts/quality/provenance.py scripts/quality/test_provenance.py
git commit -m "docs: add engineering baseline inventory"
```

---

### Task 2: Portable dependency discovery

**Files:**
- Create: `scripts/quality/portable_paths.py`
- Create: `scripts/quality/test_portable_paths.py`
- Modify: `CMakeLists.txt` at the `_ORTOOLS_CANDIDATES` block

**Interfaces:**
- Produces: `find_host_paths(paths: Iterable[pathlib.Path], repository_root: pathlib.Path) -> list[str]`.
- Scanned suffixes: `.cmake`, `.json`, `.sh`, `.yml`, `.yaml` and files named `CMakeLists.txt`.
- Rejected literals: `/home/<name>/`, `/Users/<name>/`, and Windows drive paths such as `C:\Users\name\`.
- Exclusions: `.git`, `build`, `cmake-build-*`, documentation, tests, and the detector itself.

- [ ] **Step 1: Write portability fixture tests**

Create temporary files proving that `/home/quandy/sdk`, `/Users/quandy/sdk`, and `C:\Users\quandy\sdk` are rejected while `${CMAKE_SOURCE_DIR}/sdk`, `/usr/local`, test fixtures, and Markdown are accepted.

- [ ] **Step 2: Run the test and confirm failure**

Run: `python3 -m unittest scripts.quality.test_portable_paths -v`

Expected: import failure because `portable_paths.py` does not exist.

- [ ] **Step 3: Implement the scanner**

Use compiled regular expressions and return stable, sorted `path:line:text` diagnostics. The command-line entry point scans the repository root resolved from `__file__` and exits 1 when diagnostics exist.

- [ ] **Step 4: Verify that the existing OR-Tools path is detected**

Run: `python3 scripts/quality/portable_paths.py`

Expected: non-zero exit and a diagnostic for the `/home/quandy/.../or-tools` entry in root `CMakeLists.txt`.

- [ ] **Step 5: Remove personal OR-Tools candidates**

Replace the candidate loop with explicit cache-variable behavior:

```cmake
if(AUTONOMY_ORTOOLS_ROOT)
  list(PREPEND CMAKE_PREFIX_PATH "${AUTONOMY_ORTOOLS_ROOT}")
endif()
```

Keep `AUTONOMY_WITH_ORTOOLS=OFF` as a valid configuration. If OR-Tools is enabled and cannot be found through normal CMake discovery plus `AUTONOMY_ORTOOLS_ROOT`, fail with an actionable message naming both options.

- [ ] **Step 6: Verify the fix**

Run:

```bash
python3 -m unittest scripts.quality.test_portable_paths -v
python3 scripts/quality/portable_paths.py
cmake -S . -B build/plan-portability -DAUTONOMY_WITH_ORTOOLS=OFF -DBUILD_AUTODRIVER=OFF
git diff --check -- CMakeLists.txt scripts/quality
```

Expected: tests and scan pass; CMake progresses past OR-Tools discovery. A later unrelated missing dependency may be recorded, but an OR-Tools or personal-path error fails this step.

- [ ] **Step 7: Commit only the reviewed hunks**

```bash
git add CMakeLists.txt scripts/quality/portable_paths.py scripts/quality/test_portable_paths.py
git diff --cached --check
git commit -m "build: remove host-specific dependency paths"
```

---

### Task 3: Binary-directory generated header boundary

**Files:**
- Create: `scripts/quality/generated_outputs.py`
- Create: `scripts/quality/test_generated_outputs.py`
- Modify: `CMakeLists.txt` at the `configure_file` call for `autonomy/common/config.hpp`
- Modify: `docs/quality/engineering-baseline.md` generated-output table

**Interfaces:**
- Produces: `is_within(path: pathlib.Path, root: pathlib.Path) -> bool`.
- Produces: command `python3 scripts/quality/generated_outputs.py build/plan-generated` that configures the minimal available root profile, verifies generated outputs are below the supplied build root, and checks tracked-source status before and after.

- [ ] **Step 1: Write boundary tests**

Test that `build/generated/autonomy/common/config.hpp` is accepted and `autonomy/common/config.hpp` plus `build/../autonomy/common/config.hpp` are rejected for build root `build`.

- [ ] **Step 2: Run the test and confirm failure**

Run: `python3 -m unittest scripts.quality.test_generated_outputs -v`

Expected: import failure because `generated_outputs.py` does not exist.

- [ ] **Step 3: Implement path-boundary validation**

Resolve paths with `Path.resolve()` and use `Path.is_relative_to()` on Python 3.9+ equivalent logic via `relative_to` in a `try/except ValueError`. The CLI must snapshot `git status --porcelain=v1 --untracked-files=no` before and after configuration and report only newly changed tracked paths.

- [ ] **Step 4: Move the generated header**

Change the output to:

```cmake
set(AUTONOMY_GENERATED_DIR "${PROJECT_BINARY_DIR}/generated")
file(MAKE_DIRECTORY "${AUTONOMY_GENERATED_DIR}/autonomy/common")
configure_file(
  "${PROJECT_SOURCE_DIR}/autonomy/common/config.hpp.cmake"
  "${AUTONOMY_GENERATED_DIR}/autonomy/common/config.hpp")
```

Add `$<BUILD_INTERFACE:${AUTONOMY_GENERATED_DIR}>` before the source include directory on the aggregate target that consumes `autonomy/common/config.hpp`. Do not delete a tracked source header in this task; first prove whether it is generated and tracked with `git ls-files autonomy/common/config.hpp` and record the finding.

- [ ] **Step 5: Verify header resolution and source cleanliness**

Run:

```bash
python3 -m unittest scripts.quality.test_generated_outputs -v
python3 scripts/quality/generated_outputs.py build/plan-generated
git diff --check -- CMakeLists.txt docs/quality scripts/quality
```

Expected: unit tests pass, generated header is reported under `build/plan-generated`, and no newly modified tracked source path appears.

- [ ] **Step 6: Commit the boundary change**

```bash
git add CMakeLists.txt docs/quality/engineering-baseline.md scripts/quality/generated_outputs.py scripts/quality/test_generated_outputs.py
git diff --cached --check
git commit -m "build: keep generated headers out of source tree"
```

---

### Task 4: Standalone AutoDriver package resolution

**Files:**
- Create: `autodriver/test/cmake/standalone_dependencies/CMakeLists.txt`
- Modify: `autodriver/autodriver/CMakeLists.txt` around dependency lookup and `target_link_libraries`

**Interfaces:**
- Produces CMake variables `AUTODRIVER_AUTOLINK_TARGET` and `AUTODRIVER_AUTOMSGS_TARGET`.
- Embedded mode uses existing targets `autolink` and `automsgs`.
- Standalone mode resolves `Autolink::Autolink` from `find_package(Autolink CONFIG REQUIRED)` and `automsgs::automsgs` from `find_package(automsgs CONFIG REQUIRED)`.

- [ ] **Step 1: Add a configure-contract project**

The fixture creates imported interface targets with the installed names, includes a small extracted dependency-selection module, and asserts the selected variables exactly match the imported names. To keep production code focused, create `autodriver/cmake/autodriver_dependencies.cmake` and test that module directly.

```cmake
add_library(Autolink::Autolink INTERFACE IMPORTED)
add_library(automsgs::automsgs INTERFACE IMPORTED)
include("${AUTODRIVER_SOURCE_DIR}/cmake/autodriver_dependencies.cmake")
if(NOT AUTODRIVER_AUTOLINK_TARGET STREQUAL "Autolink::Autolink")
  message(FATAL_ERROR "unexpected Autolink target")
endif()
```

- [ ] **Step 2: Run the fixture and confirm failure**

Run: `cmake -S autodriver/test/cmake/standalone_dependencies -B build/test-autodriver-dependencies -DAUTODRIVER_SOURCE_DIR=$PWD/autodriver`

Expected: failure because `autodriver_dependencies.cmake` does not exist.

- [ ] **Step 3: Implement target selection**

Create `autodriver/cmake/autodriver_dependencies.cmake`:

```cmake
if(TARGET autolink)
  set(AUTODRIVER_AUTOLINK_TARGET autolink)
else()
  if(NOT TARGET Autolink::Autolink)
    find_package(Autolink CONFIG REQUIRED)
  endif()
  set(AUTODRIVER_AUTOLINK_TARGET Autolink::Autolink)
endif()

if(TARGET automsgs)
  set(AUTODRIVER_AUTOMSGS_TARGET automsgs)
else()
  if(NOT TARGET automsgs::automsgs)
    find_package(automsgs CONFIG REQUIRED)
  endif()
  set(AUTODRIVER_AUTOMSGS_TARGET automsgs::automsgs)
endif()
```

Include it after `find_package(Eigen3 REQUIRED)` and link the two selected variables instead of bare target names. Do not create aliases for imported targets.

- [ ] **Step 4: Add the embedded-target fixture case**

Create ordinary interface targets named `autolink` and `automsgs`, include the module, and assert those names are selected without package lookup.

- [ ] **Step 5: Run both cases**

Run:

```bash
cmake -S autodriver/test/cmake/standalone_dependencies -B build/test-autodriver-dependencies -DAUTODRIVER_SOURCE_DIR=$PWD/autodriver
cmake --build build/test-autodriver-dependencies
git diff --check -- autodriver
```

Expected: configure and empty build succeed.

- [ ] **Step 6: Commit the dependency boundary**

```bash
git add autodriver/autodriver/CMakeLists.txt autodriver/cmake/autodriver_dependencies.cmake autodriver/test/cmake/standalone_dependencies/CMakeLists.txt
git commit -m "build: support standalone autodriver dependencies"
```

---

### Task 5: Reproducible dependency bootstrap and CMake presets

**Files:**
- Create: `scripts/ci/bootstrap_autodriver_dependencies.sh`
- Create: `scripts/ci/test_bootstrap_autodriver_dependencies.sh`
- Create: `autodriver/CMakePresets.json`

**Interfaces:**
- Bootstrap invocation: `scripts/ci/bootstrap_autodriver_dependencies.sh BUILD_ROOT INSTALL_PREFIX`.
- Presets: `autodriver-minimal`, `autodriver-minimal-gcc`, `autodriver-minimal-clang`, and `autodriver-sanitizers` for configure; matching build/test presets use the same names.
- `CMAKE_PREFIX_PATH` supplies installed Autolink and Automsgs.

- [ ] **Step 1: Write a shell contract test**

The test runs the bootstrap script with a temporary fake `cmake` executable that logs arguments, then asserts calls configure and install `autolink` before `automsgs`, both with the requested prefix, and disable Autolink tests/examples/docs/FastDDS plus Automsgs tests/examples where supported.

- [ ] **Step 2: Confirm the test fails**

Run: `bash scripts/ci/test_bootstrap_autodriver_dependencies.sh`

Expected: failure because the bootstrap script does not exist.

- [ ] **Step 3: Implement the bootstrap script**

Use `set -euo pipefail`, resolve the repository root relative to the script, reject build/install paths inside the source subdirectories, create both output directories, and invoke only `cmake -S`, `cmake --build`, and `cmake --install`. Pass `-DCMAKE_BUILD_TYPE=Debug`, `-DCMAKE_INSTALL_PREFIX`, and `-DCMAKE_PREFIX_PATH` explicitly.

- [ ] **Step 4: Add presets**

Use preset schema version 6. The hidden base preset sets Ninja, `${sourceDir}/../build/autodriver/${presetName}`, C++17, compile commands, tests on, examples/docs/vendor SDKs off, and `CMAKE_PREFIX_PATH` from `$env{AUTONOMY_CI_PREFIX}`. Compiler presets set `CMAKE_C_COMPILER` and `CMAKE_CXX_COMPILER`. Sanitizers add:

```json
"CMAKE_CXX_FLAGS": "-fsanitize=address,undefined -fno-omit-frame-pointer",
"CMAKE_EXE_LINKER_FLAGS": "-fsanitize=address,undefined",
"CMAKE_SHARED_LINKER_FLAGS": "-fsanitize=address,undefined"
```

Test presets must set `outputOnFailure: true` and `execution.noTestsAction: "error"`.

- [ ] **Step 5: Validate contracts without downloading dependencies**

Run:

```bash
bash -n scripts/ci/bootstrap_autodriver_dependencies.sh
bash scripts/ci/test_bootstrap_autodriver_dependencies.sh
cmake --list-presets -S autodriver
python3 -m json.tool autodriver/CMakePresets.json >/dev/null
git diff --check -- autodriver/CMakePresets.json scripts/ci
```

Expected: shell tests pass; all four configure presets are listed; JSON validation succeeds.

- [ ] **Step 6: Commit presets and bootstrap**

```bash
git add autodriver/CMakePresets.json scripts/ci/bootstrap_autodriver_dependencies.sh scripts/ci/test_bootstrap_autodriver_dependencies.sh
git commit -m "build: add autodriver minimal presets"
```

---

### Task 6: Changed-file C++ formatting gate

**Files:**
- Create: `scripts/ci/check_changed_cpp_format.sh`
- Create: `scripts/ci/test_check_changed_cpp_format.sh`

**Interfaces:**
- Invocation: `scripts/ci/check_changed_cpp_format.sh BASE_REF [HEAD_REF]`.
- Checks added or modified first-party `*.c`, `*.cc`, `*.cpp`, `*.h`, `*.hh`, `*.hpp` files using `clang-format --dry-run --Werror`.
- Excludes `thirdparty/`, copied upstream trees identified as `adapted` or `vendor`, and generated/build directories.

- [ ] **Step 1: Write the fake-Git/fake-clang-format contract test**

Create a temporary Git repository with one changed first-party `.cpp`, one changed `thirdparty/*.cpp`, and one deleted header. Use a fake `clang-format` that logs paths. Assert only the changed first-party `.cpp` is passed.

- [ ] **Step 2: Confirm failure before implementation**

Run: `bash scripts/ci/test_check_changed_cpp_format.sh`

Expected: failure because the checker does not exist.

- [ ] **Step 3: Implement the gate**

Use `git diff --diff-filter=ACMR --name-only -z "${BASE_REF}...${HEAD_REF}"`, NUL-safe reading, suffix filtering, path exclusions, and one `clang-format --dry-run --Werror` call per file. Exit 0 with a clear message if no eligible file changed.

- [ ] **Step 4: Verify locally**

Run:

```bash
bash -n scripts/ci/check_changed_cpp_format.sh
bash scripts/ci/test_check_changed_cpp_format.sh
scripts/ci/check_changed_cpp_format.sh HEAD HEAD
git diff --check -- scripts/ci
```

Expected: contract passes and the HEAD-to-HEAD check reports no eligible files.

- [ ] **Step 5: Commit the format gate**

```bash
git add scripts/ci/check_changed_cpp_format.sh scripts/ci/test_check_changed_cpp_format.sh
git commit -m "ci: check formatting on changed cpp files"
```

---

### Task 7: AutoDriver build, CTest, and sanitizer CI

**Files:**
- Create: `.github/workflows/cpp-quality.yml`
- Modify: `docs/quality/engineering-baseline.md` profile/evidence rows
- Modify: `docs/quality/source-provenance.yaml` verification fields only after successful execution

**Interfaces:**
- Jobs: `quality`, `autodriver-build`, and `autodriver-sanitizers`.
- Compiler matrix: GCC (`gcc`, `g++`) and Clang (`clang`, `clang++`) on `ubuntu-22.04`.
- Sanitizer environment: `ASAN_OPTIONS=detect_leaks=1:halt_on_error=1` and `UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1`.

- [ ] **Step 1: Create the workflow with narrow triggers**

Trigger pull requests and pushes to the default branch only when these paths change: `autodriver/**`, `autolink/**`, `automsgs/**`, `CMakeLists.txt`, `cmake/**`, `scripts/ci/**`, `scripts/quality/**`, `docs/quality/**`, or the workflow itself. Add `workflow_dispatch`.

- [ ] **Step 2: Add deterministic dependency installation**

Install this Ubuntu package set:

```text
cmake ninja-build gcc g++ clang pkg-config
libprotobuf-dev protobuf-compiler libgoogle-glog-dev
libgtest-dev libgmock-dev libtinyxml2-dev libyaml-cpp-dev
libeigen3-dev uuid-dev libudev-dev
```

Checkout recursively, set `AUTONOMY_CI_PREFIX=${{ github.workspace }}/build/ci-prefix`, and run the bootstrap script before any AutoDriver preset.

- [ ] **Step 3: Implement quality and build jobs**

The quality job runs all `scripts/quality/test_*.py`, all `scripts/ci/test_*.sh`, provenance validation, portable-path validation, `git diff --check`, and the changed-file formatting gate against the PR base SHA or previous push SHA.

Each matrix build runs:

```bash
cmake --preset autodriver-minimal-${COMPILER} -S autodriver
cmake --build --preset autodriver-minimal-${COMPILER}
ctest --preset autodriver-minimal-${COMPILER}
git diff --exit-code
```

The sanitizer job runs the corresponding sanitizer configure, build, and CTest presets with the sanitizer environment above.

- [ ] **Step 4: Validate workflow structure locally**

Run:

```bash
python3 -c 'import pathlib; text=pathlib.Path(".github/workflows/cpp-quality.yml").read_text(); assert "ubuntu-22.04" in text; assert "ctest --preset" in text; assert "git diff --exit-code" in text'
python3 -m unittest discover -s scripts/quality -p 'test_*.py' -v
for test_script in scripts/ci/test_*.sh; do bash "$test_script"; done
git diff --check -- .github docs/quality scripts
```

Expected: assertions and local contract tests pass; whitespace check is clean.

- [ ] **Step 5: Run one full local minimal profile**

On Ubuntu 22.04, or in the repository's documented Ubuntu container with the listed packages, run:

```bash
scripts/ci/bootstrap_autodriver_dependencies.sh build/ci-deps build/ci-prefix
AUTONOMY_CI_PREFIX=$PWD/build/ci-prefix cmake --preset autodriver-minimal-gcc -S autodriver
cmake --build --preset autodriver-minimal-gcc
ctest --preset autodriver-minimal-gcc
git diff --exit-code
```

Expected: configure/build succeed; all registered minimal-profile tests pass; no tracked source file changes. If the current host is not Ubuntu, keep the baseline status `unverified` until GitHub Actions supplies the execution evidence.

- [ ] **Step 6: Update evidence without overstating it**

Record compiler, OS, commit, command, test count, and result in `engineering-baseline.md`. Change a manifest verification field to `executed` only for the exact successfully exercised component/profile.

- [ ] **Step 7: Commit CI**

```bash
git add .github/workflows/cpp-quality.yml docs/quality/engineering-baseline.md docs/quality/source-provenance.yaml
git commit -m "ci: build and test autodriver minimal profile"
```

---

### Task 8: Developer documentation and final acceptance

**Files:**
- Modify: `autodriver/docs/source/guide/testing.md`
- Modify: `docs/quality/engineering-baseline.md`

**Interfaces:**
- Documents the exact bootstrap, configure, build, CTest, sanitizer, and cleanup commands already defined in Tasks 5-7.
- Explicitly distinguishes local execution evidence, CI evidence, source observation, and unverified platforms/hardware.

- [ ] **Step 1: Add the supported minimal workflow**

Document prerequisites, the two positional bootstrap arguments, `AUTONOMY_CI_PREFIX`, each preset, expected build directories, how to select a single CTest with `ctest --test-dir ... -R NAME`, and how to read sanitizer failures. State that vendor SDKs, physical devices, macOS, ARM, and the full Autonomy stack are outside this profile.

- [ ] **Step 2: Run the complete static acceptance suite**

Run:

```bash
python3 -m unittest discover -s scripts/quality -p 'test_*.py' -v
for test_script in scripts/ci/test_*.sh; do bash "$test_script"; done
python3 scripts/quality/provenance.py docs/quality/source-provenance.yaml
python3 scripts/quality/portable_paths.py
cmake --list-presets -S autodriver
git diff --check
```

Expected: every command exits 0.

- [ ] **Step 3: Run runtime acceptance where supported**

Run the GCC minimal profile and sanitizer profile using the installed prefix:

```bash
AUTONOMY_CI_PREFIX=$PWD/build/ci-prefix cmake --preset autodriver-minimal-gcc -S autodriver
cmake --build --preset autodriver-minimal-gcc
ctest --preset autodriver-minimal-gcc
AUTONOMY_CI_PREFIX=$PWD/build/ci-prefix cmake --preset autodriver-sanitizers -S autodriver
cmake --build --preset autodriver-sanitizers
ASAN_OPTIONS=detect_leaks=1:halt_on_error=1 UBSAN_OPTIONS=print_stacktrace=1:halt_on_error=1 ctest --preset autodriver-sanitizers
git diff --exit-code
```

Expected on Ubuntu 22.04: both CTest runs pass, sanitizers report no findings, and Git reports no generated changes. On another platform, do not substitute a success claim; link the first successful GitHub Actions run in the baseline.

- [ ] **Step 4: Review scope and repository state**

Run:

```bash
git log --oneline d96b9508..HEAD
git diff --stat d96b9508..HEAD
git status --short
```

Confirm that no Phase 2 target decomposition, algorithm change, vendor SDK enablement, bridge change, README artwork change, or unrelated formatting appears.

- [ ] **Step 5: Commit documentation**

```bash
git add autodriver/docs/source/guide/testing.md docs/quality/engineering-baseline.md
git commit -m "docs: document autodriver quality workflow"
```

## Completion Gate

This plan is complete only when:

- the source and target inventory is validated by the machine-readable checker;
- no developer-home path remains in scanned build inputs;
- root configuration does not create or modify tracked source files;
- AutoDriver resolves both embedded and installed Autolink/Automsgs targets;
- developers and CI use the same named minimal presets;
- GCC, Clang, and sanitizer jobs exist, and at least one Ubuntu execution record is linked;
- the 17 AutoDriver tests are enumerated and the executed subset/result is recorded precisely;
- the current user's pre-existing dirty checkout remains untouched.

Phase 1 is not complete after this plan: `autonomy-minimal`, its sanitizer path, changed-file Clang-Tidy, Markdown/CMake lint policy, and full-stack CTest remain for the next implementation plan.
