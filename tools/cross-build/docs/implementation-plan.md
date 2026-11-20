# cross-build Implementation Plan

> Execution: inline, using executing-plans. User explicitly requested design, coding and documentation now; all verification is deferred until their instruction.

Goal: implement five Profile based Linux C/C++ cross-build workflows.
Architecture: JSON-compatible profile.yaml, shared Python CLI, shared CMake toolchain, the existing Autonomy x86_64 image with both GNU compilers, per-Profile sysroots.
Tech stack: Python 3.9+, CMake 3.20+, Docker, rsync, SSH; no new Python dependency.
Spec: design.md.

## Constraints and review focus

Use tools/cross-build for target builds and docker/ for shared image builds. No tests, configure/build, image operations or board connections during this phase.
Focus: merged-/usr, wrong target ABI, escaping symlinks, failed sysroot replacement, container host/target dependency separation.

## Tasks

- [x] Write tests for Profile validation, symlink safety, ABI validation, rollback and argument forwarding; do not run.
- [x] Implement profiles, CMake configuration and shared metadata loader.
- [x] Implement sysroot staging, lock, normalization, manifest, structural checks and recoverable replacement.
- [x] Extend the shared Docker image and use it from build/install with separate output mounts and target pkg-config paths.
- [x] Write Make interface, CI include, C/C++ smoke fixture and usage/troubleshooting documents.
- [x] Record delivered files and pending validation; do not claim passing checks.

## Deferred validation

Run `make test`, `make check`, shell syntax checks and CMake toolchain checks only after user approval to start verification. Then prepare the shared image and smoke C/C++ for both ABIs with real sysroots; deployment and SDK checks need appropriate targets.

## Delivery checkpoint

All implementation and documentation tasks above are written. Verification is deliberately not executed: no unittest/check invocation, shell syntax check, CMake configure, image build, sysroot collection or target execution in this phase. The tests are unproven until run. No commit/push or top-level integration was performed.

Ruling: keep work in the requested directory in the current checkout because it began as user-created untracked skeleton files; no worktree move or unrelated file changes.
Ruling: write tests before implementation but defer red/green execution, following the user's explicit request to start verification only on a later instruction.
Ruling: same-Profile lock and retained previous-UUID directories provide recoverable publication; full cross-crash atomic replacement is outside this implementation.
