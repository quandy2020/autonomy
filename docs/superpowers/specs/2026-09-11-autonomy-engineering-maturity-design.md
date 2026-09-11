# Autonomy Engineering Maturity Improvement Design

Date: 2026-09-11

Status: Proposed
Scope: `autodriver/`, the Autonomy C++ build, CI, tests, release evidence, and source provenance

## 1. Context

Autonomy is a broad C++17 mobile-robot framework built around Autolink RT. It
contains communication integration, messages, hardware abstraction,
localization, mapping, planning, control, perception, behavior-tree tasks,
simulation, visualization, and deployment tooling.

The repository already has useful architectural foundations, but its quality
controls do not yet match its functional breadth. The principal gaps are:

- no CI path that builds and tests the core C++ projects;
- a large recursively collected `libautonomy` target with weak dependency
  boundaries;
- host-specific dependency discovery;
- limited integration, fault-injection, hardware-in-the-loop, and performance
  evidence;
- mixed project and imported-source provenance;
- no published compatibility, coverage, or benchmark evidence.

This design improves those areas incrementally. It does not rewrite algorithms,
replace Autolink, migrate the project to ROS 2, or claim functional-safety
certification.

## 2. Evidence and Constraints

### 2.1 Confirmed from source

- AutoDriver exposes explicit manager, module, driver, sink, registry, and
  chassis-safety abstractions.
- AutoDriver registers 17 CTest executables.
- Autonomy discovers all `*_test.cpp` files recursively, including tests copied
  with upstream components.
- the top-level build collects most `autonomy/**/*.cpp` files into one shared
  library;
- the only current GitHub Actions workflow validates Ansible;
- several hardware and application paths remain optional, stubbed, or dependent
  on locally installed SDKs.

### 2.2 Not established

The current review did not establish:

- a clean full build on every supported platform;
- unit-test pass rates;
- code coverage;
- latency, throughput, memory, or startup-time baselines;
- long-duration simulation or real-robot reliability;
- hardware-in-the-loop pass rates;
- functional-safety compliance.

Every implementation phase must preserve this distinction between source
presence and runtime evidence.

### 2.3 Working-tree protection

The repository contains unrelated in-progress changes, especially under
`autonomy/bridge/`, `automsgs/`, `ansible/`, configuration, and documentation.
Implementation must:

1. record `HEAD` and `git status --short` before each phase;
2. treat all pre-existing modifications as user-owned baseline state;
3. avoid restoring, staging, formatting, or rewriting unrelated files;
4. stage and commit only explicitly reviewed phase files;
5. stop when an intended edit overlaps an ambiguous in-progress change.

## 3. Goals

1. Make the supported build profiles reproducible on clean machines.
2. Add automated evidence for formatting, compilation, unit tests, and memory
   safety.
3. Reduce coupling by replacing the monolithic library with responsibility-based
   CMake targets.
4. Add system-level regression coverage for the principal robot workflows.
5. Establish AutoDriver fault, timing, and hardware validation.
6. Make releases auditable through provenance, compatibility, coverage, and
   benchmark documentation.

## 4. Non-Goals

- algorithm redesign or performance tuning without a measured baseline;
- immediate support for every optional vendor SDK in CI;
- replacing copied upstream code solely for stylistic consistency;
- introducing a second build system;
- certification claims or safety-integrity-level claims;
- one large repository-wide refactor.

## 5. Delivery Strategy

Work proceeds through six independently reviewable phases. A phase may begin
only after the previous phase's acceptance checks pass. Each phase must leave a
documented command that can reproduce its evidence.

### Phase 0: Baseline and ownership inventory

Produce a machine-readable and human-readable inventory covering:

- supported build profiles and platforms;
- first-party, adapted, generated, submodule, and vendor sources;
- CMake targets and direct dependencies;
- test ownership and runtime requirements;
- optional SDKs and the feature they enable;
- generated files and their output locations.

Acceptance criteria:

- no production source or runtime behavior changes;
- every current target is assigned to a responsibility domain;
- upstream tests are distinguishable from first-party integration tests;
- all build claims are labeled as observed, executed, or unverified.

### Phase 1: Portable build and continuous integration

#### Build profiles

Define two initial profiles:

1. `autodriver-minimal`: AutoDriver, stub chassis, no vendor camera or lidar
   SDKs, examples off, tests on;
2. `autonomy-minimal`: core common, transform, map/planning/control/task paths,
   tests on, GPU inference, visualization, simulation, gRPC, and vendor hardware
   off where dependency boundaries permit.

Profiles must be expressed as CMake presets or documented cache options. CI must
invoke the same profiles available to developers.

#### Portability fixes

- remove developer-specific absolute dependency paths;
- require optional dependency roots through cache variables or presets;
- emit generated headers only under the binary directory;
- ensure optional modules do not impose unrelated required dependencies;
- keep out-of-source builds as the supported path.

#### CI jobs

Add narrowly scoped jobs for:

- Markdown/CMake/whitespace checks;
- formatting verification;
- `autodriver-minimal` configure, build, and CTest;
- `autonomy-minimal` configure, build, and CTest;
- AddressSanitizer and UndefinedBehaviorSanitizer on the minimal profiles;
- optional Clang-Tidy checks on changed first-party C++ files.

CI must not require physical hardware, private SDKs, credentials, or
developer-specific paths.

Acceptance criteria:

- both minimal profiles configure from a clean checkout;
- CI executes core C++ tests, not only deployment checks;
- optional SDK absence produces a deliberate disabled feature or actionable
  configuration error;
- generated files do not dirty the source tree;
- sanitizer jobs report no findings in their selected tests.

### Phase 2: Responsibility-based CMake targets

Replace broad recursive inclusion incrementally with explicit domain targets:

- `autonomy_common`
- `autonomy_transform`
- `autonomy_map`
- `autonomy_localization`
- `autonomy_planning`
- `autonomy_control`
- `autonomy_perception`
- `autonomy_task`
- `autonomy_bridge`

Each target must declare its own sources, public headers, public dependencies,
private dependencies, compile definitions, and optional features. Compatibility
target aliases may temporarily preserve existing consumers.

Migration order:

1. common and transform;
2. map;
3. planning and control;
4. task;
5. localization and perception;
6. bridge and compatibility aggregate.

Acceptance criteria per migrated target:

- no recursive source glob controls its membership;
- it builds and links in isolation within a supported profile;
- public headers do not require unrelated module dependencies;
- its focused tests run through CTest;
- installed exports resolve from a clean consumer project.

### Phase 3: Workflow regression tests

Build deterministic tests around public interfaces rather than internal class
layout. Required scenarios are:

- navigation success from goal through planning and control command output;
- planning failure, recovery, retry, and terminal failure;
- cancellation producing a bounded-time stop command;
- person tracking producing target/path and obstacle-grid outputs;
- command watchdog producing a soft stop;
- component restart and stale-message rejection;
- queue pressure with documented drop/backpressure behavior;
- invalid configuration and unavailable optional backend behavior.

Tests should use fake time, in-process Autolink transport, deterministic maps,
recorded message fixtures, and stub hardware. Real hardware is reserved for
Phase 4.

Acceptance criteria:

- each scenario has explicit preconditions, outputs, timeout, and failure
  diagnostics;
- tests run headlessly in CI;
- cancellation and watchdog tests use bounded timing assertions;
- flaky tests are fixed or quarantined with an owner and expiry date.

### Phase 4: AutoDriver robustness and hardware evidence

Add a hardware validation matrix by device/backend and test:

- attach, start, sample, stop, and detach lifecycle;
- disconnect/reconnect and device replacement;
- timestamp discontinuity, jitter, drift, and wraparound;
- bounded queues, overload, and sample-drop accounting;
- malformed serial, CAN, UDP, and SDK data;
- SDK exception handling and process shutdown;
- repeated hotplug and long-duration operation;
- chassis watchdog, limit clamp, mode transition, and emergency stop;
- latency, rate, CPU, memory, and dropped-sample metrics.

Hardware-in-the-loop tests must be separated from pull-request CI and run on
labelled self-hosted hardware or as a documented release qualification step.

Acceptance criteria:

- each claimed production backend has a recorded qualification result;
- stubs and unimplemented backends are visibly marked in documentation and
  configuration validation;
- failures expose actionable diagnostics;
- benchmark inputs, environment, duration, and thresholds are versioned.

### Phase 5: Open-source governance and release evidence

- normalize remaining legacy project naming without discarding copyright;
- add a source-provenance manifest for adapted and copied components;
- document upstream version, local changes, license, and update procedure;
- add contribution, security-reporting, ownership, and review guidance;
- publish supported platform/compiler/dependency combinations;
- publish coverage and benchmark reports with reproducible commands;
- produce a versioned release checklist and changelog entry;
- provide a reproducible development image or immutable image reference.

Acceptance criteria:

- every imported source tree has traceable provenance and license information;
- a clean external user can follow one supported build path without local
  knowledge;
- release artifacts identify commit, configuration, compiler, and dependencies;
- README claims match the latest recorded verification evidence.

## 6. Architectural Rules

### 6.1 Dependency direction

Lower-level targets must not depend on task or application targets. Intended
direction:

```text
common <- transform <- map/localization <- planning/control <- task
   ^               ^                         ^                ^
   +---------------+--- perception ----------+                |
   +-------------------- bridge ------------------------------+
```

AutoDriver depends on Autolink and messages but does not depend on Autonomy
planning, control, task, or vehicle implementation.

### 6.2 Optional features

An optional feature must satisfy one of two contracts:

- disabled: no headers, libraries, or runtime files from that feature are
  required; or
- enabled: configuration fails early with the missing dependency and a precise
  installation/configuration message.

Silent partial implementations are not acceptable for production-labelled
backends.

### 6.3 Runtime and safety evidence

Unit tests validate local logic. Simulation validates workflow integration.
Hardware-in-the-loop validates device integration. Long-duration robot tests
validate operational reliability. None of these alone establishes functional
safety certification.

## 7. Error Handling and Diagnostics

New and migrated code must:

- return structured status at module boundaries;
- include component, backend, device/channel, and operation in diagnostics;
- distinguish unsupported, unavailable, invalid configuration, timeout, and
  internal failure;
- make queue overflow and dropped samples observable;
- make retry and fallback behavior explicit;
- guarantee idempotent stop/shutdown where practical;
- avoid converting configuration or dependency failures into late null-pointer
  behavior.

## 8. Verification Matrix

| Evidence | Pull request | Nightly | Release/HIL |
|---|:---:|:---:|:---:|
| Format and static checks | Required | Required | Required |
| Minimal configure/build | Required | Required | Required |
| Unit tests | Required | Required | Required |
| ASan/UBSan | Selected | Full supported set | Required |
| Workflow simulation | Selected | Full | Required |
| Recorded-data replay | Optional | Required | Required |
| Hardware-in-the-loop | No | Optional | Required per backend |
| Performance benchmark | No | Trend | Required |
| Long-duration run | No | Scheduled | Required |

Every report must contain the commit, preset, platform, compiler, dependency
versions, command, duration, and result.

## 9. Rollback and Compatibility

- keep each phase and module migration independently revertible;
- preserve existing executable names and installed interfaces until an explicit
  deprecation cycle is documented;
- add compatibility aliases before removing the aggregate library;
- do not combine algorithm behavior changes with build-target migration;
- retain the previous supported preset until its replacement passes equivalent
  tests;
- keep vendor hardware features behind explicit build options.

## 10. First Implementation Increment

The first increment covers Phase 0 and the smallest Phase 1 vertical slice:

1. add the target/dependency/test/provenance inventory;
2. remove the developer-specific OR-Tools search path;
3. introduce `autodriver-minimal` configuration;
4. add AutoDriver format, build, and CTest CI;
5. add one AutoDriver ASan/UBSan job;
6. document locally reproducible commands;
7. verify that no generated files dirty a clean source tree.

Autonomy minimal CI follows only after the AutoDriver slice is green, because it
has a materially larger dependency surface and overlaps current bridge/build
changes.

## 11. Success Criteria

This program is successful when:

- a clean external environment can reproduce supported builds;
- every first-party module has an explicit target owner and focused tests;
- core workflow regressions run automatically;
- hardware support claims are backed by recorded qualification evidence;
- failures and degraded modes are observable;
- releases carry traceable source, dependency, test, and performance evidence;
- repository documentation distinguishes confirmed source behavior, executed
  verification, static inference, and future recommendations.
