# CMake Distributed Module Management Design

## Goal

Evolve Autonomy from a configurable monolithic super-project into a collection
of well-bounded modules that can be built, tested, installed, and consumed with
only their declared dependencies.

## Constraints

- Preserve the existing root super-project workflow.
- Keep `autonomy-minimal` continuously buildable and testable.
- Do not split repositories until target, test, and Proto boundaries are clean.
- Preserve existing public C++ APIs and CTest names during the CMake migration.
- Keep optional third-party dependencies out of disabled modules and installed
  package metadata.

## Architecture

Each first-level directory below `autonomy/` is a module. A module owns its
library target, direct target dependencies, tests, generated messages, runtime
resources, and install rules. The root project owns option selection, dependency
graph validation, module traversal, package aggregation, and build profiles.

Shared capabilities are represented by small CMake interface targets rather
than one implicit core dependency bundle. Module metadata is declared once and
is used to derive traversal order, dependency discovery, package components,
and validation.

Companion projects such as Autolink, Automsgs, Autodriver, Autoviz, Autosim,
and OrbisView remain independently named projects. The root build may aggregate
them, but module targets must consume their exported targets rather than their
source-tree internals.

## Migration Stages

### 1. Module-scoped tests

Map each test source to its owning module target. Tests link that module instead
of the `autonomy` umbrella target. Shared test helpers remain in a dedicated
support target and must not reintroduce umbrella dependencies. Cross-module
tests declare extra dependencies explicitly.

### 2. Capability targets

Replace `autonomy_link_core()` with focused targets for base runtime, logging,
configuration, serialization, messaging, threading, math, vision, point-cloud,
optimization, and inference capabilities. Dependencies are `PUBLIC` only when
exposed by installed headers; implementation-only dependencies are `PRIVATE`.

### 3. Module Proto targets

Generate and export Proto libraries per owning module. A module links only its
own Proto target and directly imported Proto targets. gRPC service generation
remains conditional on the owning module and gRPC feature.

### 4. Single-source module metadata

Introduce a module registration API containing module name, module dependencies,
features, optional features, and component targets. Generate validation,
traversal, dependency groups, and package metadata from this registry.

### 5. Module-owned packaging

Each module declares its tests and installation resources locally. The root
package code aggregates export sets and package configuration. Installed CMake
components resolve only the dependencies required by requested modules.

### 6. Companion project boundaries

Prefer installed or workspace package targets for Autolink and Automsgs. Keep
source aggregation as an explicit developer mode. Do not make optional companion
applications implicit dependencies of `autonomy::autonomy`.

### 7. Build matrix

Add minimal, navigation, perception, runtime, visualization, full, CI, sanitizer,
and release profiles. Every supported profile verifies configure, build, CTest,
install, and an external `find_package` consumer.

## Failure Handling

CMake configuration must fail early when a requested module lacks a required
module or imported target. Error messages identify the module, missing target,
and controlling option. Optional backends remain disabled when unavailable and
must not leak unresolved targets into exported interfaces.

## Verification

Every migration stage follows a red-green cycle and ends with:

- CMake contract tests for the changed target graph.
- The complete `autonomy-minimal` build and CTest suite.
- The external install-consumer test.
- `git diff --check` on files changed by the stage.

Additional build profiles are required before declaring their corresponding
modules supported by the new architecture.
