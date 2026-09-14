# Explicit Product Target Dependencies

## Goal

Remove the implicit dependency from Autonomy product executables and loadable
components to the `autonomy` umbrella target. Every product target must expose
its real domain dependencies at its CMake declaration site.

## Scope

This change covers the public super-project helpers `autonomy_binary()` and
`autonomy_component()` and every first-party call site. It does not split the
global Proto target, replace source globs, or redesign third-party projects.

## Target model

`autonomy_library()` remains the owner of domain libraries named
`autonomy_<module>`, with aliases such as `autonomy::planning`.

`autonomy_binary()` creates a process target and links only:

- the targets listed in its required `DEPENDENCIES` argument;
- `glog::glog` and `gflags::gflags`, which are process-wide runtime support;
- dependencies already exported transitively by the declared targets.

It must not link `${PROJECT_NAME}`, `autonomy`, or `autonomy::autonomy`
implicitly.

`autonomy_component()` creates a loadable shared library and links only:

- the targets listed in `PRIVATE_LINK_LIBS` and `PUBLIC_LINK_LIBS`;
- `autolink` unless `NO_AUTOLINK` is specified;
- `automsgs` when that embedded infrastructure target exists.

It must not require or link the `autonomy` umbrella target. Domain ownership is
expressed through `PRIVATE_LINK_LIBS`, while the existing infrastructure
injection remains automatic because it is part of the component runtime ABI.

## Interface contracts

The binary API remains:

```cmake
autonomy_binary(<name>
  SRCS <source>...
  DEPENDENCIES <target-or-module>...)
```

`DEPENDENCIES` is mandatory. `autonomy_resolve_dependency()` resolves domain
names, concrete target names, and installed-style aliases. A declaration with
no dependencies is a configure-time error; a genuinely standalone executable
must use a plain `add_executable()` instead.

The component API remains:

```cmake
autonomy_component(<name>
  SOURCES <source>...
  [NO_AUTOLINK]
  [PUBLIC_LINK_LIBS <target>...]
  PRIVATE_LINK_LIBS <target>...)
```

At least one explicit public or private library is mandatory. Existing
component callers use `PRIVATE_LINK_LIBS` for their owning domain library.

## Call-site migration

Each product entry point declares its smallest owning domain target:

- `autonomy.control` -> `control`
- `autonomy.task` -> `task`
- `autonomy.localization` -> `localization`
- `autonomy.perception` -> `perception`
- `autonomy.bridge` -> `bridge`
- `autonomy.planning` -> `planning`
- `autonomy.monitor` -> `system`
- `autonomy.foxglove_bridge` -> `visualization`
- `autonomy.orbisview` -> `orbisview`/`autonomy_orbisview`

Grid-map demo targets declare the narrow target required by their source; they
must not use the umbrella as a generic fallback.

Components declare their owning implementation target explicitly:

- `base_component` -> `base_component`'s existing domain dependencies, without
  creating a self-link;
- `follow_component` -> its existing perception/follow dependencies;
- `audio_component` -> its existing audio dependencies.

Where a component implementation target is itself the product DSO, the caller
must list the domain libraries used by its sources rather than its own target.

## Migration safety

Call sites and helper semantics change atomically. No commit may leave an active
two-mode state where some products silently inherit the umbrella.

Contract tests scan first-party CMake declarations and configure representative
targets. They must verify:

- missing binary dependencies fail at configure time;
- missing component libraries fail at configure time;
- generated link interfaces exclude `autonomy` and `autonomy::autonomy`;
- representative Minimal-profile products link their declared module targets;
- existing target names and install destinations remain unchanged.

## Verification

The completed change must pass:

1. CMake Python contract tests;
2. `cmake --preset autonomy-minimal`;
3. `cmake --build --preset autonomy-minimal -- -j1` in a stable build directory;
4. `ctest --preset autonomy-minimal --output-on-failure`;
5. the install-consumer test;
6. `git diff --check` for all changed files.

Failures caused by concurrent removal or reconfiguration of the shared build
directory must be reported separately and rerun in a stable window; they are
not accepted as successful verification.
