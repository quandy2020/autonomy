# Autoviz Widget Library

`autoviz::widget` is the single Qt 6 Widgets design system for Autoviz. It
consolidates the useful capabilities found in the projects under
`/home/quandy/workspace/github/Ui` without exposing their brand-specific APIs.

## Design rules

- Use semantic palette roles and `ThemeTokens`; do not hard-code a second
  Fluent, Material, or Ela theme.
- Apply `ApplyWidgetTheme()` to native Qt editors, model/view controls,
  date/time controls, sliders, dialogs, and standard container widgets.
- Prefer `QPalette`, `QStyle`, and local custom painting. Never install a
  process-wide stylesheet from a component.
- Keep painting allocation-free where practical. Cache derived pens, brushes,
  paths, and device-pixel-ratio-specific icons outside `paintEvent()`.
- Use Qt model/view for data collections. Do not create one widget per table or
  tree cell.
- Stop timers and animations while hidden or when reduced motion is enabled.
- Every interactive component must support keyboard focus, accessible names,
  high DPI, and both light and dark palettes.
- Public names describe semantics (`Button`, `InfoBar`, `Surface`) rather than
  an upstream toolkit (`MaterialButton`, `FluentCard`, and so on).

## Modules

- `foundation`: semantic colors, density, typography, radii, and motion.
- `input`: buttons, switches, numeric/vector fields, suggestions, and choices.
- `layout`: surfaces/cards, separators, collapsible panels, and responsive flow.
- `navigation`: breadcrumbs, command palette, workflow steps, and navigation.
- `feedback`: progress, information bars, skeletons, and toasts.
- `data`: DataGrid, PropertyEditor, logs, pagination, filters, loading overlays,
  and timeline.
- `display`: allocation-conscious status badges and live metric primitives.
- `overlay`: popovers, teaching tips, and drawers.
- `docking`: backend-neutral docking facade.

Include only the component headers that a translation unit needs. The umbrella
header `autoviz/common/widget/widget.hpp` is intended for prototypes and the
component gallery.

## Component gallery

Build the optional gallery when developing or reviewing widgets:

```bash
docker exec -w /workspace/autonomy SpaceHero \
  colcon build --packages-up-to autonomy --cmake-args \
  -DBUILD_TEST=OFF -DAUTOVIZ_BUILD_WIDGET_GALLERY=ON
```

Launch `build/autonomy/bin/autoviz_widget_gallery` in a graphical container.
For a headless construction and layout smoke test, run:

```bash
docker exec -w /workspace/autonomy \
  -e QT_QPA_PLATFORM=offscreen SpaceHero \
  ./build/autonomy/bin/autoviz_widget_gallery --smoke-test
```

## Tests

Widget tests use QtTest with the offscreen platform and have an independent
build switch, so unrelated domain tests remain disabled:

```bash
docker exec -w /workspace/autonomy SpaceHero \
  colcon build --packages-up-to autonomy --cmake-args \
  -DBUILD_TEST=OFF -DAUTOVIZ_BUILD_WIDGET_TESTS=ON
docker exec -w /workspace/autonomy SpaceHero \
  ctest --test-dir build/autonomy -R '^autoviz_widget_' --output-on-failure
```

## Third-party boundary

MIT/BSD projects are valid implementation sources only when their copyright
and license are retained and the file is recorded in `PROVENANCE.yaml`.
Qt Advanced Docking System remains an optional LGPL shared library behind the
docking facade. GPL projects are behavior references only; their code and
assets must not be copied into this Apache-2.0 core library.
