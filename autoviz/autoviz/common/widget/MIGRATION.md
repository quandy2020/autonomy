# UI project consolidation

This document is the capability-level migration plan for every repository in
`/home/quandy/workspace/github/Ui`. The destination is one stable API in
`autoviz::widget`; duplicate branded controls are intentionally not retained.

## Unified capability mapping

### Foundation and appearance

- Fluent-Qt, ElaWidgetTools, QFluentKit, and Qt-Fluent-Widgets theme managers
  map to `foundation/ThemeTokens` and the existing `autoviz::style` service.
- Upstream icon enums map to Autoviz resource names and `QIcon`; third-party
  fonts and GPL icon assets are not copied.
- Motion, spacing, radius, typography, focus, and state colors are semantic
  tokens shared by every component.

### Input

- Fluent/Material/Ela push buttons map to `Button` and `IconButton` variants.
- Switches map to `ToggleSwitch`; button groups map to `SegmentedControl`.
- Search and password inputs map to `SearchBox` and `PasswordBox`.
- Search inputs with model-backed completion map to `AutoSuggestBox`.
- Validated scalar inputs map to `NumberField`; grouped Cartesian values map
  to `Vector3Editor`.
- Native `QLineEdit`, `QComboBox`, `QSpinBox`, `QSlider`, and date/time editors
  remain the preferred implementation for standard field semantics;
  `ApplyWidgetTheme()` applies Autoviz tokens instead of forking behavior.

### Layout and containers

- Cards and setting cards map to `Surface`.
- Expandable setting groups map to `CollapsiblePanel`.
- Wrapping chip/tool layouts map to `FlowLayout`.
- Dividers map to `Separator`.
- Scroll areas, splitters, tabs, stacks, group boxes, and toolbars use Qt's
  native containers with Autoviz tokens. Reimplementing these mature controls
  would increase accessibility and maintenance risk without adding capability.

### Navigation

- Breadcrumb bars map to `Breadcrumb`.
- Fluent/Ela navigation panels map to `NavigationRail`.
- Searchable action launchers map to `CommandPalette`; ordered workflow state
  maps to `StepIndicator`.
- Pivot/tab-like compact navigation maps to `SegmentedControl` or `QTabBar`,
  selected according to whether the control changes a value or a document.

### Feedback and overlays

- Indeterminate/determinate spinners map to `ProgressRing`.
- Badges map to `StatusBadge`; message bars map to `InfoBar`.
- Snackbars map to `Toast`; applications should serialize multiple messages
  through their existing notification model instead of stacking widgets.
- Loading placeholders map to `Skeleton`.
- Flyouts map to `Popover`; guided contextual help maps to `TeachingTip`.
- Modal side and edge panels map to `Drawer`; blocking asynchronous work maps
  to `LoadingOverlay`.
- Anchored command menus stay native `QMenu` so their keyboard semantics are
  preserved.
- Dialogs and message boxes stay native `QDialog`/`QMessageBox` and receive
  Autoviz palette tokens.

### Data and visualization

- Empty collection states map to `EmptyState`.
- Tables, trees, lists, headers, delegates, and paged collections use Qt
  model/view. Component-library additions must be proxy models or delegates,
  never item widgets per cell.
- `DataGrid`, `PropertyEditor`, `LogView`, `Pagination`, and `FilterBar`
  provide the shared model/view layer for large live datasets and parameters.
- `TimelineScrubber` handles rosbag time and diagnostic event markers;
  `Sparkline` provides fixed-capacity live metrics without a chart framework.
- Charts, gauges, audio waves, image viewers, and code editors are domain
  widgets. They belong in separate optional modules and consume the same
  `ThemeTokens`; they are not linked into the lightweight common-control core.

### Windowing and docking

- Window chrome continues to use the existing Autoviz shell/window classes.
- `DockManager` is the public docking facade. Its default implementation uses
  `QDockWidget`.
- Qt Advanced Docking System can be supplied only as a separately replaceable
  LGPL shared-library backend. It is not copied or statically linked into the
  Apache-2.0 core.

## Definition of done for a component

1. The public API uses Autoviz semantic names and Qt value types.
2. Light/dark, Shell/Frost, disabled, hovered, pressed, focus, and error states
   have deterministic rendering.
3. Keyboard operation and accessible name/description are verified.
4. 100%, 150%, and 200% device pixel ratios render without clipped content.
5. Hidden widgets stop timers and animations; paint paths do no I/O or parsing.
6. QtTest covers state changes, signals, size hints, and show/hide lifecycle.
7. Any copied source is recorded in `PROVENANCE.yaml` with its license.
