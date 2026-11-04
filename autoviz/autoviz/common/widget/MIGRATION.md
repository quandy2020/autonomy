# UI project consolidation

This document is the capability-level migration plan for every repository in
`/home/quandy/workspace/github/Ui`. The destination is one stable API in
`autoviz::widget`; duplicate branded controls are intentionally not retained.

## Unified capability mapping

### Foundation and appearance

- The `Elevatormusic/apple-hig` reference maps to a cross-platform desktop
  appearance profile in `ThemeTokens`, `ApplyWidgetTheme()`, and local custom
  painting. Only transferable principles are used; Apple platform chrome,
  proprietary fonts, SF Symbols, templates, and assets are excluded.
- Fluent-Qt, ElaWidgetTools, QFluentKit, and Qt-Fluent-Widgets theme managers
  map to `foundation/ThemeTokens` and the existing `autoviz::style` service.
- Upstream animation settings map to `foundation/MotionPolicy`. System mode
  honors `QStyle::SH_Widget_Animate`, `AUTOVIZ_REDUCED_MOTION`, and
  `PREFERS_REDUCED_MOTION`; component durations should pass through
  `MotionPolicy::ResolveDuration()`.
- Upstream icon enums map to Autoviz resource names and `QIcon`; third-party
  fonts and GPL icon assets are not copied.
- Motion, spacing, radius, typography, focus, and state colors are semantic
  tokens shared by every component.

### Input

- Fluent/Material/Ela push buttons map to the primary, secondary, subtle, and
  danger `Button` variants; icon-only actions map to `IconButton`.
- Switches map to `ToggleSwitch`; button groups map to `SegmentedControl`.
- Search and password inputs map to `SearchBox` and `PasswordBox`.
- Search inputs with model-backed completion map to `AutoSuggestBox`.
- Validated scalar inputs map to `NumberField`; grouped Cartesian values map
  to `Vector3Editor`.
- Shortcut/key-binding editors map to `ShortcutRecorder`. Cross-field dirty,
  required, validator, and submit state maps to `FormController`.
- Color wells and palette buttons map to `ColorPicker`; browse/drop targets
  map to `FileDropZone`.
- Multi-value combo boxes map to `MultiSelectComboBox`; hierarchical selectors
  map to `TreeSelect`; star inputs map to `Rating`.
- Native `QLineEdit`, `QComboBox`, `QSpinBox`, `QSlider`, and date/time editors
  remain the preferred implementation for standard field semantics;
  `ApplyWidgetTheme()` applies Autoviz tokens instead of forking behavior.

### Layout and containers

- Cards and setting cards map to `Surface`.
- Tags and filter chips map to `Tag`; identity thumbnails map to `Avatar`.
- Expandable setting groups map to `CollapsiblePanel`.
- Wrapping chip/tool layouts map to `FlowLayout`.
- Dividers map to `Separator`.
- Scroll areas, splitters, tabs, stacks, and group boxes use Qt's
  native containers with Autoviz tokens. Reimplementing these mature controls
  would increase accessibility and maintenance risk without adding capability.

### Navigation

- Breadcrumb bars map to `Breadcrumb`.
- Fluent/Ela navigation panels map to `NavigationRail`.
- Searchable action launchers map to `CommandPalette`; ordered workflow state
  maps to `StepIndicator`.
- QAction-based toolbars and command surfaces map to `CommandBar`; carousel and
  compact page indicators map to `PipsPager`.
- Pivot/tab-like compact navigation maps to `SegmentedControl` or `QTabBar`,
  selected according to whether the control changes a value or a document.

### Feedback and overlays

- Indeterminate/determinate spinners map to `ProgressRing`.
- Linear determinate and busy progress maps to the native-accessible
  `ProgressBar` wrapper.
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
- Master-detail layouts map to `ListDetailView`, which retains a single
  virtualized `QListView` and consumer-owned detail widget.
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

## Source-project closure

Every public control family found in the six source repositories has one of
the mappings below. A mapping is the migration result: branded duplicate
classes are not retained when Qt already supplies the same semantics.

### Fluent-Qt (MIT)

- Foundation, design, motion, and status modules map to `ThemeTokens`,
  `MotionPolicy`, `StatusBadge`, `InfoBar`, `ProgressBar`, and `ProgressRing`.
- Basic input and text-field modules map to the Autoviz input components or
  themed native Qt editors; collections and navigation map to `DataGrid`,
  `ListDetailView`, `NavigationRail`, `Breadcrumb`, and `PipsPager`.
- Dialog/flyout and menu/toolbar modules map to native dialogs and menus plus
  `Popover`, `TeachingTip`, `Drawer`, `CommandBar`, and `CommandPalette`.
- Charts, particles, spatial/3D, WebAssembly, and platform plugins are domain
  modules, not common controls; they stay in Autoviz rendering or optional
  platform targets and consume the same tokens.

### ElaWidgetTools (MIT)

- Buttons, toggles, text fields, suggestions, multi-select, navigation,
  breadcrumbs, cards, feedback, tables, logs, docking, and flow layout map to
  the corresponding Autoviz components and `DockManager`.
- Calendar and date/time pickers use `QCalendarWidget`, `QDateEdit`,
  `QTimeEdit`, or `QDateTimeEdit`; LCD output uses `QLCDNumber`.
- Roller/roller-picker uses a themed `QComboBox` or `QListView`; ribbon,
  app/status/tool bars use `QToolBar`, `CommandBar`, and the Autoviz shell.
- Key binding maps to `ShortcutRecorder`; action commander maps to
  `CommandPalette`. DXGI, Mica, blur, and native shadow helpers remain in
  platform code because they are window-system backends, not widgets.

### qt-material-widgets (BSD-3-Clause)

- App bar, paper/card, flat/raised/floating/icon buttons, avatar, badge,
  progress, steps, tabs, text input, toggle, autocomplete, drawer, and
  snackbar map to Autoviz components or themed native Qt controls.
- Check box, radio button, combo box, list, menu, table, scrollbar, and slider
  deliberately remain native Qt model/view controls. This preserves native
  keyboard and accessibility behavior while `ApplyWidgetTheme()` supplies the
  visual tokens.

### Qt Advanced Docking System (LGPL-2.1)

- Panel registration, placement, visibility, and layout persistence map to
  `DockManager`; its Apache core backend uses `QDockWidget`.
- Auto-hide, floating-container, tab-bar, overlay, splitter, focus, and drag
  preview internals are not copied. Applications needing exact ADS behavior
  must provide the separately linked LGPL backend described above.

### QFluentKit (GPL-3.0, reference only)

- Cards/settings, input, navigation, pagination, progress, info bars,
  teaching tips, flyouts, and adaptive layouts map to Autoviz components.
- Date/time, menu, model/view, splitter, stacked widget, scrollbar, and text
  editing use native Qt controls with Autoviz tokens.
- No QFluentKit source, stylesheet, icon font, image, or generated asset is
  copied into the Apache-2.0 core.

### Qt-Fluent-Widgets (GPL-3.0, reference only)

- Color/drop inputs, multi/tree selection, rating, tags, avatar, shortcut,
  command bar, pager, progress, skeleton, timeline, flyout, toast, teaching
  tip, drawer, and watermark map to their `autoviz::widget` equivalents.
- Date/time, calendar, list/tree/table, tabs, menus, scrollbars, splitters,
  stacks, toolboxes, and standard editors use themed native Qt controls.
- Audio waveforms, charts, chat, code editing, WebEngine, image crop/compare/
  magnify, and platform window effects are domain or optional modules and do
  not increase the dependency surface of the common-control library.
- No GPL implementation, stylesheet, icon font, or asset is copied.

## Definition of done for a component

1. The public API uses Autoviz semantic names and Qt value types.
2. Light/dark, Shell/Frost, disabled, hovered, pressed, focus, and error states
   have deterministic rendering.
3. Keyboard operation and accessible name/description are verified.
4. 100%, 150%, and 200% device pixel ratios render without clipped content;
   representative controls are rendered at all three ratios in QtTest.
5. Hidden widgets stop timers and animations; paint paths do no I/O or parsing.
6. QtTest covers state changes, signals, size hints, and show/hide lifecycle.
7. Any copied source is recorded in `PROVENANCE.yaml` with its license.
