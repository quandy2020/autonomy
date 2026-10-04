# Autoviz UI layout

Sources under `autoviz/ui/` are grouped by **role**; file names are short nouns
(the folder already carries the category).

| Path | Role |
|------|------|
| `frame.hpp` / `frame.cpp` | Thin `VisualizationFrame` facade (owns collaborators) |
| `frame_layout.hpp/.cpp` | `FrameLayout` — dock host / tiling / expand |
| `frame_panels.hpp/.cpp` | `FramePanels` — create/wire feature panels |
| `frame_viewport.hpp/.cpp` | `FrameViewport` — render windows / tools / HUD |
| `frame_chrome.hpp/.cpp` | `FrameChrome` — menus, toolbar, status |
| `frame_session.hpp/.cpp` | `FrameSession` — config I/O, DnD, timers |
| `frame_detail.hpp` | Shared helpers used by multiple frame TUs |
| `viewport_panel` `record_drop_overlay` | Viewport dock entry; record DnD overlay |
| `panel_host` `viewport_*` | Panel host + viewport chrome widgets |

`VisualizationFrame` is a composition root: public API + Qt slots/overrides forward to
collaborators. Collaborators are friends of each other and hold their own state.
| `app/` | Preferences, settings, i18n, splash, icons |
| `theme/` | Stylesheet API, glass tokens, Fusion proxy |
| `panel/` | Dock / catalog / title tools / add-panel |
| `displays/` | Displays panel + tree + add-display |
| `views/` | Views panel |
| `inspector/` | Property / selection / tool properties |
| `dialog/` | Import record and shared dialog helpers |
| `raw/` | Raw messages panel |
| `tf_tree/` | TF tree panel |
| `time/` | Time panel |
| `channels/` | Channels panel |
| `channel_graph/` | Channel graph |
| `image/` `map/` `plot/` `publish/` `service/` `teleop/` | Feature panels |

## Naming

- Prefer `#include "autoviz/ui/<dir>/<noun>.hpp"` (or `autoviz/ui/<noun>.hpp` for top-level shell).
- Drop redundant prefixes already implied by the folder
  (`panel_dock_widget` → `panel/dock`, `displays_panel` → `displays/panel`).
- Keep C++ type names stable unless a rename is intentional
  (`VisualizationFrame` lives in `frame.hpp`).
