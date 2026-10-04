# Autoviz styles

Five noun modules under `:/autoviz/styles/`. C++ API lives in `autoviz/ui/theme/`.

| QSS | Role |
|-----|------|
| `theme.qss` | Application theme |
| `menu.qss` | Panels menu |
| `chrome.qss` | Dock, bars, layout, surfaces, viewport |
| `widget.qss` | Shared fields, groups, badges, buttons |
| `panel.qss` | All feature panels (`@section <panel>.<part>`) |

| C++ (`ui/theme/`) | Role |
|-------------------|------|
| `style.hpp` | `style::sheet` / `type` / `mark` |
| `glass.hpp` | Shell / Overlay tokens |
| `panel.hpp` | Panel chrome helpers + `AppThemeIds` |
| `application.hpp` | `ApplyAppTheme` / menu prep |
| `proxy.hpp` | `ProxyStyle` (Fusion proxy) |

```cpp
#include "autoviz/ui/theme/style.hpp"

style::sheet("theme");
style::sheet("chrome/dock");
style::sheet("widget/primary_button");
style::sheet("teleop");                 // all teleop.* sections
style::sheet("teleop/header");          // @section teleop.header
style::type(style::Role::Muted, 11);
style::mark(style::Mark::Rule);
```
