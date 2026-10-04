# Autoviz string translations

Qt Linguist workflow for Autoviz UI strings (`tr()`).

## Layout

| File | Purpose |
|------|---------|
| `autoviz.ts` | English source catalog (`lupdate` extract) |
| `autoviz_zh_CN.ts` | Simplified Chinese |

Supported runtime languages: **English** (source, no `.qm`) and **简体中文** (`zh_CN`).

## Updating catalogs

From the `autoviz` package root:

```bash
python3 tools/translations/autoviz_lupdate.py
# optional: merge matching strings from QGroundControl
python3 tools/translations/autoviz_lupdate.py \
  --qgc-translations /path/to/qgroundcontrol/translations
```

The script:

1. Runs `lupdate` on `autoviz/` (C++ `tr()`).
2. Refreshes `autoviz.ts` and `autoviz_zh_CN.ts`.
3. Optionally merges matching English `<source>` strings from QGC.
4. Applies Autoviz-specific zh-CN fills.

Rebuild `autoviz` to compile `.qm` into the binary (`qt_add_translations` → `:/i18n`).

## Code conventions

- C++ widgets: `tr("...")` in `QObject` subclasses (`Q_OBJECT` required).
- Panel catalog strings live in C++ (`panel_catalog.cpp`); dock titles use `tr()` in
  `visualization_frame.cpp`.

Runtime language: Settings → Language, or system locale when set to “Follow system”.
