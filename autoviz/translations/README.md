# Autoviz string translations

Autoviz uses the Qt Linguist workflow, aligned with
[QGroundControl translations](https://github.com/mavlink/qgroundcontrol/tree/master/translations).

## Layout

| File | Purpose |
|------|---------|
| `autoviz.ts` | Base catalog (English source strings from `lupdate`) |
| `autoviz_<locale>.ts` | Per-locale catalogs (same locale set as QGC `qgc_source_*.ts`) |

Supported locales: `az_AZ`, `bg_BG`, `de_DE`, `el_GR`, `eo`, `es_ES`, `fi_FI`, `fr_FR`,
`he_IL`, `it_IT`, `ja_JP`, `ko_KR`, `nb_NO`, `nl_NL`, `no_NO`, `pl_PL`, `pt_PT`, `ru_RU`,
`sv_SE`, `tr_TR`, `uk_UA`, `zh_CN`, `zh_TW`.

## Updating catalogs

From `src/autonomy/autoviz`:

```bash
python3 tools/translations/autoviz_lupdate.py \
  --qgc-translations /path/to/qgroundcontrol/translations
```

The script:

1. Runs `lupdate` on `autoviz/` (C++) and `qml/` (QML `qsTr()`).
2. Refreshes `autoviz.ts` and all `autoviz_<locale>.ts` files.
3. Merges **matching English `<source>` strings** from QGC `qgc_source_<locale>.ts`.
4. Applies Aviz-specific zh-CN overrides (Vehicle 3D panel, etc.).

Rebuild `autoviz` to compile `.qm` files into the binary (`qt_add_translations`).

## Code conventions

- C++ widgets: `tr("...")` in `QObject` subclasses (`Q_OBJECT` required).
- QML: `qsTr("...")`.
- Panel catalog strings live in C++ (`panel_catalog.cpp`); dock titles use `tr()` in
  `visualization_frame.cpp`.

Runtime language follows the system locale (`QLocale::system()`).
