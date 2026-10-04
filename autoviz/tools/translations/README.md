# Autoviz translation tools

See also [`../../translations/README.md`](../../translations/README.md).

```bash
# From autoviz package root
python3 tools/translations/autoviz_lupdate.py
python3 tools/translations/autoviz_lupdate.py \
  --qgc-translations /path/to/qgroundcontrol/translations
```

Produces:

- `translations/autoviz.ts` — English source catalog
- `translations/autoviz_zh_CN.ts` — Simplified Chinese

`autoviz_core_zh_cn_fill.py` fills core UI strings used by Autoviz (called from `autoviz_lupdate.py`).
