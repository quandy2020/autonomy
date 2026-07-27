# Autoviz translation tools — see also ../translations/README.md (catalog layout).

Run from the autoviz package root:

```bash
python3 tools/translations/autoviz_lupdate.py
python3 tools/translations/autoviz_lupdate.py --qgc-translations /path/to/qgroundcontrol/translations
```

This mirrors QGroundControl's `tools/translations/qgc_lupdate.py` workflow: refresh `.ts`
catalogs with `lupdate`, then merge matching strings from QGC locale files.
