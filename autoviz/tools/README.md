# Autoviz Tools

Development scripts for [Aviz](../) — structured like
[QGroundControl tools](https://github.com/mavlink/qgroundcontrol/tree/master/tools).

## Quick start

From `src/autonomy/autoviz`:

```bash
python3 tools/configure.py          # CMake configure (Ogre viewport always ON)
python3 tools/configure.py --release
# --ogre is a deprecated no-op
python3 tools/build.py              # Build autoviz target
python3 tools/translations/autoviz_lupdate.py
```

## Directory layout

```text
tools/
├── configure.py              # cmake -S autoviz -B autoviz/build
├── build.py                  # Build autoviz target
├── clean.py                  # Remove build/ and Python caches
├── common/                   # Shared Python helpers (paths, logging, proc)
├── translations/             # Qt Linguist / lupdate (QGC-style)
│   └── autoviz_lupdate.py
└── setup/                    # One-time asset / environment setup
```

Runtime helpers for packaging live under [`../deploy/`](../deploy/).

## Common tasks

| Task | Command |
|------|---------|
| Configure Debug | `python3 tools/configure.py` |
| Release build | `python3 tools/configure.py --release && python3 tools/build.py` |
| Update translations | `python3 tools/translations/autoviz_lupdate.py` |
| Clean build tree | `python3 tools/clean.py` |

## Translations

See [`translations/README.md`](translations/README.md) and [`../translations/README.md`](../translations/README.md).

Catalog files live in `../translations/` (`autoviz.ts` English source +
`autoviz_zh_CN.ts`). Update with `tools/translations/autoviz_lupdate.py`.
Optional: merge matching English strings from QGC `qgc_source_zh_CN.ts`.


## Python environment

Optional local venv:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e "tools/[dev]"
```
