# Autoviz Tools

Development scripts for [Aviz](../) — structured like
[QGroundControl tools](https://github.com/mavlink/qgroundcontrol/tree/master/tools).

## Quick start

From `src/autonomy/autoviz`:

```bash
python3 tools/configure.py --ogre --qml   # CMake configure (BUILD_AUTOVIZ=ON)
python3 tools/build.py                    # Build autoviz target
python3 tools/translations/autoviz_lupdate.py
```

## Directory layout

```text
tools/
├── configure.py              # CMake configure wrapper (Autonomy super-project)
├── build.py                  # Build autoviz target
├── clean.py                  # Remove build/ and Python caches
├── common/                   # Shared Python helpers (paths, logging, proc)
├── translations/             # Qt Linguist / lupdate (QGC-style)
│   └── autoviz_lupdate.py
├── setup/                    # One-time asset / environment setup
│   └── copy_qgc_drone_meshes.sh
```

Runtime helpers installed with the package live under [`../scripts/`](../scripts/)
(`mcap_to_record.py`, `publish_test_sensors.py`, BICMap examples, desktop install).

## Common tasks

| Task | Command |
|------|---------|
| Configure Debug + Ogre + QML | `python3 tools/configure.py --ogre --qml` |
| Release build | `python3 tools/configure.py --release --ogre --qml && python3 tools/build.py` |
| Update translations | `python3 tools/translations/autoviz_lupdate.py` |
| Copy QGC F450 meshes | `tools/setup/copy_qgc_drone_meshes.sh [QGC_ROOT]` |
| Publish test sensor data | `python3 ../scripts/publish_test_sensors.py` |
| Clean build tree | `python3 tools/clean.py` |

## Translations

See [`translations/README.md`](translations/README.md) and [`../translations/README.md`](../translations/README.md).

Catalog files live in `../translations/`; update them with `tools/translations/autoviz_lupdate.py`.
Matching English strings are merged from QGC `qgc_source_*.ts` when available.

## Python environment

Optional local venv (for future `tools/tests/`):

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -e "tools/[dev]"
```
