# Autoviz Examples

## Tutorial publishers (Python / C++)

Live Autolink publishers for Autoviz Display/Panel smoke tests:

| Lang | Path | Run |
|------|------|-----|
| Python | [`python/`](python/) | `/usr/bin/python3 examples/python/01_tutorial_poses.py` |
| C++ | [`cpp/`](cpp/) | `./build/bin/examples/autoviz_cpp_01_poses` |

Same channel names across both. See each subdirectory README for the full 01–24 index.

---

## BICMap catalog (Strata)

BICMap router scenarios for live preview and headless regression tests. Layout follows
[ROS 2 RViz](https://github.com/ros2/rviz) display tests (`rviz_default_plugins/test/…`)
and publishers (`rviz_default_plugins/publishers/…`).

## Catalog

36 router examples are listed in [`catalog.yaml`](catalog.yaml). Each entry maps to:

| Component | Location |
|-----------|----------|
| Live publisher (C++) | `tools/bicmap_example_publisher` |
| Live publisher (Python) | `scripts/publish_bicmap_example.py` |
| Headless gtest | `tests/bicmap_examples_test.cpp` |
| Session config | `config/default.autoviz` (Fixed Frame=`map`) |

Keep all four in sync. Verify with:

```bash
python3 examples/verify_catalog.py
```

## Live preview

Terminal 1 — publish one example:

```bash
scripts/run_bicmap_example.sh Slam publish
# or
python3 scripts/publish_bicmap_example.py --example Slam
```

Terminal 2 — open autoviz:

```bash
./build/bin/autoviz -c config/default.autoviz
```

One-shot (publisher + GUI):

```bash
scripts/run_bicmap_example.sh Slam autoviz
```

Cycle all 36 examples:

```bash
scripts/run_bicmap_example.sh "" cycle
```

List names:

```bash
./build/bin/autoviz_bicmap_publisher --list
```

## Headless tests

Configure with tests enabled, then run:

```bash
python3 tools/configure.py --tests
cmake --build build --target autoviz_bicmap_examples_test
examples/run_all_tests.sh
```

Or via CTest (after `-DBUILD_AUTOVIZ_TESTS=ON`):

```bash
ctest -R 'autoviz_(examples_catalog|bicmap_examples)' --output-on-failure
```

Custom target:

```bash
cmake --build build --target autoviz_examples_tests
```

Tests exercise each display’s `processMessage` / `onDraw` path without a window,
similar to RViz `*_display_test.cpp` (non-visual) suites.

## Categories

| Category | Count | Examples |
|----------|------:|----------|
| indoor | 12 | Slam, BuildMap, PointCloud, … |
| base | 11 | MapTools, POIMarkers, PathPlanning, … |
| scene | 8 | RobotGuideTour, SecurityPatrol, … |
| outdoor | 4 | OutdoorBuildings, OutdoorHdMap, … |
| expand | 3 | POIadvancedLabel, MapEditor, GraphicDrawing |
