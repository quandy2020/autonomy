# Argus apps: runner / benchmark / dataset readers / tools.
#
# Layout:
#   apps/
#   ├── dataset/     # KITTI / NCD / TUM RGB-D (+ Oxford Spires) readers
#   ├── runner/      # argus_runner + legacy offline_* binaries
#   ├── benchmark/   # argus_benchmark (+ eval_pose_traj alias)
#   └── tools/       # dataset_converter (+ slice_kitti); ARGUS_BUILD_TOOLS

add_subdirectory(dataset)
add_subdirectory(runner)
add_subdirectory(benchmark)
# tools/ is added from the root when ARGUS_BUILD_TOOLS=ON (independent of APPS).
