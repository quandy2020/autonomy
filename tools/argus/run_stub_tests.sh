#!/usr/bin/env bash
# Compile/run Argus unit tests that do not require automsgs / full repo CMake.
# Usage: tools/argus/run_stub_tests.sh
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
SRC="$ROOT/autonomy/localization/argus"
OUT="${ARGUS_STUB_OUT:-/tmp/argus_stub_tests}"
mkdir -p "$OUT"

EIGEN_INC="${EIGEN_INC:-/usr/include/eigen3}"
if [[ ! -f "$EIGEN_INC/Eigen/Core" ]]; then
  EIGEN_INC="/opt/homebrew/include/eigen3"
fi
if [[ ! -f "$EIGEN_INC/Eigen/Core" ]]; then
  echo "Eigen3 headers not found (set EIGEN_INC)" >&2
  exit 1
fi

OCV_INC="${OpenCV_INCLUDE_DIR:-}"
OCV_LIB="${OpenCV_LIB_DIR:-}"
if [[ -z "$OCV_INC" ]]; then
  for c in /opt/homebrew/Cellar/opencv/*/include/opencv5 \
           /opt/homebrew/opt/opencv/include/opencv5 \
           /usr/include/opencv4 \
           /usr/local/include/opencv4; do
    if [[ -f "$c/opencv2/core.hpp" ]]; then
      OCV_INC="$c"
      break
    fi
  done
fi
if [[ -z "$OCV_LIB" ]]; then
  for c in /opt/homebrew/opt/opencv/lib \
           /opt/homebrew/Cellar/opencv/*/lib \
           /usr/lib/x86_64-linux-gnu \
           /usr/local/lib; do
    if [[ -e "$c/libopencv_core.so" || -e "$c/libopencv_core.dylib" || -e "$c/libopencv_core.a" ]]; then
      OCV_LIB="$c"
      break
    fi
  done
fi

CXXFLAGS=(-std=c++17 -O1 -I"$SRC" -I"$EIGEN_INC" -pthread)
LDFLAGS=(-pthread)
OCV_LDFLAGS=()
if [[ -n "$OCV_LIB" ]]; then
  OCV_LDFLAGS+=(-L"$OCV_LIB" -Wl,-rpath,"$OCV_LIB")
fi
# OpenCV 5: calib3d→calib; undistortPoints moved to geometry.
OCV_CALIB_LIB=opencv_calib3d
OCV_EXTRA_LIBS=()
if [[ -n "$OCV_LIB" ]]; then
  if [[ -e "$OCV_LIB/libopencv_calib.dylib" || -e "$OCV_LIB/libopencv_calib.so" || -e "$OCV_LIB/libopencv_calib.a" ]]; then
    OCV_CALIB_LIB=opencv_calib
  fi
  if [[ -e "$OCV_LIB/libopencv_geometry.dylib" || -e "$OCV_LIB/libopencv_geometry.so" || -e "$OCV_LIB/libopencv_geometry.a" ]]; then
    OCV_EXTRA_LIBS+=(-lopencv_geometry)
  fi
fi

echo "==> test_transform"
c++ "${CXXFLAGS[@]}" \
  "$SRC/argus/common/math/transform/rigid_transform.cpp" \
  "$SRC/argus/common/math/transform/timestamped_transform.cpp" \
  "$SRC/argus/common/math/transform/transform_interpolation_buffer.cpp" \
  "$SRC/test/test_transform.cpp" \
  "${LDFLAGS[@]}" -o "$OUT/test_transform"
"$OUT/test_transform"

echo "==> test_relocalization"
c++ "${CXXFLAGS[@]}" \
  "$SRC/argus/common/math/transform/rigid_transform.cpp" \
  "$SRC/argus/relocalization/config.cpp" \
  "$SRC/argus/relocalization/thread_pool.cpp" \
  "$SRC/argus/relocalization/prior_map.cpp" \
  "$SRC/argus/relocalization/scan_context/codec.cpp" \
  "$SRC/argus/relocalization/scan_context/scan_context.cpp" \
  "$SRC/argus/relocalization/bbs/search.cpp" \
  "$SRC/argus/relocalization/bbs/engine.cpp" \
  "$SRC/argus/relocalization/manager.cpp" \
  "$SRC/test/test_relocalization.cpp" \
  "${LDFLAGS[@]}" -o "$OUT/test_relocalization"
"$OUT/test_relocalization"

echo "==> test_pose_eval"
c++ "${CXXFLAGS[@]}" \
  "$SRC/argus/common/math/transform/rigid_transform.cpp" \
  "$SRC/argus/relocalization/prior_map.cpp" \
  "$SRC/argus/relocalization/pose_eval.cpp" \
  "$SRC/test/test_pose_eval.cpp" \
  "${LDFLAGS[@]}" -o "$OUT/test_pose_eval"
(cd "$OUT" && ./test_pose_eval)

echo "==> test_ikd_tree"
c++ "${CXXFLAGS[@]}" \
  "$SRC/argus/map/ikd_tree.cpp" \
  "$SRC/test/test_ikd_tree.cpp" \
  "${LDFLAGS[@]}" -o "$OUT/test_ikd_tree"
"$OUT/test_ikd_tree"

echo "==> test_degeneracy_detector"
c++ "${CXXFLAGS[@]}" \
  "$SRC/argus/perception/degeneracy_detector.cpp" \
  "$SRC/argus/fusion/anisotropic_covariance.cpp" \
  "$SRC/test/test_degeneracy_detector.cpp" \
  "${LDFLAGS[@]}" -o "$OUT/test_degeneracy_detector"
"$OUT/test_degeneracy_detector"

echo "==> test_confidence_gating"
c++ "${CXXFLAGS[@]}" \
  "$SRC/argus/fusion/confidence_gating.cpp" \
  "$SRC/argus/perception/sensor_health_monitor.cpp" \
  "$SRC/test/test_confidence_gating.cpp" \
  "${LDFLAGS[@]}" -o "$OUT/test_confidence_gating"
"$OUT/test_confidence_gating"

echo "==> test_intensity_assist"
c++ "${CXXFLAGS[@]}" \
  "$SRC/argus/common/math/transform/rigid_transform.cpp" \
  "$SRC/argus/fusion/intensity_assist.cpp" \
  "$SRC/argus/relocalization/loop_closure/consensus.cpp" \
  "$SRC/test/test_intensity_assist.cpp" \
  "${LDFLAGS[@]}" -o "$OUT/test_intensity_assist"
"$OUT/test_intensity_assist"

STUB_INC="$ROOT/tools/argus/stub_include"

OCC_SRCS=(
  "$SRC/argus/map/occupancy_grid_subgrid.cpp"
  "$SRC/argus/map/occupancy_grid_map.cpp"
  "$SRC/argus/map/occupancy_grid_builder.cpp"
)

VOXEL_SRCS=(
  "$SRC/argus/map/voxel_map_config.cpp"
  "$SRC/argus/map/voxel_body_cov.cpp"
  "$SRC/argus/map/lio_frame_buffers.cpp"
  "$SRC/argus/map/voxel_octo_tree.cpp"
  "$SRC/argus/map/voxel_map.cpp"
)

if [[ -n "$OCV_INC" ]]; then
  echo "==> test_occupancy_grid (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/argus/common/math/transform/rigid_transform.cpp" \
    "${OCC_SRCS[@]}" \
    "$SRC/test/test_occupancy_grid.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" -lopencv_core -lopencv_imgproc \
    -o "$OUT/test_occupancy_grid"
  "$OUT/test_occupancy_grid"

  echo "==> test_map_builder (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/argus/common/math/transform/rigid_transform.cpp" \
    "$SRC/argus/map/ikd_tree.cpp" \
    "$SRC/argus/map/map_session.cpp" \
    "$SRC/argus/map/visual_point.cpp" \
    "$SRC/argus/map/voxel_points.cpp" \
    "$SRC/argus/map/sub_sparse_map.cpp" \
    "$SRC/argus/map/visual_map.cpp" \
    "${OCC_SRCS[@]}" \
    "$SRC/argus/mapping/map_package.cpp" \
    "$SRC/argus/mapping/map_builder.cpp" \
    "$SRC/argus/relocalization/prior_map.cpp" \
    "$SRC/test/test_map_builder.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -o "$OUT/test_map_builder"
  (cd "$OUT" && ./test_map_builder)

  echo "==> test_production_e2e (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/argus/common/math/transform/rigid_transform.cpp" \
    "$SRC/argus/backend/pose_graph/pose_graph.cpp" \
    "$SRC/argus/frontend/sensor_sync.cpp" \
    "$SRC/argus/map/ikd_tree.cpp" \
    "$SRC/argus/map/map_session.cpp" \
    "$SRC/argus/map/visual_point.cpp" \
    "$SRC/argus/map/voxel_points.cpp" \
    "$SRC/argus/map/sub_sparse_map.cpp" \
    "$SRC/argus/map/visual_map.cpp" \
    "${OCC_SRCS[@]}" \
    "$SRC/argus/mapping/map_package.cpp" \
    "$SRC/argus/mapping/map_builder.cpp" \
    "$SRC/argus/perception/degeneracy_detector.cpp" \
    "$SRC/argus/relocalization/prior_map.cpp" \
    "$SRC/argus/relocalization/scan_context/codec.cpp" \
    "$SRC/argus/relocalization/scan_context/scan_context.cpp" \
    "$SRC/argus/relocalization/bbs/search.cpp" \
    "$SRC/argus/relocalization/bbs/engine.cpp" \
    "$SRC/test/test_production_e2e.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -o "$OUT/test_production_e2e"
  (cd "$OUT" && ./test_production_e2e)

  echo "==> test_loop_closure (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" \
    "$SRC/argus/common/math/transform/rigid_transform.cpp" \
    "$SRC/argus/backend/pose_graph/pose_graph.cpp" \
    "$SRC/argus/relocalization/scan_context/codec.cpp" \
    "$SRC/argus/relocalization/loop_closure/scan_context/detector.cpp" \
    "$SRC/argus/relocalization/loop_closure/consensus.cpp" \
    "$SRC/argus/relocalization/loop_closure/cross_session.cpp" \
    "$SRC/argus/relocalization/loop_closure/manager.cpp" \
    "$SRC/argus/map/map_session.cpp" \
    "$SRC/test/test_loop_closure.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" -lopencv_core \
    -o "$OUT/test_loop_closure"
  "$OUT/test_loop_closure"

  echo "==> test_dynamic_filter (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/argus/frontend/dynamic_filter_factory_config.cpp" \
    "$SRC/argus/frontend/spatiotemporal_dynamic_filter.cpp" \
    "$SRC/argus/frontend/dynamic_filter_base.cpp" \
    "$SRC/argus/frontend/semantic_dynamic_filter.cpp" \
    "$SRC/test/test_dynamic_filter.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -o "$OUT/test_dynamic_filter"
  "$OUT/test_dynamic_filter"

  echo "==> test_cross_session (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" \
    "$SRC/argus/common/math/transform/rigid_transform.cpp" \
    "$SRC/argus/backend/pose_graph/pose_graph.cpp" \
    "$SRC/argus/relocalization/loop_closure/cross_session.cpp" \
    "$SRC/argus/map/map_session.cpp" \
    "$SRC/test/test_cross_session.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" -lopencv_core \
    -o "$OUT/test_cross_session"
  "$OUT/test_cross_session"

  echo "==> test_map_cleaning (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/argus/map/map_confidence.cpp" \
    "$SRC/argus/map/map_self_cleaning.cpp" \
    "${VOXEL_SRCS[@]}" \
    "$SRC/argus/perception/degeneracy_detector.cpp" \
    "$SRC/argus/fusion/anisotropic_covariance.cpp" \
    "$SRC/test/test_map_cleaning.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" -lopencv_core \
    -o "$OUT/test_map_cleaning"
  "$OUT/test_map_cleaning"

  echo "==> test_semantic_dynamic_filter (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/argus/frontend/dynamic_filter_factory_config.cpp" \
    "$SRC/argus/frontend/semantic_dynamic_filter.cpp" \
    "$SRC/argus/frontend/dynamic_filter_base.cpp" \
    "$SRC/argus/frontend/spatiotemporal_dynamic_filter.cpp" \
    "$SRC/test/test_semantic_dynamic_filter.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -o "$OUT/test_semantic_dynamic_filter"
  "$OUT/test_semantic_dynamic_filter"

  echo "==> verify_kitti (real data if present)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/apps/datasets/kitti_dataset.cpp" \
    "$SRC/apps/verify_kitti.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -o "$OUT/verify_kitti"
  KITTI_ROOT="${ARGUS_KITTI_ROOT:-}"
  if [[ -z "$KITTI_ROOT" && -d "$SRC/testdata/kitti_real/velodyne" ]]; then
    KITTI_ROOT="$SRC/testdata/kitti_real"
  fi
  if [[ -n "$KITTI_ROOT" && -d "$KITTI_ROOT" ]]; then
    "$OUT/verify_kitti" "$KITTI_ROOT" --max-frames 3
  else
    echo "    skip (set ARGUS_KITTI_ROOT or slice_kitti → testdata/kitti_real)"
  fi

  echo "==> verify_oxford_datasets (layout smoke)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/apps/datasets/pcd_io.cpp" \
    "$SRC/apps/datasets/oxford_spires_options.cpp" \
    "$SRC/apps/datasets/oxford_spires_dataset.cpp" \
    "$SRC/apps/datasets/newer_college_options.cpp" \
    "$SRC/apps/datasets/newer_college_dataset.cpp" \
    "$SRC/apps/verify_oxford_datasets.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs \
    -o "$OUT/verify_oxford_datasets"
  # Tiny ASCII PCD + PNG under OUT to exercise Open/Read (not committed).
  SMOKE="$OUT/oxford_smoke"
  rm -rf "$SMOKE"
  mkdir -p "$SMOKE/spires/raw/lidar-clouds" "$SMOKE/spires/raw/cam0" \
           "$SMOKE/ncd/ouster_points" "$SMOKE/ncd/cam0"
  TS=1710338097943291736
  {
    echo "# .PCD v0.7 - Point Cloud Data file format"
    echo "VERSION 0.7"
    echo "FIELDS x y z intensity"
    echo "SIZE 4 4 4 4"
    echo "TYPE F F F F"
    echo "COUNT 1 1 1 1"
    echo "WIDTH 120"
    echo "HEIGHT 1"
    echo "POINTS 120"
    echo "DATA ascii"
    for i in $(seq 0 119); do
      echo "$i.0 0.1 0.2 1.0"
    done
  } > "$SMOKE/spires/raw/lidar-clouds/${TS}.pcd"
  cp "$SMOKE/spires/raw/lidar-clouds/${TS}.pcd" "$SMOKE/ncd/ouster_points/${TS}.pcd"
  # Minimal valid 1x1 PNG.
  echo 'iVBORw0KGgoAAAANSUhEUgAAAAEAAAABCAYAAAAfFcSJAAAADUlEQVR42mP8z8BQDwAEhQGAhKmMIQAAAABJRU5ErkJggg==' \
    | base64 -d > "$SMOKE/spires/raw/cam0/${TS}.png"
  cp "$SMOKE/spires/raw/cam0/${TS}.png" "$SMOKE/ncd/cam0/${TS}.png"
  "$OUT/verify_oxford_datasets" --spires "$SMOKE/spires" --max-frames 1
  "$OUT/verify_oxford_datasets" --ncd "$SMOKE/ncd" --max-frames 1
  if [[ -n "${ARGUS_SPIRES_ROOT:-}" && -d "${ARGUS_SPIRES_ROOT}" ]]; then
    "$OUT/verify_oxford_datasets" --spires "$ARGUS_SPIRES_ROOT" --max-frames 2
  fi
  if [[ -n "${ARGUS_NCD_ROOT:-}" && -d "${ARGUS_NCD_ROOT}" ]]; then
    "$OUT/verify_oxford_datasets" --ncd "$ARGUS_NCD_ROOT" --max-frames 2
  fi

  echo "==> test_camera (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" \
    "$SRC/argus/common/camera/pinhole_camera.cpp" \
    "$SRC/argus/common/camera/equidistant_camera.cpp" \
    "$SRC/argus/common/camera/atan_camera.cpp" \
    "$SRC/argus/common/camera/polynomial_camera.cpp" \
    "$SRC/argus/common/camera/omni_camera.cpp" \
    "$SRC/argus/common/camera/camera_loader.cpp" \
    "$SRC/test/test_camera.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -l"$OCV_CALIB_LIB" \
    "${OCV_EXTRA_LIBS[@]}" \
    -o "$OUT/test_camera"
  "$OUT/test_camera"

  echo "==> test_vio_manager (OpenCV=$OCV_INC)"
  c++ "${CXXFLAGS[@]}" -I"$OCV_INC" -I"$STUB_INC" \
    "$SRC/argus/common/camera/pinhole_camera.cpp" \
    "$SRC/argus/map/visual_point.cpp" \
    "$SRC/argus/map/voxel_points.cpp" \
    "$SRC/argus/map/sub_sparse_map.cpp" \
    "$SRC/argus/map/visual_map.cpp" \
    "${VOXEL_SRCS[@]}" \
    "$SRC/argus/odometry/frame.cpp" \
    "$SRC/argus/odometry/vio_manager.cpp" \
    "$SRC/test/test_vio_manager.cpp" \
    "${LDFLAGS[@]}" "${OCV_LDFLAGS[@]}" \
    -lopencv_core -lopencv_imgproc -lopencv_imgcodecs -l"$OCV_CALIB_LIB" \
    "${OCV_EXTRA_LIBS[@]}" \
    -o "$OUT/test_vio_manager"
  "$OUT/test_vio_manager"
else
  echo "==> skip OpenCV tests (headers not found)"
fi

echo "OK argus stub tests"
