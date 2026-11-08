# apps/tools/dataset_converter — dataset format helpers

set(_ARGUS_KITTI_DATASET_SRCS
  ${PROJECT_SOURCE_DIR}/apps/dataset/kitti_reader.cpp)

argus_binary(argus_dataset_converter
  SRCS main.cpp ${_ARGUS_KITTI_DATASET_SRCS})

# Backward-compatible CLI used by docs / README.
argus_binary(slice_kitti
  SRCS ${PROJECT_SOURCE_DIR}/apps/runner/slice_kitti.cpp
       ${_ARGUS_KITTI_DATASET_SRCS})
