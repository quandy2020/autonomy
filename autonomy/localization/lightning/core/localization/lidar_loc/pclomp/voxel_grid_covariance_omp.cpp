#include "voxel_grid_covariance_omp.hpp"
#include "voxel_grid_covariance_omp_impl.hpp"

template class pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class pclomp::VoxelGridCovariance<pcl::PointXYZI>;
