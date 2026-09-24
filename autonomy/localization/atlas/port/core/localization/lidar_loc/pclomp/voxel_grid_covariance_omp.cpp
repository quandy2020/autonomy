#include "voxel_grid_covariance_omp.hpp"
#include "voxel_grid_covariance_omp_impl.hpp"

template class atlas_pclomp::VoxelGridCovariance<pcl::PointXYZ>;
template class atlas_pclomp::VoxelGridCovariance<pcl::PointXYZI>;
