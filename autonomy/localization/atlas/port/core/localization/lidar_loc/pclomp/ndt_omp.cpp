#include "ndt_omp.hpp"
#include "ndt_omp_impl.hpp"

template class atlas_pclomp::NormalDistributionsTransform<pcl::PointXYZ, pcl::PointXYZ>;
template class atlas_pclomp::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI>;
