#include "pointcloud_preprocess.hpp"

#include <glog/logging.h>

namespace lightning {

void PointCloudPreprocess::Set(LidarType lid_type, double bld, int pfilt_num) {
    lidar_type_ = lid_type;
    blind_ = bld;
    point_filter_num_ = pfilt_num;
}

void PointCloudPreprocess::Process(const CloudPtr& cloud_in, PointCloudType::Ptr& pcl_out) {
    cloud_out_.clear();
    if (!cloud_in || cloud_in->empty()) {
        *pcl_out = cloud_out_;
        return;
    }

    cloud_out_.reserve(cloud_in->size());
    const double blind2 = blind_ * blind_;
    for (std::size_t i = 0; i < cloud_in->size(); ++i) {
        if (point_filter_num_ > 1 && (i % static_cast<std::size_t>(point_filter_num_)) != 0) {
            continue;
        }
        const auto& pt = cloud_in->points[i];
        const double range2 = static_cast<double>(pt.x) * pt.x + static_cast<double>(pt.y) * pt.y +
                              static_cast<double>(pt.z) * pt.z;
        if (range2 < blind2) {
            continue;
        }
        if (pt.z < height_min_ || pt.z > height_max_) {
            continue;
        }
        cloud_out_.push_back(pt);
    }
    cloud_out_.width = static_cast<uint32_t>(cloud_out_.size());
    cloud_out_.height = 1;
    cloud_out_.is_dense = false;
    cloud_out_.header = cloud_in->header;
    *pcl_out = cloud_out_;
}

}  // namespace lightning
