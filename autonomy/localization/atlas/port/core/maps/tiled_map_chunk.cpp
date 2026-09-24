//
// Created by xiang on 23-2-7.
//

#include "tiled_map_chunk.hpp"

#include <pcl/io/pcd_io.h>

namespace atlas_lio {

void MapChunk::AddPoint(const PointType& pt) {
    if (cloud_ == nullptr) {
        cloud_.reset(new PointCloudType);
        cloud_->reserve(50000);
    }
    cloud_->points.emplace_back(pt);
}

void MapChunk::LoadCloud() {
    if (cloud_ == nullptr) {
        cloud_.reset(new PointCloudType);
    }
    pcl::io::loadPCDFile(filename_, *cloud_);

    loaded_ = true;
}

void MapChunk::Unload() {
    cloud_ = nullptr;
    loaded_ = false;
}

}  // namespace atlas_lio