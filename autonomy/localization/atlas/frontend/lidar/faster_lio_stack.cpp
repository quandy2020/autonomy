/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/localization/atlas/frontend/lidar/faster_lio_stack.hpp"

#include <cmath>
#include <exception>
#include <filesystem>
#include <fstream>

#include "glog/logging.h"
#include "opencv2/imgcodecs.hpp"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include "yaml-cpp/yaml.h"

#include "autonomy/localization/atlas/io/point_cloud_io.hpp"
#include "autonomy/localization/atlas/port/core/g2p5/g2p5.hpp"
#include "autonomy/localization/atlas/port/core/lio/laser_mapping.hpp"
#include "autonomy/localization/atlas/port/core/localization/localization.hpp"
#include "autonomy/localization/atlas/port/core/loop_closing/loop_closing.hpp"
#include "autonomy/localization/atlas/port/core/maps/tiled_map.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

SE3 ToAtlas(const atlas_lio::SE3& pose) {
    SE3 out = SE3::Identity();
    out.linear() = pose.rotationMatrix();
    out.translation() = pose.translation();
    return out;
}

// Newest keyframe that the pose graph has already moved. A keyframe created
// after the last loop still has opt == lio, so it must not wipe the correction.
atlas_lio::SE3 LoopAlignment(
    const std::vector<atlas_lio::Keyframe::Ptr>& keyframes,
    const atlas_lio::Keyframe::Ptr& skip) {
    for (auto it = keyframes.rbegin(); it != keyframes.rend(); ++it) {
        const atlas_lio::Keyframe::Ptr& kf = *it;
        if (!kf || kf == skip) {
            continue;
        }
        const atlas_lio::SE3 delta =
            kf->GetOptPose() * kf->GetLIOPose().inverse();
        if (delta.log().norm() > 1e-6) {
            return delta;
        }
    }
    return atlas_lio::SE3();
}

atlas_lio::CloudPtr ToLioCloud(const PointCloud2& msg) {
    PointCloud decoded;
    if (!DecodePointCloud2(msg, &decoded) || decoded.empty()) {
        return nullptr;
    }
    auto cloud = std::make_shared<atlas_lio::PointCloudType>();
    cloud->reserve(decoded.size());
    const double stamp = HeaderStampSec(msg.header());
    cloud->header.stamp = static_cast<std::uint64_t>(stamp * 1e9);
    for (const PointXYZI& src : decoded) {
        atlas_lio::PointType point;
        point.x = src.x;
        point.y = src.y;
        point.z = src.z;
        point.intensity = src.intensity;
        point.time = (src.timestamp - stamp) * 1000.0;
        cloud->push_back(point);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = false;
    return cloud;
}

sensor::imu::Measurement FromProtoImu(const Imu& msg) {
    sensor::imu::Measurement sample;
    sample.timestamp = HeaderStampSec(msg.header());
    sample.acceleration = Vec3(msg.linear_acceleration().x(),
                               msg.linear_acceleration().y(),
                               msg.linear_acceleration().z());
    sample.angular_velocity = Vec3(msg.angular_velocity().x(),
                                   msg.angular_velocity().y(),
                                   msg.angular_velocity().z());
    return sample;
}

atlas_lio::SE3 ToLio(const SE3& pose) {
    Eigen::Quaterniond rotation(pose.linear());
    rotation.normalize();
    return atlas_lio::SE3(rotation, pose.translation());
}

void EncodeLioCloud(const atlas_lio::CloudPtr& map, PointCloud2* cloud) {
    if (!map || cloud == nullptr) {
        return;
    }
    PointCloud points;
    points.reserve(map->size());
    for (const atlas_lio::PointType& src : map->points) {
        PointXYZI point;
        point.x = src.x;
        point.y = src.y;
        point.z = src.z;
        point.intensity = src.intensity;
        points.push_back(point);
    }
    EncodePointCloud2(points, cloud, "map");
}

atlas_lio::IMUPtr ToLioImu(const sensor::imu::Measurement& measurement) {
    auto imu = std::make_shared<atlas_lio::IMU>();
    imu->timestamp = measurement.timestamp;
    imu->angular_velocity = measurement.angular_velocity;
    imu->linear_acceleration = measurement.acceleration;
    return imu;
}

}  // namespace

struct FasterLioStack::Impl {
    std::shared_ptr<atlas_lio::g2p5::G2P5> grid;
    std::shared_ptr<atlas_lio::LoopClosing> loop;
    std::shared_ptr<atlas_lio::LaserMapping> mapping;
    std::shared_ptr<atlas_lio::loc::Localization> loc;
    atlas_lio::Keyframe::Ptr last_kf;
    bool localization = false;
    float chunk_size = 100.f;
    SE3 visual_prior = SE3::Identity();
    double visual_weight = 0.0;
    bool has_visual_prior = false;
    double last_imu_time = -1.0;
    atlas_lio::SE3 map_from_lio;
    atlas_lio::CloudPtr static_map;
    automsgs::msgs::map_msgs::OccupancyGrid static_grid;
    bool has_static_grid = false;
};

void FasterLioStack::LoadSavedProducts(const std::string& directory) {
    if (!impl_ || directory.empty()) {
        return;
    }
    Impl* impl = impl_.get();
    const std::string pcd_path = directory + "/global.pcd";
    if (std::filesystem::exists(pcd_path)) {
        auto cloud = std::make_shared<atlas_lio::PointCloudType>();
        if (pcl::io::loadPCDFile(pcd_path, *cloud) == 0 && !cloud->empty()) {
            impl->static_map = std::move(cloud);
        }
    }
    const std::string image_path = directory + "/map.pgm";
    const std::string yaml_path = directory + "/map.yaml";
    if (!std::filesystem::exists(image_path) ||
        !std::filesystem::exists(yaml_path)) {
        return;
    }
    const cv::Mat image = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    if (image.empty()) {
        return;
    }
    double resolution = 0.1;
    double origin_x = 0.0;
    double origin_y = 0.0;
    try {
        const YAML::Node yaml = YAML::LoadFile(yaml_path);
        if (yaml["resolution"]) {
            resolution = yaml["resolution"].as<double>();
        }
        if (yaml["origin"] && yaml["origin"].IsSequence() &&
            yaml["origin"].size() >= 2) {
            origin_x = yaml["origin"][0].as<double>();
            origin_y = yaml["origin"][1].as<double>();
        }
    } catch (const std::exception&) {
        return;
    }
    const int width = image.cols;
    const int height = image.rows;
    automsgs::msgs::map_msgs::OccupancyGrid grid;
    auto* info = grid.mutable_info();
    info->set_resolution(static_cast<float>(resolution));
    info->set_width(static_cast<uint32_t>(width));
    info->set_height(static_cast<uint32_t>(height));
    info->mutable_origin()->mutable_position()->set_x(origin_x);
    info->mutable_origin()->mutable_position()->set_y(origin_y);
    info->mutable_origin()->mutable_orientation()->set_w(1.0);
    grid.mutable_data()->Resize(width * height, -1);
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            const uchar value = image.at<uchar>(height - 1 - y, x);
            int cell = -1;
            if (value >= 250) {
                cell = 0;
            } else if (value <= 5) {
                cell = 100;
            }
            (*grid.mutable_data())[y * width + x] = cell;
        }
    }
    impl->static_grid = std::move(grid);
    impl->has_static_grid = true;
}

FasterLioStack::FasterLioStack() : impl_(std::make_unique<Impl>()) {}

FasterLioStack::~FasterLioStack() = default;

bool FasterLioStack::Init(const AtlasConfig& config) {
    active_ = false;
    if (config.config_path.empty() || !impl_) {
        return false;
    }
    try {
        const bool want_localization =
            config.mission != Mission::kMapping &&
            !config.lidar_map_directory.empty();
        if (want_localization) {
            impl_->loc = std::make_shared<atlas_lio::loc::Localization>();
            if (!impl_->loc->Init(config.config_path,
                                  config.lidar_map_directory)) {
                impl_->loc.reset();
                return false;
            }
            LoadSavedProducts(config.lidar_map_directory);
            impl_->localization = true;
            active_ = true;
            return true;
        }

        impl_->mapping = std::make_shared<atlas_lio::LaserMapping>();
        if (!impl_->mapping->Init(config.config_path)) {
            impl_->mapping.reset();
            return false;
        }

        bool with_grid = true;
        bool with_loop = true;
        const YAML::Node system = YAML::LoadFile(config.config_path)["system"];
        if (system) {
            with_grid = system["with_g2p5"].as<bool>(true);
            with_loop = system["with_loop_closing"].as<bool>(true);
        }
        if (with_grid) {
            atlas_lio::g2p5::G2P5::Options grid_options;
            grid_options.online_mode_ = true;
            impl_->grid = std::make_shared<atlas_lio::g2p5::G2P5>(grid_options);
            impl_->grid->Init(config.config_path);
        }
        if (with_loop) {
            atlas_lio::LoopClosing::Options loop_options;
            loop_options.online_mode_ = true;
            impl_->loop = std::make_shared<atlas_lio::LoopClosing>(loop_options);
            impl_->loop->SetLoopClosedCB([grid = impl_->grid]() {
                if (grid) {
                    grid->RedrawGlobalMap();
                }
            });
            impl_->loop->Init(config.config_path);
        }
        impl_->chunk_size =
            static_cast<float>(std::max(1.0, config.lidar_chunk_size));
        impl_->localization = false;
        active_ = true;
        return true;
    } catch (const std::exception& error) {
        LOG(ERROR) << "FasterLioStack init failed: " << error.what();
        impl_->loop.reset();
        impl_->grid.reset();
        impl_->mapping.reset();
        impl_->loc.reset();
        return false;
    }
}

void FasterLioStack::ProcessImu(const sensor::imu::Measurement& measurement) {
    if (!active_ || !impl_) {
        return;
    }
    if (impl_->last_imu_time >= 0.0 &&
        measurement.timestamp <= impl_->last_imu_time) {
        return;
    }
    impl_->last_imu_time = measurement.timestamp;
    const atlas_lio::IMUPtr imu = ToLioImu(measurement);
    if (impl_->loc) {
        impl_->loc->ProcessIMUMsg(imu);
        return;
    }
    if (impl_->mapping) {
        impl_->mapping->ProcessIMU(imu);
    }
}

bool FasterLioStack::ProcessCloud(const SensorData& data, SE3* T_wb) {
    if (!active_ || !impl_ || T_wb == nullptr || !data.has_lidar) {
        return false;
    }
    if (data.has_imu) {
        for (const Imu& sample : data.imu) {
            ProcessImu(FromProtoImu(sample));
        }
    }
    const atlas_lio::CloudPtr cloud = ToLioCloud(data.lidar);
    if (!cloud) {
        return false;
    }
    if (impl_->loc) {
        impl_->loc->ProcessLidarMsg(cloud);
        const atlas_lio::loc::LocalizationResult& result = impl_->loc->result();
        if (!result.valid_) {
            return false;
        }
        *T_wb = ToAtlas(result.pose_);
        return true;
    }
    if (!impl_->mapping) {
        return false;
    }
    if (impl_->has_visual_prior) {
        impl_->mapping->ApplyPosePrior(ToLio(impl_->visual_prior),
                                       impl_->visual_weight);
        impl_->has_visual_prior = false;
    }
    impl_->mapping->ProcessPointCloud2(cloud);
    if (!impl_->mapping->Run()) {
        return false;
    }
    const atlas_lio::NavState state = impl_->mapping->GetState();
    if (!state.pose_is_ok_) {
        return false;
    }
    atlas_lio::SE3 T_map_imu = state.GetPose();
    const atlas_lio::Keyframe::Ptr kf = impl_->mapping->GetKeyframe();
    const bool is_new = kf && kf != impl_->last_kf;
    if (is_new) {
        impl_->map_from_lio =
            LoopAlignment(impl_->mapping->GetAllKeyframes(), kf);
        kf->SetOptPose(impl_->map_from_lio * kf->GetLIOPose());
        impl_->last_kf = kf;
        if (impl_->loop) {
            impl_->loop->AddKF(kf);
        }
        if (impl_->grid) {
            impl_->grid->PushKeyframe(kf);
        }
    } else if (kf) {
        impl_->map_from_lio = kf->GetOptPose() * kf->GetLIOPose().inverse();
    }
    T_map_imu = impl_->map_from_lio * T_map_imu;
    *T_wb = ToAtlas(T_map_imu);
    return true;
}

bool FasterLioStack::FillDenseCloud(PointCloud2* cloud) const {
    if (!active_ || !impl_ || cloud == nullptr) {
        return false;
    }
    if (!impl_->mapping) {
        if (!impl_->static_map) {
            return false;
        }
        EncodeLioCloud(impl_->static_map, cloud);
        return cloud->width() > 0;
    }
    const atlas_lio::CloudPtr map =
        impl_->mapping->GetGlobalMap(/*use_lio_pose=*/false, true, -1.f);
    if (!map || map->empty()) {
        return false;
    }
    EncodeLioCloud(map, cloud);
    return true;
}

bool FasterLioStack::FillRegisteredCloud(PointCloud2* cloud) const {
    if (!active_ || !impl_ || cloud == nullptr) {
        return false;
    }
    atlas_lio::CloudPtr scan;
    atlas_lio::SE3 align;
    if (impl_->mapping) {
        scan = impl_->mapping->GetScanWorld();
        align = impl_->map_from_lio;
    } else if (impl_->loc) {
        scan = impl_->loc->LatestScanWorld();
        align = atlas_lio::SE3();
    }
    if (!scan || scan->empty()) {
        return false;
    }
    auto aligned = std::make_shared<atlas_lio::PointCloudType>();
    pcl::transformPointCloud(*scan, *aligned, align.matrix());
    EncodeLioCloud(aligned, cloud);
    return cloud->width() > 0;
}

bool FasterLioStack::FillOccupancyGrid(
    automsgs::msgs::map_msgs::OccupancyGrid* grid) const {
    if (!active_ || !impl_ || grid == nullptr) {
        return false;
    }
    if (impl_->grid) {
        const auto map = impl_->grid->GetNewestMap();
        if (map) {
            *grid = map->ToROS();
            return true;
        }
    }
    if (!impl_->has_static_grid) {
        return false;
    }
    *grid = impl_->static_grid;
    return true;
}

bool FasterLioStack::FillLidarConstraints(LidarConstraintGraph* graph) const {
    if (!active_ || !impl_ || graph == nullptr || !impl_->loop) {
        return false;
    }
    const auto viz = impl_->loop->GetConstraintViz();
    auto Copy = [](const std::vector<atlas_lio::LoopClosing::LoopEdgeViz>& src,
                   std::vector<LidarConstraintEdge>* dst) {
        dst->clear();
        dst->reserve(src.size());
        for (const auto& edge : src) {
            LidarConstraintEdge out;
            out.x0 = edge.p1.x();
            out.y0 = edge.p1.y();
            out.z0 = edge.p1.z();
            out.x1 = edge.p2.x();
            out.y1 = edge.p2.y();
            out.z1 = edge.p2.z();
            dst->push_back(out);
        }
    };
    Copy(viz.odom, &graph->odom);
    Copy(viz.loops, &graph->loops);
    Copy(viz.reloc, &graph->reloc);
    return !graph->odom.empty() || !graph->loops.empty() ||
           !graph->reloc.empty();
}

bool FasterLioStack::SaveMap(const std::string& directory) const {
    if (!active_ || !impl_ || !impl_->mapping || directory.empty()) {
        return false;
    }
    const std::vector<atlas_lio::Keyframe::Ptr> keyframes =
        impl_->mapping->GetAllKeyframes();
    if (keyframes.empty() || !keyframes.front()) {
        return false;
    }
    const atlas_lio::CloudPtr global =
        impl_->mapping->GetGlobalMap(/*use_lio_pose=*/false, true, -1.f);
    if (!global || global->empty()) {
        return false;
    }
    std::error_code ec;
    std::filesystem::remove_all(directory, ec);
    std::filesystem::create_directories(directory, ec);
    if (ec) {
        LOG(ERROR) << "cannot create map directory " << directory;
        return false;
    }

    atlas_lio::TiledMap::Options options;
    options.map_path_ = directory;
    options.chunk_size_ = impl_->chunk_size;
    options.inv_chunk_size_ = 1.f / impl_->chunk_size;
    atlas_lio::TiledMap tiles(options);
    if (!tiles.ConvertFromFullPCD(global, keyframes.front()->GetOptPose(),
                                  directory)) {
        return false;
    }
    pcl::io::savePCDFileBinaryCompressed(directory + "/global.pcd", *global);

    if (impl_->grid) {
        const auto newest = impl_->grid->GetNewestMap();
        if (newest) {
            const auto occupancy = newest->ToROS();
            const int width = static_cast<int>(occupancy.info().width());
            const int height = static_cast<int>(occupancy.info().height());
            if (width > 0 && height > 0 &&
                occupancy.data_size() >= width * height) {
                cv::Mat image(height, width, CV_8UC1);
                for (int y = 0; y < height; ++y) {
                    for (int x = 0; x < width; ++x) {
                        const int8_t cell = occupancy.data(y * width + x);
                        uchar value = 128;
                        if (cell == 0) {
                            value = 255;
                        } else if (cell == 100) {
                            value = 0;
                        }
                        image.at<uchar>(height - 1 - y, x) = value;
                    }
                }
                cv::imwrite(directory + "/map.pgm", image);
                YAML::Emitter emitter;
                emitter << YAML::BeginMap;
                emitter << YAML::Key << "image" << YAML::Value << "map.pgm";
                emitter << YAML::Key << "mode" << YAML::Value << "trinary";
                emitter << YAML::Key << "resolution" << YAML::Value
                        << occupancy.info().resolution();
                emitter << YAML::Key << "origin" << YAML::Value
                        << YAML::Flow << YAML::BeginSeq
                        << occupancy.info().origin().position().x()
                        << occupancy.info().origin().position().y() << 0.0
                        << YAML::EndSeq;
                emitter << YAML::Key << "negate" << YAML::Value << 0;
                emitter << YAML::Key << "occupied_thresh" << YAML::Value
                        << 0.65;
                emitter << YAML::Key << "free_thresh" << YAML::Value << 0.25;
                emitter << YAML::EndMap;
                std::ofstream yaml(directory + "/map.yaml");
                yaml << emitter.c_str();
            }
        }
    }
    LOG(INFO) << "tiled lidar map saved to " << directory;
    return true;
}

bool FasterLioStack::localizing() const {
    return active_ && impl_ && impl_->localization;
}

void FasterLioStack::SetVisualPrior(const SE3& T_wb, double weight) {
    if (!impl_ || !(weight > 0.0)) {
        return;
    }
    impl_->visual_prior = T_wb;
    impl_->visual_weight = weight;
    impl_->has_visual_prior = true;
}

void FasterLioStack::SetOccupancyCallback(
    std::function<void(const automsgs::msgs::map_msgs::OccupancyGrid&)>
        callback) {
    if (!impl_ || !impl_->grid || !callback) {
        return;
    }
    impl_->grid->SetMapUpdateCallback(
        [callback](atlas_lio::g2p5::G2P5MapPtr map) {
            if (map) {
                callback(map->ToROS());
            }
        });
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
