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

#include "argus/mapping/map_builder.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <opencv2/imgcodecs.hpp>

namespace argus {
namespace mapping {
namespace {

namespace fs = std::filesystem;

map::IkdTreeConfig MakeIkdConfig(const MapBuilderConfig& cfg) {
    map::IkdTreeConfig ikd_cfg;
    ikd_cfg.downsample_resolution = cfg.ikd_downsample_resolution;
    return ikd_cfg;
}

}  // namespace

MapBuilder::MapBuilder(MapBuilderConfig config)
    : config_(std::move(config)), ikd_(MakeIkdConfig(config_)) {
    Reset(config_.session_id);
}

void MapBuilder::Reset(const std::string& session_id) {
    config_.session_id = session_id.empty() ? "map" : session_id;
    session_.Clear();
    map::SessionMeta meta;
    meta.session_id = config_.session_id;
    meta.node_id_base = config_.node_id_base;
    session_.Register(meta);
    keyframes_.clear();
    ikd_.Clear();
    visual_.Clear();
    package_meta_ = MapPackageMeta{};
    package_meta_.session_id = config_.session_id;
    package_meta_.prior_resolution = config_.prior_resolution;
    package_meta_.ikd_downsample_resolution = config_.ikd_downsample_resolution;
    occupancy_.reset();
    if (config_.build_occupancy_grid) {
        occupancy_ = std::make_unique<map::OccupancyGridBuilder>(
            config_.occupancy_grid);
    }
}

map::OccupancyGridMap::SharedPtr MapBuilder::occupancy_grid() const {
    if (occupancy_ == nullptr) {
        return nullptr;
    }
    return occupancy_->map();
}

void MapBuilder::AddVisualLandmark(const Eigen::Vector3d& pos,
                                   const Eigen::Vector3d& normal) {
    auto* vp = new map::VisualPoint(pos);
    vp->normal_ = normal;
    if (normal.squaredNorm() > 1e-12) {
        vp->is_normal_initialized_ = true;
    }
    visual_.InsertPoint(vp);
}

void MapBuilder::ImportVisualLandmarks(const map::VisualMap& src) {
    visual_.Clear();
    for (const auto& pair : src.feat_map()) {
        if (pair.second == nullptr) {
            continue;
        }
        for (const map::VisualPoint* vp : pair.second->voxel_points) {
            if (vp == nullptr) {
                continue;
            }
            AddVisualLandmark(vp->pos_, vp->normal_);
        }
    }
}

void MapBuilder::AddKeyFrame(const MapBuilderKeyFrame& kf) {
    keyframes_.push_back(kf);

    map::SessionKeyFrame sk;
    sk.local_id = kf.local_id;
    sk.stamp_sec = kf.stamp_sec;
    const map::SessionMeta* meta = session_.Get(config_.session_id);
    if (meta != nullptr) {
        sk.T_session_b = meta->T_world_session.inverse() * kf.T_w_b;
    } else {
        sk.T_session_b = kf.T_w_b;
    }
    auto traj = session_.Trajectory(config_.session_id);
    traj.push_back(sk);
    session_.SetTrajectory(config_.session_id, std::move(traj));

    if (meta != nullptr) {
        map::SessionMeta* m = session_.GetMutable(config_.session_id);
        if (m != nullptr) {
            if (traj.size() == 1) {
                m->t_start_sec = kf.stamp_sec;
            }
            m->t_end_sec = kf.stamp_sec;
        }
    }

    sensor::PointCloudXYZI world;
    world.reserve(kf.cloud.size());
    for (std::size_t i = 0; i < kf.cloud.size(); ++i) {
        Eigen::Vector3d pb(kf.cloud[i].x, kf.cloud[i].y, kf.cloud[i].z);
        const Eigen::Vector3d pw = kf.cloud_in_world ? pb : (kf.T_w_b * pb);
        sensor::PointType p;
        p.x = static_cast<float>(pw.x());
        p.y = static_cast<float>(pw.y());
        p.z = static_cast<float>(pw.z());
        p.intensity = kf.cloud[i].intensity;
        world.push_back(p);
    }
    ikd_.AddPoints(world);

    if (occupancy_ != nullptr && !kf.cloud.empty()) {
        occupancy_->AddScan(kf.T_w_b, kf.cloud, kf.cloud_in_world);
    }
}

bool MapBuilder::SaveOccupancyPng(const std::string& path) const {
    if (occupancy_ == nullptr || occupancy_->map() == nullptr ||
        occupancy_->map()->empty()) {
        return true;  // optional
    }
    const cv::Mat img = occupancy_->map()->ToCv();
    if (img.empty()) {
        return true;
    }
    return cv::imwrite(path, img);
}

sensor::PointCloudXYZI MapBuilder::CollectMapCloud() const {
    std::vector<Eigen::Vector3d> pts;
    ikd_.ExportPoints(&pts);
    sensor::PointCloudXYZI cloud;
    cloud.reserve(pts.size());
    const std::size_t stride =
        (config_.max_export_points > 0 &&
         pts.size() > config_.max_export_points)
            ? (pts.size() / config_.max_export_points + 1)
            : 1;
    for (std::size_t i = 0; i < pts.size(); i += stride) {
        sensor::PointType p;
        p.x = static_cast<float>(pts[i].x());
        p.y = static_cast<float>(pts[i].y());
        p.z = static_cast<float>(pts[i].z());
        cloud.push_back(p);
        if (config_.max_export_points > 0 &&
            cloud.size() >= config_.max_export_points) {
            break;
        }
    }
    return cloud;
}

bool MapBuilder::SaveCloudXyz(const std::string& path) const {
    std::ostringstream os;
    const sensor::PointCloudXYZI cloud = CollectMapCloud();
    for (std::size_t i = 0; i < cloud.size(); ++i) {
        os << cloud[i].x << ' ' << cloud[i].y << ' ' << cloud[i].z << '\n';
    }
    return AtomicWriteFile(path, os.str());
}

bool MapBuilder::SavePriorXyz(const std::string& path) const {
    // Occupancy prior cloud = downsampled map points (same as map.xyz export).
    return SaveCloudXyz(path);
}

bool MapBuilder::SaveTrajectoryTum(const std::string& path) const {
    std::ostringstream os;
    os.setf(std::ios::fixed, std::ios::floatfield);
    os << std::setprecision(9);
    for (const MapBuilderKeyFrame& kf : keyframes_) {
        const Eigen::Vector3d& t = kf.T_w_b.translation();
        const Eigen::Quaterniond& q = kf.T_w_b.rotation();
        os << kf.stamp_sec << ' ' << t.x() << ' ' << t.y() << ' ' << t.z()
           << ' ' << q.x() << ' ' << q.y() << ' ' << q.z() << ' ' << q.w()
           << '\n';
    }
    return AtomicWriteFile(path, os.str());
}

bool MapBuilder::SaveKeyframeClouds(const std::string& directory) const {
    std::error_code ec;
    fs::create_directories(directory, ec);
    std::ostringstream index;
    for (const MapBuilderKeyFrame& kf : keyframes_) {
        if (kf.cloud.empty()) {
            continue;
        }
        char stem[32];
        std::snprintf(stem, sizeof(stem), "%06d", kf.local_id);
        const fs::path bin = fs::path(directory) / (std::string(stem) + ".bin");
        std::ofstream out(bin, std::ios::binary);
        if (!out) {
            return false;
        }
        for (std::size_t i = 0; i < kf.cloud.size(); ++i) {
            const float xyz[4] = {kf.cloud[i].x, kf.cloud[i].y, kf.cloud[i].z,
                                  kf.cloud[i].intensity};
            out.write(reinterpret_cast<const char*>(xyz), sizeof(xyz));
        }
        index << kf.local_id << ' ' << stem << ".bin " << kf.cloud.size() << ' '
              << (kf.cloud_in_world ? 1 : 0) << '\n';
    }
    return AtomicWriteFile((fs::path(directory) / "index.txt").string(),
                           index.str());
}

bool MapBuilder::LoadKeyframeClouds(const std::string& directory) {
    const fs::path idx = fs::path(directory) / "index.txt";
    if (!fs::exists(idx)) {
        return true;  // optional (v3 packages without SC clouds)
    }
    std::ifstream in(idx.string());
    if (!in) {
        return false;
    }
    std::unordered_map<int, sensor::PointCloudXYZI> by_id;
    std::unordered_map<int, bool> world_flag;
    int id = 0;
    std::string name;
    std::size_t npts = 0;
    int in_world = 0;
    while (in >> id >> name >> npts >> in_world) {
        const fs::path bin = fs::path(directory) / name;
        std::ifstream bins(bin, std::ios::binary);
        if (!bins) {
            continue;
        }
        sensor::PointCloudXYZI cloud;
        cloud.reserve(npts);
        for (std::size_t i = 0; i < npts; ++i) {
            float xyz[4];
            bins.read(reinterpret_cast<char*>(xyz), sizeof(xyz));
            if (!bins) {
                break;
            }
            sensor::PointType p;
            p.x = xyz[0];
            p.y = xyz[1];
            p.z = xyz[2];
            p.intensity = xyz[3];
            cloud.push_back(p);
        }
        by_id[id] = std::move(cloud);
        world_flag[id] = in_world != 0;
    }
    for (MapBuilderKeyFrame& kf : keyframes_) {
        auto it = by_id.find(kf.local_id);
        if (it == by_id.end()) {
            continue;
        }
        kf.cloud = std::move(it->second);
        kf.cloud_in_world = world_flag[kf.local_id];
    }
    return true;
}

bool MapBuilder::LoadCloudXyz(const std::string& path) {
    std::ifstream in(path);
    if (!in) {
        return false;
    }
    sensor::PointCloudXYZI cloud;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#') {
            continue;
        }
        double x = 0;
        double y = 0;
        double z = 0;
        std::istringstream ss(line);
        if (!(ss >> x >> y >> z)) {
            continue;
        }
        sensor::PointType p;
        p.x = static_cast<float>(x);
        p.y = static_cast<float>(y);
        p.z = static_cast<float>(z);
        cloud.push_back(p);
    }
    if (cloud.empty()) {
        return false;
    }
    ikd_.Build(cloud);
    return true;
}

bool MapBuilder::Save(const std::string& directory) const {
    std::error_code ec;
    fs::create_directories(directory, ec);
    const fs::path dir(directory);
    if (!session_.Save((dir / "session.txt").string())) {
        return false;
    }
    if (!SaveCloudXyz((dir / "map.xyz").string())) {
        return false;
    }
    if (!SavePriorXyz((dir / "prior.xyz").string())) {
        return false;
    }
    if (!SaveTrajectoryTum((dir / "trajectory.tum").string())) {
        return false;
    }
    if (!visual_.SaveLandmarks((dir / "visual.xyz").string())) {
        return false;
    }
    if (!SaveKeyframeClouds((dir / "keyframes").string())) {
        return false;
    }
    if (config_.build_occupancy_grid) {
        if (!SaveOccupancyPng((dir / "occupancy.png").string())) {
            return false;
        }
    }

    package_meta_.version = kMapPackageVersion;
    package_meta_.session_id = config_.session_id;
    package_meta_.prior_resolution = config_.prior_resolution;
    package_meta_.ikd_downsample_resolution = config_.ikd_downsample_resolution;
    package_meta_.map_points = ikd_.size();
    package_meta_.visual_landmarks = visual_.LandmarkCount();
    package_meta_.keyframes = keyframes_.size();
    package_meta_.map_xyz_fnv64 = FileFnv64((dir / "map.xyz").string());
    return SaveMapPackageMeta((dir / "map_meta.txt").string(), package_meta_);
}

bool MapBuilder::Load(const std::string& directory) {
    const fs::path dir(directory);
    if (!IsMapPackageDirectory(directory)) {
        return false;
    }

    const fs::path meta_path = dir / "map_meta.txt";
    if (fs::exists(meta_path)) {
        if (!LoadMapPackageMeta(meta_path.string(), &package_meta_)) {
            return false;
        }
        std::string verr;
        if (!VerifyMapPackage(directory, package_meta_, &verr)) {
            return false;
        }
        if (package_meta_.prior_resolution > 1e-9) {
            config_.prior_resolution = package_meta_.prior_resolution;
        }
        if (package_meta_.ikd_downsample_resolution > 1e-9) {
            config_.ikd_downsample_resolution =
                package_meta_.ikd_downsample_resolution;
        }
    } else {
        package_meta_ = MapPackageMeta{};
        package_meta_.version = 1;
    }

    if (!session_.Load((dir / "session.txt").string())) {
        return false;
    }
    const auto ids = session_.SessionIds();
    if (!ids.empty()) {
        config_.session_id = ids.front();
        package_meta_.session_id = config_.session_id;
    }
    keyframes_.clear();
    for (const map::SessionKeyFrame& sk :
         session_.Trajectory(config_.session_id)) {
        MapBuilderKeyFrame kf;
        kf.local_id = sk.local_id;
        kf.stamp_sec = sk.stamp_sec;
        transform::Rigid3d Twb;
        if (session_.WorldPose(config_.session_id, sk.local_id, &Twb)) {
            kf.T_w_b = Twb;
        }
        keyframes_.push_back(kf);
    }
    if (!LoadCloudXyz((dir / "map.xyz").string())) {
        return false;
    }
    const fs::path visual_path = dir / "visual.xyz";
    if (fs::exists(visual_path)) {
        if (!visual_.LoadLandmarks(visual_path.string())) {
            return false;
        }
    } else {
        visual_.Clear();
    }
    if (!LoadKeyframeClouds((dir / "keyframes").string())) {
        return false;
    }
    package_meta_.map_points = ikd_.size();
    package_meta_.visual_landmarks = visual_.LandmarkCount();
    package_meta_.keyframes = keyframes_.size();
    return true;
}

std::vector<loop_closure::KeyFrame> MapBuilder::ExportLoopKeyFrames() const {
    const map::SessionMeta* meta = session_.Get(config_.session_id);
    const int base = (meta != nullptr) ? meta->node_id_base : config_.node_id_base;
    std::vector<loop_closure::KeyFrame> out;
    out.reserve(keyframes_.size());
    for (const MapBuilderKeyFrame& mk : keyframes_) {
        if (mk.cloud.empty()) {
            continue;
        }
        loop_closure::KeyFrame kf;
        kf.id = base + mk.local_id;
        kf.stamp_sec = mk.stamp_sec;
        kf.T_w_b = mk.T_w_b;
        kf.session_id = config_.session_id;
        kf.session_local_id = mk.local_id;
        if (mk.cloud_in_world) {
            // Prior LTA / SC expect body-frame clouds.
            const transform::Rigid3d T_b_w = mk.T_w_b.inverse();
            kf.cloud.reserve(mk.cloud.size());
            for (const auto& p : mk.cloud.points) {
                const Eigen::Vector3d pb =
                    T_b_w * Eigen::Vector3d(p.x, p.y, p.z);
                sensor::PointType q = p;
                q.x = static_cast<float>(pb.x());
                q.y = static_cast<float>(pb.y());
                q.z = static_cast<float>(pb.z());
                kf.cloud.push_back(q);
            }
        } else {
            kf.cloud = mk.cloud;
        }
        out.push_back(std::move(kf));
    }
    return out;
}

relocalization::PriorOccupancyMap::SharedPtr MapBuilder::ExportPriorMap()
    const {
    const sensor::PointCloudXYZI cloud = CollectMapCloud();
    if (cloud.empty()) {
        return nullptr;
    }
    return relocalization::PriorMapLoader::FromCloud(cloud,
                                                     config_.prior_resolution);
}

}  // namespace mapping
}  // namespace argus
