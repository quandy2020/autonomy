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

/**
 * @file slam_system.cpp
 * @brief SlamSystem implementation: Init/Shutdown, Track*, Atlas IO, and backend start.
 */

#include "autonomy/localization/atlas/system/slam_system.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <thread>

#include "Eigen/Geometry"

#include "autonomy/localization/atlas/map/atlas_io.hpp"
#include "autonomy/localization/atlas/sensor/imu/pose.hpp"
#include "autonomy/localization/atlas/system/constants.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace {

bool IsPureMonocular(SlamSystem::Sensor sensor) {
    return sensor == SlamSystem::Sensor::kMonocular;
}

bool IsInertial(SlamSystem::Sensor sensor) {
    return sensor == SlamSystem::Sensor::kImuMonocular ||
           sensor == SlamSystem::Sensor::kImuStereo ||
           sensor == SlamSystem::Sensor::kImuRgbd;
}

Map* LargestMap(MultiMap* atlas, Map* fallback) {
    Map* best = fallback;
    std::size_t best_count = best != nullptr ? best->KeyFramesInMap() : 0;
    if (atlas == nullptr) {
        return best;
    }
    for (const auto& map : atlas->GetAllMaps()) {
        if (!map) {
            continue;
        }
        if (map->KeyFramesInMap() > best_count) {
            best = map.get();
            best_count = map->KeyFramesInMap();
        }
    }
    return best;
}

std::vector<std::shared_ptr<KeyFrame>> SortedKeyFrames(Map* map) {
    std::vector<std::shared_ptr<KeyFrame>> keyframes =
        map != nullptr ? map->GetAllKeyFrames()
                       : std::vector<std::shared_ptr<KeyFrame>>{};
    std::sort(keyframes.begin(), keyframes.end(),
              [](const auto& a, const auto& b) {
                  if (!a || !b) {
                      return static_cast<bool>(a);
                  }
                  return a->id < b->id;
              });
    return keyframes;
}

SE3 CameraPoseFromReference(const SE3& relative_cw,
                            std::shared_ptr<KeyFrame> reference,
                            const SE3& origin) {
    SE3 to_origin = SE3Identity();
    for (int guard = 0; reference && reference->isBad() && guard < 1000;
         ++guard) {
        to_origin = to_origin * reference->pose_to_parent;
        reference = reference->GetParent();
    }
    if (!reference) {
        return SE3Identity();
    }
    return relative_cw * (to_origin * reference->GetPose() * origin);
}

void WritePositionQuaternion(std::ostream& out, double stamp, const SE3& pose,
                             bool nanoseconds) {
    const Eigen::Quaterniond quaternion(pose.rotation());
    const Vec3 translation = pose.translation();
    out << std::setprecision(6);
    if (nanoseconds) {
        out << (stamp * 1e9);
    } else {
        out << stamp;
    }
    out << " " << std::setprecision(9) << translation.x() << " "
        << translation.y() << " " << translation.z() << " " << quaternion.x()
        << " " << quaternion.y() << " " << quaternion.z() << " "
        << quaternion.w() << "\n";
}

}  // namespace

bool SlamSystem::Init(const AtlasConfig& config, Sensor sensor) {
    sensor_ = sensor;
    tracking::Tracker::Options options;
    switch (sensor) {
        case Sensor::kStereo:
            options.sensor = tracking::Tracker::Sensor::kStereo;
            break;
        case Sensor::kImuStereo:
            options.sensor = tracking::Tracker::Sensor::kImuStereo;
            break;
        case Sensor::kMonocular:
            options.sensor = tracking::Tracker::Sensor::kMonocular;
            break;
        case Sensor::kImuMonocular:
            options.sensor = tracking::Tracker::Sensor::kImuMonocular;
            break;
        case Sensor::kImuRgbd:
            options.sensor = tracking::Tracker::Sensor::kImuRgbd;
            break;
        case Sensor::kRgbd:
        default:
            options.sensor = tracking::Tracker::Sensor::kRgbd;
            break;
    }
    ApplyConfigToTrackerOptions(config, &options.orb, &options.depth_map_factor,
                                &options.depth_threshold, &options.rgb);
    config_ = config;
    multi_map_ = std::make_shared<MultiMap>();
    tracker_.SetMultiMap(multi_map_);
    if (!tracker_.Init(config, options)) {
        return false;
    }
    if (!config.atlas_load_file.empty()) {
        LoadAtlas(config.atlas_load_file);
        tracker_.SetMultiMap(multi_map_);
    }
    if (!map_manager_.Init(config)) {
        return false;
    }

    // ≥2 workers: LocalMapping + LoopClosing long-lived Run loops.
    scheduler_ = std::make_unique<SlamScheduler>(/*num_threads=*/3,
                                                 /*max_tasks=*/256);

    if (tracker_.mutable_keyframe_database() != nullptr &&
        tracker_.mutable_map() != nullptr) {
        loop_closing_ = std::make_unique<backend::LoopClosing>(
            tracker_.mutable_map(), tracker_.mutable_keyframe_database(),
            multi_map_.get());
        const bool fix_scale =
            !(sensor == Sensor::kMonocular || sensor == Sensor::kImuMonocular);
        loop_closing_->set_fix_scale(fix_scale);
    }

    StartBackendWorkers();
    return true;
}

void SlamSystem::StartBackendWorkers() {
    if (tracker_.mutable_local_mapping() != nullptr) {
        tracker_.mutable_local_mapping()->SetLoopCloser(loop_closing_.get());
        tracker_.mutable_local_mapping()->Start(scheduler_.get());
        tracker_.set_async_mapping(true);
    }
    if (loop_closing_) {
        if (tracker_.mutable_local_mapping() != nullptr) {
            loop_closing_->SetLocalMapper(tracker_.mutable_local_mapping());
        }
        loop_closing_->Start(scheduler_.get());
    }
}

void SlamSystem::CreateNewMap() {
    if (tracker_.mutable_local_mapping() != nullptr) {
        tracker_.mutable_local_mapping()->RequestStop();
    }
    tracker_.CreateNewMap();
    RebuildBackendForActiveMap();
}

void SlamSystem::RebuildBackendForActiveMap() {
    visualizer_.ClearTrajectory();
    if (loop_closing_ && tracker_.mutable_map() != nullptr) {
        // Rebuild loop closer against the new active map.
        loop_closing_->RequestFinish();
        loop_closing_ = std::make_unique<backend::LoopClosing>(
            tracker_.mutable_map(), tracker_.mutable_keyframe_database(),
            multi_map_.get());
        const bool fix_scale =
            !(sensor_ == Sensor::kMonocular || sensor_ == Sensor::kImuMonocular);
        loop_closing_->set_fix_scale(fix_scale);
    }
    if (scheduler_) {
        StartBackendWorkers();
    }
}

void SlamSystem::ActivateLocalizationMode() {
    activate_localization_ = true;
}

void SlamSystem::DeactivateLocalizationMode() {
    deactivate_localization_ = true;
}

void SlamSystem::ApplyModeChange() {
    if (reset_) {
        tracker_.Reset();
        RebuildBackendForActiveMap();
        reset_ = false;
        reset_active_map_ = false;
        last_map_change_idx_ = 0;
    } else if (reset_active_map_) {
        tracker_.ResetActiveMap();
        RebuildBackendForActiveMap();
        reset_active_map_ = false;
        last_map_change_idx_ = 0;
    }
    mapping = tracker_.mutable_local_mapping();
    if (activate_localization_) {
        if (mapping != nullptr) {
            mapping->RequestPause();
            for (int i = 0; i < 50 && !mapping->isStopped(); ++i) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        tracker_.set_only_tracking(true);
        activate_localization_ = false;
    }
    if (deactivate_localization_) {
        tracker_.set_only_tracking(false);
        if (mapping != nullptr) {
            mapping->Release();
        }
        deactivate_localization_ = false;
    }
}

void SlamSystem::ChangeDataset() {
    Map* map = tracker_.mutable_map();
    if (map != nullptr && map->KeyFramesInMap() < 12) {
        tracker_.ResetActiveMap();
        RebuildBackendForActiveMap();
        return;
    }
    if (tracker_.mutable_local_mapping() != nullptr) {
        tracker_.mutable_local_mapping()->RequestStop();
    }
    tracker_.CreateMapInAtlas();
    RebuildBackendForActiveMap();
}

void SlamSystem::SaveDebugData(int init_idx) {
    auto* mapping = tracker_.mutable_local_mapping();
    if (mapping == nullptr) {
        return;
    }
    const int section = mapping->init_section();
    const std::string tag = std::to_string(section);

    if (Map* map = tracker_.mutable_map()) {
        std::ofstream trajectory("init_FrameTrajectoy_" + tag + "_" +
                                 std::to_string(init_idx) + ".txt");
        trajectory << std::fixed;
        auto keyframes = map->GetAllKeyFrames();
        std::sort(keyframes.begin(), keyframes.end(),
                  [](const auto& a, const auto& b) {
                      return a && b && a->timestamp < b->timestamp;
                  });
        for (const auto& kf : keyframes) {
            if (!kf) {
                continue;
            }
            const SE3 Twb = kf->GetImuPose();
            const Eigen::Quaterniond q(Twb.rotation());
            const Vec3 t = Twb.translation();
            const auto stamp_ns =
                static_cast<long long>(std::llround(kf->timestamp * 1e9));
            trajectory << stamp_ns << " " << t.x() << " " << t.y() << " "
                       << t.z() << " " << q.x() << " " << q.y() << " " << q.z()
                       << " " << q.w() << "\n";
        }
    }

    std::ofstream scale_file("init_Scale_" + tag + ".txt", std::ios::app);
    scale_file << std::fixed << mapping->imu_scale() << "\n";

    const Mat33& Rwg = mapping->gravity_rotation();
    std::ofstream gravity("init_GDir_" + tag + ".txt", std::ios::app);
    gravity << std::fixed;
    for (int r = 0; r < 3; ++r) {
        gravity << Rwg(r, 0) << "," << Rwg(r, 1) << "," << Rwg(r, 2) << "\n";
    }

    std::ofstream cost("init_CompCost_" + tag + ".txt", std::ios::app);
    cost << std::fixed << mapping->init_cost_sec() << "\n";

    const Vec3& bg = mapping->gyro_bias();
    const Vec3& ba = mapping->acc_bias();
    std::ofstream biases("init_Biases_" + tag + ".txt", std::ios::app);
    biases << std::fixed << bg.x() << "," << bg.y() << "," << bg.z() << "\n"
           << ba.x() << "," << ba.y() << "," << ba.z() << "\n";

    const Eigen::MatrixXd& cov = mapping->inertial_covariance();
    std::ofstream cov_file("init_CovMatrix_" + tag + "_" +
                               std::to_string(init_idx) + ".txt",
                           std::ios::app);
    cov_file << std::fixed << std::setprecision(15);
    for (int r = 0; r < cov.rows(); ++r) {
        for (int c = 0; c < cov.cols(); ++c) {
            if (c != 0) {
                cov_file << ",";
            }
            cov_file << cov(r, c);
        }
        cov_file << "\n";
    }

    std::ofstream time_file("init_Time_" + tag + ".txt", std::ios::app);
    time_file << std::fixed << mapping->init_time_sec() << "\n";
}

void SlamSystem::SaveTrajectoryTUM(const std::string& filename) {
    if (IsPureMonocular(sensor_)) {
        return;
    }
    Map* map = LargestMap(multi_map_.get(), tracker_.mutable_map());
    const auto keyframes = SortedKeyFrames(map);
    if (keyframes.empty() || !keyframes.front()) {
        return;
    }
    const SE3 origin = keyframes.front()->GetPoseInverse();
    std::ofstream out(filename);
    out << std::fixed;
    for (const auto& sample : tracker_.frame_poses()) {
        if (sample.lost || !sample.reference) {
            continue;
        }
        if (sample.reference->GetMap() != map &&
            sample.reference->GetMap() != nullptr) {
            continue;
        }
        const SE3 Twc = CameraPoseFromReference(sample.relative_cw,
                                                sample.reference, origin)
                            .inverse();
        WritePositionQuaternion(out, sample.timestamp, Twc, false);
    }
}

void SlamSystem::SaveKeyFrameTrajectoryTUM(const std::string& filename) {
    Map* map = LargestMap(multi_map_.get(), tracker_.mutable_map());
    std::ofstream out(filename);
    out << std::fixed;
    for (const auto& keyframe : SortedKeyFrames(map)) {
        if (!keyframe || keyframe->isBad()) {
            continue;
        }
        WritePositionQuaternion(out, keyframe->timestamp,
                                keyframe->GetPoseInverse(), false);
    }
}

void SlamSystem::SaveTrajectoryEuRoC(const std::string& filename) {
    SaveTrajectoryEuRoC(filename,
                        LargestMap(multi_map_.get(), tracker_.mutable_map()));
}

void SlamSystem::SaveTrajectoryEuRoC(const std::string& filename, Map* map) {
    const auto keyframes = SortedKeyFrames(map);
    if (keyframes.empty() || !keyframes.front()) {
        return;
    }
    const bool inertial = IsInertial(sensor_);
    const SE3 origin = inertial ? keyframes.front()->GetImuPose()
                                : keyframes.front()->GetPoseInverse();
    std::ofstream out(filename);
    out << std::fixed;
    for (const auto& sample : tracker_.frame_poses()) {
        if (sample.lost || !sample.reference ||
            sample.reference->GetMap() != map) {
            continue;
        }
        const SE3 camera = CameraPoseFromReference(sample.relative_cw,
                                                   sample.reference, origin);
        if (!inertial) {
            WritePositionQuaternion(out, sample.timestamp, camera.inverse(),
                                    true);
            continue;
        }
        const SE3 body_from_camera =
            sensor::imu::DefaultCalibOr(sample.reference->imu_calib)
                .T_camera_body.inverse();
        WritePositionQuaternion(out, sample.timestamp,
                                (body_from_camera * camera).inverse(), true);
    }
}

void SlamSystem::SaveKeyFrameTrajectoryEuRoC(const std::string& filename) {
    SaveKeyFrameTrajectoryEuRoC(
        filename, LargestMap(multi_map_.get(), tracker_.mutable_map()));
}

void SlamSystem::SaveKeyFrameTrajectoryEuRoC(const std::string& filename,
                                            Map* map) {
    if (map == nullptr) {
        return;
    }
    const bool inertial = IsInertial(sensor_);
    std::ofstream out(filename);
    out << std::fixed;
    for (const auto& keyframe : SortedKeyFrames(map)) {
        if (!keyframe || keyframe->isBad()) {
            continue;
        }
        const SE3 pose =
            inertial ? keyframe->GetImuPose() : keyframe->GetPoseInverse();
        WritePositionQuaternion(out, keyframe->timestamp, pose, true);
    }
}

void SlamSystem::SaveTrajectoryKITTI(const std::string& filename) {
    if (IsPureMonocular(sensor_)) {
        return;
    }
    Map* map = LargestMap(multi_map_.get(), tracker_.mutable_map());
    const auto keyframes = SortedKeyFrames(map);
    if (keyframes.empty() || !keyframes.front()) {
        return;
    }
    const SE3 origin = keyframes.front()->GetPoseInverse();
    std::ofstream out(filename);
    out << std::fixed << std::setprecision(9);
    for (const auto& sample : tracker_.frame_poses()) {
        if (!sample.reference) {
            continue;
        }
        const SE3 Twc = CameraPoseFromReference(sample.relative_cw,
                                                sample.reference, origin)
                            .inverse();
        const Mat33 rotation = Twc.rotation();
        const Vec3 translation = Twc.translation();
        for (int row = 0; row < 3; ++row) {
            if (row != 0) {
                out << " ";
            }
            out << rotation(row, 0) << " " << rotation(row, 1) << " "
                << rotation(row, 2) << " " << translation(row);
        }
        out << "\n";
    }
}

bool SlamSystem::MapChanged() {
    Map* map = tracker_.mutable_map();
    if (map == nullptr) {
        return false;
    }
    const int current = map->GetLastBigChangeIdx();
    if (last_map_change_idx_ < current) {
        last_map_change_idx_ = current;
        return true;
    }
    return false;
}

int SlamSystem::GetTrackingState() const {
    return static_cast<int>(tracker_.state());
}

bool SlamSystem::isLost() {
    const Map* map = tracker_.mutable_map();
    if (map == nullptr || !map->isImuInitialized()) {
        return false;
    }
    return tracker_.state() == tracking::Tracker::State::kLost;
}

double SlamSystem::GetTimeFromIMUInit() {
    const auto* mapping = tracker_.mutable_local_mapping();
    Map* map = tracker_.mutable_map();
    if (mapping == nullptr || map == nullptr || !map->isImuInitialized()) {
        return 0.0;
    }
    const double elapsed = mapping->current_keyframe_timestamp() -
                           mapping->first_imu_keyframe_timestamp();
    return elapsed > 0.0 ? elapsed : 0.0;
}

bool SlamSystem::isFinished() { return GetTimeFromIMUInit() > 0.1; }

void SlamSystem::Reset() { reset_ = true; }

void SlamSystem::ResetActiveMap() { reset_active_map_ = true; }

void SlamSystem::ChangeCalibration(const std::string& settings_path) {
    tracker_.ChangeCalibration(settings_path);
}

void SlamSystem::Shutdown() {
    if (!config_.atlas_save_file.empty()) {
        SaveAtlas(config_.atlas_save_file);
    }
    if (tracker_.mutable_local_mapping() != nullptr) {
        tracker_.mutable_local_mapping()->RequestStop();
    }
    if (loop_closing_) {
        loop_closing_->RequestFinish();
    }
    tracker_.set_async_mapping(false);
    scheduler_.reset();
    tracker_.Reset();
}

void SlamSystem::StartVisualization(
    const std::shared_ptr<autolink::Node>& node,
    const SlamVisualizer::Options& options) {
    SlamVisualizer::Options opts = options;
    if (opts.map_frame.empty()) {
        opts.map_frame = kMapFrame;
    }
    if (opts.body_frame.empty()) {
        opts.body_frame = kBodyFrame;
    }
    if (opts.camera_frame.empty()) {
        opts.camera_frame = kCameraFrame;
    }
    visualizer_.Configure(node, opts);
}

void SlamSystem::PublishVisualization(double timestamp_sec) {
    if (!visualizer_.enabled()) {
        return;
    }
    OdometryResult odom;
    if (!tracker_.GetResult(&odom)) {
        return;
    }
    visualizer_.Publish(odom, tracker_.mutable_map(), timestamp_sec);
}

SE3 SlamSystem::TrackStereo(const cv::Mat& left, const cv::Mat& right,
                            double timestamp) {
    ApplyModeChange();
    const SE3 pose = tracker_.GrabImageStereo(left, right, timestamp);
    FlushPendingKeyframes();
    PublishVisualization(timestamp);
    return pose;
}

SE3 SlamSystem::TrackRgbd(const cv::Mat& rgb, const cv::Mat& depth,
                          double timestamp) {
    ApplyModeChange();
    const SE3 pose = tracker_.GrabImageRgbd(rgb, depth, timestamp);
    FlushPendingKeyframes();
    PublishVisualization(timestamp);
    return pose;
}

SE3 SlamSystem::TrackMonocular(const cv::Mat& image, double timestamp) {
    ApplyModeChange();
    const SE3 pose = tracker_.GrabImageMonocular(image, timestamp);
    FlushPendingKeyframes();
    PublishVisualization(timestamp);
    return pose;
}

void SlamSystem::GrabImuData(const sensor::imu::Measurement& measurement) {
    tracker_.GrabImuData(measurement);
}

void SlamSystem::SetImuCalib(const sensor::imu::Calib& calib) {
    tracker_.SetImuCalib(calib);
}

bool SlamSystem::SaveAtlas(const std::string& path) const {
    if (!multi_map_ || path.empty()) {
        return false;
    }
    return SaveAtlasJson(*multi_map_, path);
}

bool SlamSystem::LoadAtlas(const std::string& path) {
    if (!multi_map_) {
        multi_map_ = std::make_shared<MultiMap>();
    }
    return LoadAtlasJson(multi_map_.get(), path);
}

bool SlamSystem::GetOdometry(OdometryResult* out) const {
    return tracker_.GetResult(out);
}

void SlamSystem::FlushPendingKeyframes() {
    Keyframe keyframe;
    while (tracker_.ConsumePendingKeyframe(&keyframe)) {
        map_manager_.AddKeyframe(keyframe);
    }
    // LoopClosing is fed by LocalMapping after each processed keyframe.
}

}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
