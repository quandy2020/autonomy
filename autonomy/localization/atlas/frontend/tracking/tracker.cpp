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
 *
 * Tracking flow adapted from ORB-SLAM3 Tracking.cc.
 */

/**
 * @file tracker.cpp
 * @brief Tracker implementation: GrabImage, Track, local map, keyframes, IMU preintegration.
 *
 * @note Main path should run on the Tracking thread; GrabImuData may enqueue from others.
 */

#include "autonomy/localization/atlas/frontend/tracking/tracker.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <set>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include "autonomy/localization/atlas/backend/optimizer.hpp"
#include "autonomy/localization/atlas/frontend/match/orb_matcher.hpp"
#include "autonomy/localization/atlas/frontend/solve/mlpnp_solver.hpp"
#include "autonomy/localization/atlas/frontend/tracking/pose_optimization.hpp"
#include "autonomy/localization/atlas/frontend/tracking/two_view_reconstruction.hpp"
#include "autonomy/localization/atlas/map/map_point.hpp"
#include "autonomy/localization/atlas/sensor/camera/camera_factory.hpp"

namespace autonomy {
namespace localization {
namespace atlas {
namespace tracking {
namespace {

std::string DefaultVocabularyPath() {
    const char* candidates[] = {
        "autonomy/localization/conf/atlas/orb_vocab.fbow",
        "share/autonomy/localization/conf/atlas/orb_vocab.fbow",
        "../share/autonomy/localization/conf/atlas/orb_vocab.fbow",
    };
    for (const char* path : candidates) {
        if (std::filesystem::exists(path)) {
            return path;
        }
    }
    return "autonomy/localization/conf/atlas/orb_vocab.fbow";
}

//! ORB CreateNewKeyFrame: also observe matched right index at Nleft + r.
void LinkFisheyeRightObservation(
    tracking::Frame* frame, const std::shared_ptr<KeyFrame>& keyframe,
    const std::shared_ptr<MapPoint>& map_point, int left_index) {
    if (!frame || !keyframe || !map_point || !frame->HasDualCameraIndex()) {
        return;
    }
    if (left_index < 0 ||
        left_index >= static_cast<int>(frame->left_to_right_match.size())) {
        return;
    }
    const int right_local = frame->left_to_right_match[static_cast<size_t>(
        left_index)];
    if (right_local < 0) {
        return;
    }
    const int right_index = frame->num_left + right_local;
    if (right_index >= 0 &&
        right_index < static_cast<int>(frame->map_points.size())) {
        frame->map_points[static_cast<size_t>(right_index)] = map_point;
    }
    map_point->AddObservation(keyframe, right_index);
    keyframe->AddMapPoint(map_point, right_index);
}

}  // namespace

bool Tracker::LoadVocabulary(const AtlasConfig& config) {
    vocabulary_ = std::make_shared<feature::OrbVocabulary>();
    std::string path = config.vocabulary_path;
    if (path.empty()) {
        path = DefaultVocabularyPath();
    }
    if (!vocabulary_->Load(path)) {
        vocabulary_.reset();
        keyframe_database_.reset();
        return false;
    }
    keyframe_database_ = std::make_unique<KeyFrameDatabase>(vocabulary_);
    return true;
}

bool Tracker::Init(const AtlasConfig& config, Options options) {
    config_ = config;
    options_ = std::move(options);
    if (options_.sensor == Sensor::kRgbd ||
        options_.sensor == Sensor::kImuRgbd) {
        // keep
    } else if (config_.mode == FrontendMode::kVo ||
               config_.mode == FrontendMode::kVio) {
        options_.sensor = Sensor::kRgbd;
    }

    // Optional Camera.newWidth/newHeight (ORB Settings resize).
    need_resize_ = false;
    if (config_.image_new_width > 0 && config_.image_new_height > 0 &&
        config_.image_width > 0 && config_.image_height > 0) {
        const double sx = static_cast<double>(config_.image_new_width) /
                          static_cast<double>(config_.image_width);
        const double sy = static_cast<double>(config_.image_new_height) /
                          static_cast<double>(config_.image_height);
        config_.camera_fx *= sx;
        config_.camera_fy *= sy;
        config_.camera_cx *= sx;
        config_.camera_cy *= sy;
        if (config_.camera_bf > 0.0) {
            config_.camera_bf *= sx;
            config_.camera_baseline_meters =
                config_.camera_bf / config_.camera_fx;
        }
        config_.camera_overlapping_begin = static_cast<int>(std::lround(
            config_.camera_overlapping_begin * sx));
        config_.camera_overlapping_end = static_cast<int>(std::lround(
            config_.camera_overlapping_end * sx));
        config_.camera2_overlapping_begin = static_cast<int>(std::lround(
            config_.camera2_overlapping_begin * sx));
        config_.camera2_overlapping_end = static_cast<int>(std::lround(
            config_.camera2_overlapping_end * sx));
        if (config_.camera2_fx > 0.0) {
            config_.camera2_fx *= sx;
            config_.camera2_fy *= sy;
            config_.camera2_cx *= sx;
            config_.camera2_cy *= sy;
        }
        resize_width_ = config_.image_new_width;
        resize_height_ = config_.image_new_height;
        need_resize_ = true;
        config_.image_width = config_.image_new_width;
        config_.image_height = config_.image_new_height;
    }
    BuildDistCoefFromConfig();
    BuildCameraFromConfig();

    orb_extractor_left_ = std::make_unique<feature::OrbExtractor>(
        options_.orb.num_features, options_.orb.scale_factor,
        options_.orb.num_levels, options_.orb.initial_fast_threshold,
        options_.orb.minimum_fast_threshold);
    orb_extractor_right_ = std::make_unique<feature::OrbExtractor>(
        options_.orb.num_features, options_.orb.scale_factor,
        options_.orb.num_levels, options_.orb.initial_fast_threshold,
        options_.orb.minimum_fast_threshold);
    orb_extractor_ini_ = std::make_unique<feature::OrbExtractor>(
        2 * options_.orb.num_features, options_.orb.scale_factor,
        options_.orb.num_levels, options_.orb.initial_fast_threshold,
        options_.orb.minimum_fast_threshold);

    LoadVocabulary(config_);

    if (multi_map_) {
        map_ = multi_map_->GetCurrentMap();
    } else {
        multi_map_ = std::make_shared<MultiMap>();
        map_ = multi_map_->GetCurrentMap();
    }
    const bool monocular = options_.sensor == Sensor::kMonocular ||
                           options_.sensor == Sensor::kImuMonocular;
    local_mapping_ = std::make_unique<LocalMapping>(map_.get(), monocular);
    local_mapping_->SetTracker(this);
    local_mapping_->SetFarPoints(config_.th_far_points);
    insert_kfs_when_lost_ = config_.insert_kfs_when_lost;
    Reset();
    return true;
}

void Tracker::SetMultiMap(const std::shared_ptr<MultiMap>& multi_map) {
    multi_map_ = multi_map;
    if (multi_map_) {
        map_ = multi_map_->GetCurrentMap();
    }
}

void Tracker::CreateNewMap() {
    if (local_mapping_) {
        local_mapping_->RequestStop();
    }
    if (!multi_map_) {
        multi_map_ = std::make_shared<MultiMap>();
    } else {
        multi_map_->CreateNewMap();
    }
    map_ = multi_map_->GetCurrentMap();
    const bool monocular = IsMonocularSensor();
    local_mapping_ = std::make_unique<LocalMapping>(map_.get(), monocular);
    local_mapping_->SetTracker(this);
    local_mapping_->SetFarPoints(config_.th_far_points);
    async_mapping_ = false;
    last_keyframe_.reset();
    reference_keyframe_.reset();
    local_keyframes_.clear();
    local_map_points_.clear();
    velocity_valid_ = false;
    has_initial_frame_ = false;
    ini_matches_.clear();
    state_ = State::kNoImagesYet;
}

void Tracker::CreateMapInAtlas() {
    CreateNewMap();
    velocity_ = SE3Identity();
    velocity_valid_ = false;
    last_frame_ = Frame();
    // Keep current_frame_ for the next GrabImage cycle.
    ResetFrameImuPreintegrator();
    EnsureImuCalib();
    if (IsImuSensor()) {
        imu_preintegrated_from_last_kf_ =
            std::make_shared<sensor::imu::Preintegrator>(last_bias_,
                                                         imu_calib_);
    }
    state_ = State::kNoImagesYet;
}

void Tracker::ResetActiveMap() {
    if (local_mapping_) {
        local_mapping_->RequestStop();
    }
    if (multi_map_) {
        multi_map_->clearMap();
        map_ = multi_map_->GetCurrentMap();
    } else if (map_) {
        map_->clear();
    }
    if (keyframe_database_) {
        // Keep vocab; drop inverted file entries for wiped map.
        keyframe_database_->clear();
    }
    const bool monocular = IsMonocularSensor();
    local_mapping_ = std::make_unique<LocalMapping>(map_.get(), monocular);
    local_mapping_->SetTracker(this);
    local_mapping_->SetFarPoints(config_.th_far_points);
    async_mapping_ = false;
    last_keyframe_.reset();
    reference_keyframe_.reset();
    local_keyframes_.clear();
    local_map_points_.clear();
    velocity_ = SE3Identity();
    velocity_valid_ = false;
    has_initial_frame_ = false;
    ini_matches_.clear();
    last_frame_ = Frame();
    frame_poses_.clear();
    ResetFrameImuPreintegrator();
    state_ = State::kNoImagesYet;
}

void Tracker::Reset() {
    if (local_mapping_) {
        local_mapping_->RequestStop();
    }
    if (multi_map_) {
        multi_map_->clearAtlas();
        multi_map_->CreateNewMap();
        map_ = multi_map_->GetCurrentMap();
    } else if (map_) {
        map_->clear();
    }
    if (keyframe_database_) {
        keyframe_database_->clear();
    }
    state_ = State::kNoImagesYet;
    last_keyframe_.reset();
    reference_keyframe_.reset();
    local_keyframes_.clear();
    local_map_points_.clear();
    velocity_ = SE3Identity();
    velocity_valid_ = false;
    matches_inliers_ = 0;
    last_result_ = OdometryResult{};
    pending_system_keyframe_.reset();
    last_keyframe_frame_id_ = 0;
    has_initial_frame_ = false;
    ini_matches_.clear();
    last_frame_ = Frame();
    frame_poses_.clear();
    Frame::next_id = 0;
    KeyFrame::next_id = 0;
    const bool monocular = IsMonocularSensor();
    local_mapping_ = std::make_unique<LocalMapping>(map_.get(), monocular);
    local_mapping_->SetTracker(this);
    local_mapping_->SetFarPoints(config_.th_far_points);
    async_mapping_ = false;
    ResetFrameImuPreintegrator();
}

void Tracker::PrepareFrame() {
    current_frame_.camera = camera_;
    current_frame_.camera2 = camera2_;
    if (has_T_c1_c2_) {
        current_frame_.T_c1_c2 = T_c1_c2_;
    }
    if (camera_ && camera_->width() > 0 && camera_->height() > 0) {
        current_frame_.ComputeImageBounds(camera_->width(), camera_->height());
    }
    if (!dist_coef_.empty()) {
        current_frame_.ApplyDistortion(dist_coef_);
    } else if (camera_ && camera_->width() > 0) {
        current_frame_.AssignFeaturesToGrid();
    }
    // Non-rectified fisheye stereo: BF + TriangulateMatches (ORB path).
    if (camera2_ && has_T_c1_c2_ &&
        current_frame_.orb_extractor_right != nullptr &&
        !current_frame_.keypoints_right.empty() && camera_ &&
        (camera_->type() == sensor::GeometricCamera::Type::kKannalaBrandt ||
         camera2_->type() == sensor::GeometricCamera::Type::kKannalaBrandt)) {
        current_frame_.ComputeStereoFishEyeMatches();
        current_frame_.FinalizeFisheyeStereo();
    }
    if (vocabulary_) {
        current_frame_.SetVocabulary(vocabulary_);
        current_frame_.ComputeBoW();
    }
}

void Tracker::CheckReplacedInLastFrame() {
    for (auto& map_point : last_frame_.map_points) {
        if (!map_point) {
            continue;
        }
        auto replaced = map_point->GetReplaced();
        if (replaced) {
            map_point = replaced;
        }
    }
}

void Tracker::RegisterKeyFrame(const std::shared_ptr<KeyFrame>& keyframe) {
    if (!keyframe) {
        return;
    }
    if (vocabulary_) {
        keyframe->SetVocabulary(vocabulary_);
        keyframe->ComputeBoW();
    }
    if (keyframe_database_) {
        keyframe_database_->Add(keyframe);
    }
}

bool Tracker::IsImuSensor() const {
    return options_.sensor == Sensor::kImuMonocular ||
           options_.sensor == Sensor::kImuStereo ||
           options_.sensor == Sensor::kImuRgbd;
}

void Tracker::GrabImuData(const sensor::imu::Measurement& measurement) {
    imu_queue_.Enqueue(measurement);
}

void Tracker::PreintegrateImu() {
    if (!IsImuSensor()) {
        return;
    }
    sensor::imu::Measurement m;
    while (imu_queue_.Dequeue(&m)) {
        imu_pending_.push_back(m);
    }
    if (imu_pending_.empty()) {
        return;
    }
    EnsureImuCalib();
    if (!imu_preintegrated_from_last_kf_) {
        imu_preintegrated_from_last_kf_ =
            std::make_shared<sensor::imu::Preintegrator>(last_bias_,
                                                         imu_calib_);
    }
    if (!imu_preintegrated_from_last_frame_) {
        imu_preintegrated_from_last_frame_ =
            std::make_shared<sensor::imu::Preintegrator>(last_bias_,
                                                         imu_calib_);
    }

    double last_t_kf =
        (last_keyframe_ ? last_keyframe_->timestamp
                        : imu_pending_.front().timestamp) +
        imu_preintegrated_from_last_kf_->delta_t;
    double last_t_fr =
        (last_frame_.has_pose() ? last_frame_.timestamp
                                : imu_pending_.front().timestamp) +
        imu_preintegrated_from_last_frame_->delta_t;
    while (!imu_pending_.empty() &&
           imu_pending_.front().timestamp <= current_frame_.timestamp) {
        const auto& sample = imu_pending_.front();
        auto ClampDt = [](double dt) {
            if (dt <= 0.0) {
                return 1e-3;
            }
            return std::min(dt, 0.5);
        };
        const double dt_kf = ClampDt(sample.timestamp - last_t_kf);
        const double dt_fr = ClampDt(sample.timestamp - last_t_fr);
        imu_preintegrated_from_last_kf_->IntegrateNewMeasurement(
            sample.acceleration, sample.angular_velocity, dt_kf);
        imu_preintegrated_from_last_frame_->IntegrateNewMeasurement(
            sample.acceleration, sample.angular_velocity, dt_fr);
        last_t_kf = sample.timestamp;
        last_t_fr = sample.timestamp;
        imu_pending_.pop_front();
    }

    current_frame_.imu_preintegrated = imu_preintegrated_from_last_kf_;
    current_frame_.imu_preintegrated_from_last_frame =
        imu_preintegrated_from_last_frame_;
    current_frame_.imu_bias = last_bias_;
    current_frame_.imu_calib = imu_calib_;
}

void Tracker::EnsureImuCalib() {
    if (imu_calib_.is_set) {
        return;
    }
    // Tbc = I until SetImuCalib; noise from AtlasConfig.
    imu_calib_.Set(SE3Identity(), config_.imu_gyro_noise,
                   config_.imu_accel_noise, config_.imu_gyro_bias_random_walk,
                   config_.imu_accel_bias_random_walk);
}

void Tracker::SetImuCalib(const sensor::imu::Calib& calib) {
    imu_calib_ = calib;
    imu_calib_.is_set = true;
}

void Tracker::ResetFrameImuPreintegrator() {
    EnsureImuCalib();
    imu_preintegrated_from_last_frame_ =
        std::make_shared<sensor::imu::Preintegrator>(last_bias_, imu_calib_);
}

bool Tracker::PredictStateImu() {
    if (!last_keyframe_ || !imu_preintegrated_from_last_kf_ ||
        !last_frame_.has_pose()) {
        return false;
    }
    const auto& pre = *imu_preintegrated_from_last_kf_;
    if (pre.delta_t < 1e-4) {
        return false;
    }
    EnsureImuCalib();
    current_frame_.imu_calib = imu_calib_;
    const SE3 Twb_last = last_keyframe_->GetImuPose();
    const Mat33 Rwb = Twb_last.rotation();
    const Vec3 twb = Twb_last.translation();
    const Vec3 vel = last_keyframe_->has_velocity ? last_keyframe_->velocity_world
                                                  : Vec3::Zero();
    const Vec3 g(0.0, 0.0, -sensor::imu::kGravity);
    const double dt = pre.delta_t;
    const Mat33 dR = pre.GetUpdatedDeltaRotation();
    const Vec3 dV = pre.GetUpdatedDeltaVelocity();
    const Vec3 dP = pre.GetUpdatedDeltaPosition();

    SE3 Twb_pred = SE3Identity();
    Twb_pred.linear() = Rwb * dR;
    Twb_pred.translation() =
        twb + vel * dt + 0.5 * g * dt * dt + Rwb * dP;
    current_frame_.SetImuPose(Twb_pred);
    current_frame_.velocity_world = vel + g * dt + Rwb * dV;
    current_frame_.has_velocity = true;
    current_frame_.imu_bias = last_bias_;
    current_frame_.imu_preintegrated = imu_preintegrated_from_last_kf_;
    current_frame_.imu_preintegrated_from_last_frame =
        imu_preintegrated_from_last_frame_;
    return true;
}

int Tracker::OptimizeCurrentPose() {
    if (!IsImuSensor() || !map_ || !map_->isImuInitialized()) {
        return PoseOptimization(&current_frame_);
    }
    current_frame_.imu_bias = last_bias_;
    current_frame_.imu_preintegrated = imu_preintegrated_from_last_kf_;
    current_frame_.imu_preintegrated_from_last_frame =
        imu_preintegrated_from_last_frame_;
    current_frame_.reference_keyframe = last_keyframe_
                                            ? last_keyframe_
                                            : reference_keyframe_;

    int inliers = 0;
    const bool reinit = map_ && !map_->GetInertialBA2();
    if (!map_updated_for_inertial_ &&
        current_frame_.imu_preintegrated_from_last_frame &&
        last_frame_.has_pose()) {
        inliers = backend::Optimizer::PoseInertialOptimizationLastFrame(
            &current_frame_, &last_frame_, reinit);
    } else if (last_keyframe_ && current_frame_.imu_preintegrated) {
        inliers = backend::Optimizer::PoseInertialOptimizationLastKeyFrame(
            &current_frame_, last_keyframe_, reinit);
        map_updated_for_inertial_ = false;
    } else {
        inliers = PoseOptimization(&current_frame_);
    }
    last_bias_ = current_frame_.imu_bias;
    return inliers;
}

bool Tracker::IsMonocularSensor() const {
    return options_.sensor == Sensor::kMonocular ||
           options_.sensor == Sensor::kImuMonocular;
}

cv::Mat Tracker::ToGray(const cv::Mat& image) const {
    if (image.empty()) {
        return {};
    }
    if (image.channels() == 1) {
        return image;
    }
    cv::Mat gray;
    if (options_.rgb) {
        cv::cvtColor(image, gray,
                     image.channels() == 4 ? cv::COLOR_RGBA2GRAY
                                          : cv::COLOR_RGB2GRAY);
    } else {
        cv::cvtColor(image, gray,
                     image.channels() == 4 ? cv::COLOR_BGRA2GRAY
                                          : cv::COLOR_BGR2GRAY);
    }
    return gray;
}

void Tracker::BuildDistCoefFromConfig() {
    dist_coef_ = cv::Mat::zeros(4, 1, CV_32F);
    if (config_.camera_distortion.empty()) {
        return;
    }
    const int n = static_cast<int>(config_.camera_distortion.size());
    dist_coef_ = cv::Mat::zeros(std::max(n, 4), 1, CV_32F);
    for (int i = 0; i < n; ++i) {
        dist_coef_.at<float>(i) =
            static_cast<float>(config_.camera_distortion[static_cast<size_t>(i)]);
    }
    if (std::fabs(dist_coef_.at<float>(0)) > 1e-12f) {
        config_.need_undistort = true;
    }
}

void Tracker::BuildCameraFromConfig() {
    using sensor::camera::CameraFactory;
    CameraFactory::Intrinsics intr;
    switch (config_.camera_type) {
        case CameraModelType::kKannalaBrandt:
            intr.model = "kannala_brandt";
            break;
        case CameraModelType::kRadTan:
            intr.model = "radtan";
            break;
        case CameraModelType::kFov:
            intr.model = "fov";
            break;
        case CameraModelType::kUcm:
            intr.model = "ucm";
            break;
        case CameraModelType::kEucm:
            intr.model = "eucm";
            break;
        case CameraModelType::kDoubleSphere:
            intr.model = "double_sphere";
            break;
        case CameraModelType::kEquirectangular:
            intr.model = "equirectangular";
            break;
        case CameraModelType::kRadialDivision:
            intr.model = "radial_division";
            break;
        case CameraModelType::kRectified:
        case CameraModelType::kPinhole:
        default:
            intr.model = "pinhole";
            break;
    }
    intr.fx = config_.camera_fx;
    intr.fy = config_.camera_fy;
    intr.cx = config_.camera_cx;
    intr.cy = config_.camera_cy;
    intr.width = config_.image_width;
    intr.height = config_.image_height;
    const auto& d = config_.camera_distortion;
    if (d.size() >= 1) {
        intr.k1 = d[0];
    }
    if (d.size() >= 2) {
        intr.k2 = d[1];
    }
    if (d.size() >= 3) {
        intr.p1 = d[2];
    }
    if (d.size() >= 4) {
        intr.p2 = d[3];
    }
    if (d.size() >= 5) {
        intr.k3 = d[4];
    }
    if (d.size() >= 6) {
        intr.k4 = d[5];
    }
    // KannalaBrandt: k1..k4 often occupy first 4 slots.
    if (config_.camera_type == CameraModelType::kKannalaBrandt &&
        d.size() >= 4) {
        intr.k1 = d[0];
        intr.k2 = d[1];
        intr.k3 = d[2];
        intr.k4 = d[3];
        intr.p1 = 0.0;
        intr.p2 = 0.0;
    }
    camera_ = CameraFactory::Create(intr);

    camera2_.reset();
    has_T_c1_c2_ = false;
    T_c1_c2_ = SE3Identity();
    if (config_.camera2_fx > 0.0 && config_.camera2_fy > 0.0) {
        CameraFactory::Intrinsics intr2 = intr;
        intr2.fx = config_.camera2_fx;
        intr2.fy = config_.camera2_fy;
        intr2.cx = config_.camera2_cx;
        intr2.cy = config_.camera2_cy;
        const auto& d2 = config_.camera2_distortion;
        if (d2.size() >= 1) {
            intr2.k1 = d2[0];
        }
        if (d2.size() >= 2) {
            intr2.k2 = d2[1];
        }
        if (d2.size() >= 3) {
            intr2.k3 = d2[2];
        }
        if (d2.size() >= 4) {
            intr2.k4 = d2[3];
        }
        if (config_.camera_type == CameraModelType::kKannalaBrandt &&
            d2.size() >= 4) {
            intr2.k1 = d2[0];
            intr2.k2 = d2[1];
            intr2.k3 = d2[2];
            intr2.k4 = d2[3];
            intr2.p1 = 0.0;
            intr2.p2 = 0.0;
        }
        camera2_ = CameraFactory::Create(intr2);
    }
    if (config_.T_c1_c2.size() == 16) {
        Mat33 R;
        R << config_.T_c1_c2[0], config_.T_c1_c2[1], config_.T_c1_c2[2],
            config_.T_c1_c2[4], config_.T_c1_c2[5], config_.T_c1_c2[6],
            config_.T_c1_c2[8], config_.T_c1_c2[9], config_.T_c1_c2[10];
        const Vec3 t(config_.T_c1_c2[3], config_.T_c1_c2[7],
                     config_.T_c1_c2[11]);
        T_c1_c2_ = SE3Identity();
        T_c1_c2_.linear() = R;
        T_c1_c2_.translation() = t;
        has_T_c1_c2_ = true;
        // Fill bf from |t| when baseline unset.
        const double bl = t.norm();
        if (bl > 1e-6 && config_.camera_baseline_meters <= 0.0) {
            config_.camera_baseline_meters = bl;
        }
    }
}

cv::Mat Tracker::PreprocessImage(const cv::Mat& image) const {
    cv::Mat gray = ToGray(image);
    if (gray.empty()) {
        return gray;
    }
    if (need_resize_ && resize_width_ > 0 && resize_height_ > 0) {
        cv::Mat resized;
        cv::resize(gray, resized, cv::Size(resize_width_, resize_height_));
        return resized;
    }
    return gray;
}

SE3 Tracker::GrabImageRgbd(const cv::Mat& rgb, const cv::Mat& depth,
                           double timestamp) {
    cv::Mat gray = PreprocessImage(rgb);
    cv::Mat depth32 = depth;
    if (need_resize_ && resize_width_ > 0 && resize_height_ > 0 &&
        !depth.empty()) {
        cv::resize(depth, depth32, cv::Size(resize_width_, resize_height_), 0,
                   0, cv::INTER_NEAREST);
    }
    if (depth32.type() != CV_32F ||
        std::abs(options_.depth_map_factor - 1.f) > 1e-5f) {
        depth32.convertTo(depth32, CV_32F, options_.depth_map_factor);
    }

    const float bf = static_cast<float>(config_.camera_fx *
                                        config_.camera_baseline_meters);
    current_frame_ =
        Frame(gray, depth32, timestamp, orb_extractor_left_.get(),
              static_cast<float>(config_.camera_fx),
              static_cast<float>(config_.camera_fy),
              static_cast<float>(config_.camera_cx),
              static_cast<float>(config_.camera_cy), bf,
              options_.depth_threshold);
    PrepareFrame();
    Track();
    return current_frame_.has_pose() ? current_frame_.GetPose()
                                     : SE3Identity();
}

SE3 Tracker::GrabImageStereo(const cv::Mat& left, const cv::Mat& right,
                             double timestamp) {
    cv::Mat left_gray = PreprocessImage(left);
    cv::Mat right_gray = PreprocessImage(right);
    const float bf = static_cast<float>(config_.camera_fx *
                                        config_.camera_baseline_meters);
    const std::vector<int> lap_l = {config_.camera_overlapping_begin,
                                    config_.camera_overlapping_end};
    const std::vector<int> lap_r = {config_.camera2_overlapping_begin,
                                    config_.camera2_overlapping_end};
    current_frame_ = Frame(
        left_gray, right_gray, timestamp, orb_extractor_left_.get(),
        orb_extractor_right_.get(), static_cast<float>(config_.camera_fx),
        static_cast<float>(config_.camera_fy),
        static_cast<float>(config_.camera_cx),
        static_cast<float>(config_.camera_cy), bf, options_.depth_threshold,
        lap_l, lap_r);
    PrepareFrame();
    Track();
    return current_frame_.has_pose() ? current_frame_.GetPose()
                                     : SE3Identity();
}

SE3 Tracker::GrabImageMonocular(const cv::Mat& image, double timestamp) {
    cv::Mat gray = PreprocessImage(image);
    feature::OrbExtractor* extractor =
        (state_ == State::kNotInitialized || state_ == State::kNoImagesYet)
            ? orb_extractor_ini_.get()
            : orb_extractor_left_.get();
    const float bf = static_cast<float>(config_.camera_fx *
                                        config_.camera_baseline_meters);
    current_frame_ =
        Frame(gray, timestamp, extractor, static_cast<float>(config_.camera_fx),
              static_cast<float>(config_.camera_fy),
              static_cast<float>(config_.camera_cx),
              static_cast<float>(config_.camera_cy), bf,
              options_.depth_threshold);
    PrepareFrame();
    Track();
    return current_frame_.has_pose() ? current_frame_.GetPose()
                                     : SE3Identity();
}

bool Tracker::Process(const SensorData& data) {
    if (data.has_imu) {
        for (const auto& imu_msg : data.imu) {
            sensor::imu::Measurement m;
            m.timestamp = HeaderStampSec(imu_msg.header());
            m.acceleration = Vec3(imu_msg.linear_acceleration().x(),
                                  imu_msg.linear_acceleration().y(),
                                  imu_msg.linear_acceleration().z());
            m.angular_velocity = Vec3(imu_msg.angular_velocity().x(),
                                      imu_msg.angular_velocity().y(),
                                      imu_msg.angular_velocity().z());
            GrabImuData(m);
        }
    }

    if (!data.has_image) {
        PublishResult(false);
        return false;
    }

    cv::Mat image = ImageToCvMat(data.image);
    if (image.empty()) {
        PublishResult(false);
        return false;
    }
    if (data.image.encoding() == "rgb8") {
        cv::cvtColor(image, image, cv::COLOR_RGB2BGR);
    }

    const double stamp = HeaderStampSec(data.image.header());
    if (data.has_depth && !data.depth.data().empty()) {
        cv::Mat depth = ImageToCvMat(data.depth);
        if (depth.empty()) {
            PublishResult(false);
            return false;
        }
        if (depth.type() == CV_16UC1) {
            depth.convertTo(depth, CV_32FC1, 1.0 / 1000.0);
        }
        GrabImageRgbd(image, depth, stamp);
    } else if (data.has_image_right && !data.image_right.data().empty()) {
        cv::Mat right = ImageToCvMat(data.image_right);
        if (right.empty()) {
            PublishResult(false);
            return false;
        }
        if (data.image_right.encoding() == "rgb8") {
            cv::cvtColor(right, right, cv::COLOR_RGB2BGR);
        }
        GrabImageStereo(image, right, stamp);
    } else {
        GrabImageMonocular(image, stamp);
    }
    return last_result_.valid;
}

void Tracker::Track() {
    if (local_mapping_ && local_mapping_->bad_imu()) {
        ResetActiveMap();
        state_ = State::kNoImagesYet;
        PublishResult(false);
        return;
    }

    if (state_ == State::kNoImagesYet) {
        state_ = State::kNotInitialized;
    }

    if (IsImuSensor()) {
        PreintegrateImu();
    }

    if (state_ == State::kNotInitialized) {
        if (options_.sensor == Sensor::kStereo ||
            options_.sensor == Sensor::kRgbd ||
            options_.sensor == Sensor::kImuStereo ||
            options_.sensor == Sensor::kImuRgbd) {
            StereoInitialization();
        } else if (IsMonocularSensor()) {
            MonocularInitialization();
        }
        last_frame_ = current_frame_;
        PublishResult(state_ == State::kOk);
        return;
    }

    bool ok = false;
    if (!only_tracking_) {
        if (state_ == State::kOk) {
            CheckReplacedInLastFrame();
            if (IsImuSensor() && map_ && map_->isImuInitialized() &&
                PredictStateImu()) {
                ok = TrackWithMotionModel();
                if (!ok) {
                    ok = TrackReferenceKeyFrame();
                }
            } else if (velocity_valid_) {
                ok = TrackWithMotionModel();
                if (!ok) {
                    ok = TrackReferenceKeyFrame();
                }
            } else {
                ok = TrackReferenceKeyFrame();
            }
            if (!ok) {
                const int n_kfs =
                    map_ ? static_cast<int>(map_->KeyFramesInMap()) : 0;
                if (n_kfs > 10) {
                    state_ = State::kRecentlyLost;
                    time_stamp_lost_ = current_frame_.timestamp;
                } else {
                    state_ = State::kLost;
                }
            }
        } else if (state_ == State::kRecentlyLost) {
            ok = true;
            if (IsImuSensor()) {
                if (map_ && map_->isImuInitialized()) {
                    PredictStateImu();
                } else {
                    ok = false;
                }
                if (current_frame_.timestamp - time_stamp_lost_ >
                    time_recently_lost_) {
                    state_ = State::kLost;
                    ok = false;
                }
            } else {
                ok = Relocalization();
                if (!ok && current_frame_.timestamp - time_stamp_lost_ > 3.0) {
                    state_ = State::kLost;
                }
            }
        } else if (state_ == State::kLost) {
            const int n_kfs =
                map_ ? static_cast<int>(map_->KeyFramesInMap()) : 0;
            if (n_kfs < 10) {
                ResetActiveMap();
            } else {
                CreateMapInAtlas();
            }
            last_frame_ = current_frame_;
            PublishResult(false);
            return;
        }
    } else {
        // Localization-only (ORB mbOnlyTracking): VO dual-hypothesis when mbVO.
        if (state_ == State::kLost) {
            ok = Relocalization();
            if (ok) {
                vo_mode_ = false;
            }
        } else {
            CheckReplacedInLastFrame();
            if (!vo_mode_) {
                if (velocity_valid_) {
                    ok = TrackWithMotionModel();
                } else {
                    ok = TrackReferenceKeyFrame();
                }
            } else {
                // Few map matches last frame: try motion model + reloc, prefer reloc.
                bool ok_mm = false;
                bool ok_reloc = false;
                std::vector<std::shared_ptr<MapPoint>> mps_mm;
                std::vector<bool> outliers_mm;
                SE3 Tcw_mm = SE3Identity();
                if (velocity_valid_) {
                    ok_mm = TrackWithMotionModel();
                    mps_mm = current_frame_.map_points;
                    outliers_mm = current_frame_.outliers;
                    if (current_frame_.has_pose()) {
                        Tcw_mm = current_frame_.GetPose();
                    }
                }
                ok_reloc = Relocalization();
                if (ok_mm && !ok_reloc) {
                    current_frame_.SetPose(Tcw_mm);
                    current_frame_.map_points = mps_mm;
                    current_frame_.outliers = outliers_mm;
                    if (vo_mode_) {
                        for (int i = 0; i < current_frame_.TotalFeatures();
                             ++i) {
                            auto& mp =
                                current_frame_.map_points[static_cast<size_t>(i)];
                            if (mp &&
                                i < static_cast<int>(
                                        current_frame_.outliers.size()) &&
                                !current_frame_.outliers[static_cast<size_t>(
                                    i)]) {
                                mp->IncreaseFound();
                            }
                        }
                    }
                } else if (ok_reloc) {
                    vo_mode_ = false;
                }
                ok = ok_reloc || ok_mm;
            }
        }
    }

    current_frame_.reference_keyframe = reference_keyframe_;

    if (!only_tracking_) {
        if (ok) {
            ok = TrackLocalMap();
        }
    } else if (ok && !vo_mode_) {
        // Localization: skip TrackLocalMap while in VO mode (ORB).
        ok = TrackLocalMap();
    }

    if (ok) {
        state_ = State::kOk;
        if (last_frame_.has_pose() && current_frame_.has_pose()) {
            velocity_ =
                current_frame_.GetPose() * last_frame_.GetPose().inverse();
            velocity_valid_ = true;
        }
        if (NeedNewKeyFrame()) {
            CreateNewKeyFrame();
        }
    } else if (state_ == State::kRecentlyLost) {
        velocity_valid_ = false;
        if (insert_kfs_when_lost_ && NeedNewKeyFrame()) {
            CreateNewKeyFrame();
        }
    } else if (state_ != State::kLost) {
        const int n_kfs =
            map_ ? static_cast<int>(map_->KeyFramesInMap()) : 0;
        if (n_kfs > 10) {
            state_ = State::kRecentlyLost;
            time_stamp_lost_ = current_frame_.timestamp;
        } else {
            state_ = State::kLost;
        }
        velocity_valid_ = false;
    }

    if (local_mapping_ && !async_mapping_) {
        while (local_mapping_->ProcessNextKeyFrame()) {
        }
    }

    RecordFramePose();
    last_frame_ = current_frame_;
    ResetFrameImuPreintegrator();
    PublishResult(state_ == State::kOk || state_ == State::kRecentlyLost);
}

void Tracker::RecordFramePose() {
    if (state_ != State::kOk && state_ != State::kRecentlyLost) {
        return;
    }
    FramePoseRecord sample;
    sample.timestamp = current_frame_.timestamp;
    sample.lost = false;
    if (current_frame_.has_pose() && reference_keyframe_) {
        sample.relative_cw = current_frame_.GetPose() *
                             reference_keyframe_->GetPoseInverse();
        sample.reference = reference_keyframe_;
    } else if (!frame_poses_.empty()) {
        sample.relative_cw = frame_poses_.back().relative_cw;
        sample.reference = frame_poses_.back().reference;
        sample.lost = true;
    } else {
        return;
    }
    frame_poses_.push_back(sample);
}

void Tracker::ChangeCalibration(const std::string& settings_path) {
    AtlasConfig loaded;
    if (!LoadConfig(settings_path, &loaded)) {
        return;
    }
    config_.camera_type = loaded.camera_type;
    config_.camera_fx = loaded.camera_fx;
    config_.camera_fy = loaded.camera_fy;
    config_.camera_cx = loaded.camera_cx;
    config_.camera_cy = loaded.camera_cy;
    config_.camera_distortion = loaded.camera_distortion;
    config_.camera_bf = loaded.camera_bf;
    config_.camera_baseline_meters = loaded.camera_baseline_meters;
    config_.image_width = loaded.image_width;
    config_.image_height = loaded.image_height;
    config_.camera2_fx = loaded.camera2_fx;
    config_.camera2_fy = loaded.camera2_fy;
    config_.camera2_cx = loaded.camera2_cx;
    config_.camera2_cy = loaded.camera2_cy;
    config_.camera2_distortion = loaded.camera2_distortion;
    BuildDistCoefFromConfig();
    BuildCameraFromConfig();
}

void Tracker::StereoInitialization() {
    if (current_frame_.num_keypoints <= 500) {
        return;
    }

    current_frame_.SetPose(SE3Identity());

    auto keyframe = std::make_shared<KeyFrame>(current_frame_, map_.get());
    map_->AddKeyFrame(keyframe);

    for (int i = 0; i < current_frame_.num_keypoints; ++i) {
        if (current_frame_.depths[i] <= 0.f) {
            continue;
        }
        Vec3 point_world;
        if (!current_frame_.UnprojectStereo(i, &point_world)) {
            continue;
        }
        auto map_point =
            std::make_shared<MapPoint>(point_world, keyframe, map_.get());
        map_point->AddObservation(keyframe, i);
        keyframe->AddMapPoint(map_point, i);
        LinkFisheyeRightObservation(&current_frame_, keyframe, map_point, i);
        map_point->ComputeDistinctiveDescriptors();
        map_point->UpdateNormalAndDepth();
        map_->AddMapPoint(map_point);
        current_frame_.map_points[i] = map_point;
    }

    map_->keyframe_origins.push_back(keyframe);
    map_->SetReferenceMapPoints(map_->GetAllMapPoints());

    last_keyframe_ = keyframe;
    reference_keyframe_ = keyframe;
    current_frame_.reference_keyframe = keyframe;
    local_keyframes_ = {keyframe};
    local_map_points_ = map_->GetAllMapPoints();
    last_keyframe_frame_id_ = current_frame_.id;
    pending_system_keyframe_ = MakeSystemKeyframe(keyframe);

    RegisterKeyFrame(keyframe);

    if (local_mapping_) {
        local_mapping_->InsertKeyFrame(keyframe);
        if (!async_mapping_) {
            local_mapping_->ProcessNextKeyFrame();
        }
    }

    state_ = State::kOk;
}

void Tracker::MonocularInitialization() {
    if (!has_initial_frame_) {
        if (current_frame_.num_keypoints > 100) {
            initial_frame_ = current_frame_;
            has_initial_frame_ = true;
            ini_matches_.assign(current_frame_.num_keypoints, -1);
        }
        return;
    }

    // Match initial → current (ORB-SLAM3 SearchForInitialization).
    if (initial_frame_.descriptors.empty() ||
        current_frame_.descriptors.empty()) {
        return;
    }
    if (ini_prev_matched_.size() !=
        static_cast<size_t>(initial_frame_.num_keypoints)) {
        ini_prev_matched_.resize(
            static_cast<size_t>(initial_frame_.num_keypoints));
        for (int i = 0; i < initial_frame_.num_keypoints; ++i) {
            ini_prev_matched_[static_cast<size_t>(i)] =
                initial_frame_.keypoints_undistorted[static_cast<size_t>(i)].pt;
        }
    }
    feature::OrbMatcher matcher(0.9f, true);
    std::vector<int> matches12;
    const int nmatches = matcher.SearchForInitialization(
        initial_frame_, current_frame_, &ini_prev_matched_, &matches12, 100);
    if (nmatches < 100) {
        return;
    }
    ini_matches_ = matches12;

    Mat33 K = Mat33::Identity();
    K(0, 0) = config_.camera_fx;
    K(1, 1) = config_.camera_fy;
    K(0, 2) = config_.camera_cx;
    K(1, 2) = config_.camera_cy;
    TwoViewReconstruction reconstructor(K);
    SE3 T21;
    std::vector<cv::Point3f> points3d;
    std::vector<bool> triangulated;
    if (!reconstructor.Reconstruct(initial_frame_.keypoints_undistorted,
                                   current_frame_.keypoints_undistorted,
                                   matches12, &T21, &points3d, &triangulated)) {
        return;
    }

    initial_frame_.SetPose(SE3Identity());
    current_frame_.SetPose(T21);

    auto keyframe1 =
        std::make_shared<KeyFrame>(initial_frame_, map_.get());
    auto keyframe2 = std::make_shared<KeyFrame>(current_frame_, map_.get());
    map_->AddKeyFrame(keyframe1);
    map_->AddKeyFrame(keyframe2);

    for (size_t i = 0; i < triangulated.size(); ++i) {
        if (!triangulated[i] || matches12[i] < 0) {
            continue;
        }
        const cv::Point3f& p = points3d[i];
        const Vec3 Pw(p.x, p.y, p.z);
        auto map_point = std::make_shared<MapPoint>(Pw, keyframe1, map_.get());
        map_point->AddObservation(keyframe1, static_cast<int>(i));
        map_point->AddObservation(keyframe2, matches12[i]);
        keyframe1->AddMapPoint(map_point, static_cast<int>(i));
        keyframe2->AddMapPoint(map_point, matches12[i]);
        map_point->ComputeDistinctiveDescriptors();
        map_point->UpdateNormalAndDepth();
        map_->AddMapPoint(map_point);
        current_frame_.map_points[matches12[i]] = map_point;
    }

    keyframe1->UpdateConnections();
    keyframe2->UpdateConnections();
    map_->keyframe_origins.push_back(keyframe1);
    map_->SetReferenceMapPoints(map_->GetAllMapPoints());

    last_keyframe_ = keyframe2;
    reference_keyframe_ = keyframe2;
    current_frame_.reference_keyframe = keyframe2;
    local_keyframes_ = {keyframe1, keyframe2};
    local_map_points_ = map_->GetAllMapPoints();
    last_keyframe_frame_id_ = current_frame_.id;
    pending_system_keyframe_ = MakeSystemKeyframe(keyframe2);
    has_initial_frame_ = false;

    RegisterKeyFrame(keyframe1);
    RegisterKeyFrame(keyframe2);

    if (local_mapping_) {
        local_mapping_->InsertKeyFrame(keyframe1);
        local_mapping_->InsertKeyFrame(keyframe2);
        if (!async_mapping_) {
            while (local_mapping_->ProcessNextKeyFrame()) {
            }
        }
    }
    state_ = State::kOk;
}

bool Tracker::TrackReferenceKeyFrame() {
    if (!reference_keyframe_ || !last_frame_.has_pose()) {
        return false;
    }

    feature::OrbMatcher matcher(0.7f, true);
    std::vector<std::shared_ptr<MapPoint>> matches;
    const int nmatches =
        matcher.SearchByBoW(reference_keyframe_, current_frame_, &matches);
    if (nmatches < 15) {
        return false;
    }

    current_frame_.map_points = matches;
    current_frame_.SetPose(last_frame_.GetPose());
    OptimizeCurrentPose();

    int nmatches_map = 0;
    for (int i = 0; i < current_frame_.num_keypoints; ++i) {
        auto& map_point = current_frame_.map_points[i];
        if (!map_point) {
            continue;
        }
        if (current_frame_.outliers[i]) {
            map_point->track_in_view = false;
            map_point->last_frame_seen = current_frame_.id;
            map_point.reset();
            current_frame_.outliers[i] = false;
        } else if (map_point->Observations() > 0) {
            ++nmatches_map;
        }
    }
    matches_inliers_ = nmatches_map;
    return IsImuSensor() ? true : nmatches_map >= 10;
}

bool Tracker::TrackWithMotionModel() {
    if (!last_frame_.has_pose()) {
        return false;
    }
    UpdateLastFrame();

    const bool imu_predicted =
        IsImuSensor() && map_ && map_->isImuInitialized() &&
        current_frame_.has_pose();
    if (!imu_predicted && !velocity_valid_) {
        return false;
    }

    feature::OrbMatcher matcher(0.9f, true);
    if (!imu_predicted) {
        current_frame_.SetPose(velocity_ * last_frame_.GetPose());
    }
    std::fill(current_frame_.map_points.begin(), current_frame_.map_points.end(),
              nullptr);

    const bool monocular = options_.sensor == Sensor::kMonocular ||
                           options_.sensor == Sensor::kImuMonocular;
    int th = (options_.sensor == Sensor::kStereo) ? 7 : 15;
    int nmatches =
        matcher.SearchByProjection(current_frame_, last_frame_, th, monocular);
    if (nmatches < 20) {
        std::fill(current_frame_.map_points.begin(),
                  current_frame_.map_points.end(), nullptr);
        nmatches = matcher.SearchByProjection(current_frame_, last_frame_,
                                              2.f * th, monocular);
    }
    if (nmatches < 20) {
        return IsImuSensor();
    }

    OptimizeCurrentPose();

    int nmatches_map = 0;
    const int n_feat = current_frame_.TotalFeatures();
    for (int i = 0; i < n_feat; ++i) {
        auto& map_point = current_frame_.map_points[static_cast<size_t>(i)];
        if (!map_point) {
            continue;
        }
        if (i < static_cast<int>(current_frame_.outliers.size()) &&
            current_frame_.outliers[static_cast<size_t>(i)]) {
            map_point->track_in_view = false;
            map_point->last_frame_seen = current_frame_.id;
            map_point.reset();
            current_frame_.outliers[static_cast<size_t>(i)] = false;
            --nmatches;
        } else if (map_point->Observations() > 0) {
            ++nmatches_map;
        }
    }
    matches_inliers_ = nmatches_map;
    if (only_tracking_) {
        vo_mode_ = nmatches_map < 10;
        return nmatches > 20;
    }
    return IsImuSensor() ? true : nmatches_map >= 10;
}

bool Tracker::TrackLocalMap() {
    UpdateLocalMap();
    SearchLocalPoints();
    OptimizeCurrentPose();

    matches_inliers_ = 0;
    for (int i = 0; i < current_frame_.num_keypoints; ++i) {
        auto& map_point = current_frame_.map_points[i];
        if (!map_point) {
            continue;
        }
        if (current_frame_.outliers[i]) {
            map_point.reset();
        } else if (map_point->Observations() > 0) {
            ++matches_inliers_;
        }
    }
    return matches_inliers_ >= 30 ||
           (matches_inliers_ >= 15 && local_map_points_.size() < 50);
}

void Tracker::UpdateLocalMap() {
    UpdateLocalKeyFrames();
    UpdateLocalPoints();
}

void Tracker::UpdateLocalKeyFrames() {
    local_keyframes_.clear();
    if (reference_keyframe_) {
        local_keyframes_ = reference_keyframe_->GetBestCovisibilityKeyFrames(80);
        if (std::find(local_keyframes_.begin(), local_keyframes_.end(),
                      reference_keyframe_) == local_keyframes_.end()) {
            local_keyframes_.insert(local_keyframes_.begin(),
                                    reference_keyframe_);
        }
    }
    if (last_keyframe_ &&
        std::find(local_keyframes_.begin(), local_keyframes_.end(),
                  last_keyframe_) == local_keyframes_.end()) {
        local_keyframes_.push_back(last_keyframe_);
    }
    if (local_keyframes_.empty() && map_) {
        local_keyframes_ = map_->GetAllKeyFrames();
    }
    if (!local_keyframes_.empty()) {
        reference_keyframe_ = local_keyframes_.front();
    }
}

void Tracker::UpdateLocalPoints() {
    local_map_points_.clear();
    for (const auto& keyframe : local_keyframes_) {
        if (!keyframe) {
            continue;
        }
        for (const auto& map_point : keyframe->GetMapPoints()) {
            if (!map_point || map_point->isBad()) {
                continue;
            }
            if (map_point->last_frame_seen == current_frame_.id) {
                continue;
            }
            local_map_points_.push_back(map_point);
            map_point->last_frame_seen = current_frame_.id;
        }
    }
}

void Tracker::SearchLocalPoints() {
    if (!current_frame_.has_pose()) {
        return;
    }

    // Mark already-matched points (ORB SearchLocalPoints).
    for (auto& map_point : current_frame_.map_points) {
        if (!map_point) {
            continue;
        }
        if (map_point->isBad()) {
            map_point.reset();
            continue;
        }
        map_point->IncreaseVisible();
        map_point->last_frame_seen = current_frame_.id;
        map_point->track_in_view = false;
        map_point->track_in_view_r = false;
    }

    int n_to_match = 0;
    for (const auto& map_point : local_map_points_) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        if (map_point->last_frame_seen == current_frame_.id) {
            continue;
        }
        const bool in_left =
            current_frame_.IsInFrustum(map_point, 0.5f, false);
        bool in_right = false;
        if (current_frame_.HasDualCameraIndex()) {
            in_right = current_frame_.IsInFrustum(map_point, 0.5f, true);
        }
        if (in_left || in_right) {
            map_point->IncreaseVisible();
            ++n_to_match;
        }
    }

    if (n_to_match <= 0) {
        return;
    }

    float th = 1.f;
    if (options_.sensor == Sensor::kRgbd ||
        options_.sensor == Sensor::kImuRgbd) {
        th = 3.f;
    }
    if (map_ && map_->isImuInitialized()) {
        th = map_->GetInertialBA2() ? 2.f : 6.f;
    } else if (IsImuSensor() && map_ && !map_->isImuInitialized()) {
        th = 10.f;
    }
    if (current_frame_.id < last_reloc_frame_id_ + 2) {
        th = 5.f;
    }
    if (state_ == State::kLost || state_ == State::kRecentlyLost) {
        th = 15.f;
    }

    feature::OrbMatcher matcher(0.8f, true);
    const bool far = local_mapping_ && local_mapping_->far_points();
    const float th_far =
        local_mapping_ ? local_mapping_->th_far_points() : 50.f;
    matcher.SearchByProjection(current_frame_, local_map_points_, th, far,
                               th_far);
}

bool Tracker::NeedNewKeyFrame() const {
    if (state_ != State::kOk &&
        !(insert_kfs_when_lost_ && state_ == State::kRecentlyLost)) {
        return false;
    }
    if (!last_keyframe_) {
        return false;
    }
    if (only_tracking_) {
        return false;
    }
    if (local_mapping_ &&
        (local_mapping_->isStopped() || local_mapping_->stopRequested())) {
        return false;
    }

    // Before IMU init: insert KF every ~0.25s (ORB).
    if (IsImuSensor() && map_ && !map_->isImuInitialized()) {
        return (current_frame_.timestamp - last_keyframe_->timestamp) >= 0.25;
    }

    const int n_kfs =
        map_ ? static_cast<int>(map_->KeyFramesInMap()) : 0;
    if (current_frame_.id < last_reloc_frame_id_ + max_frames_ &&
        n_kfs > max_frames_) {
        return false;
    }

    int min_obs = (n_kfs <= 2) ? 2 : 3;
    const int n_ref_matches =
        reference_keyframe_ ? reference_keyframe_->TrackedMapPoints(min_obs)
                            : 0;
    const bool local_idle =
        !local_mapping_ || local_mapping_->AcceptKeyFrames() ||
        local_mapping_->IsInitializing();

    int tracked_close = 0;
    int non_tracked_close = 0;
    if (!IsMonocularSensor()) {
        for (int i = 0; i < current_frame_.num_keypoints; ++i) {
            if (current_frame_.depths[i] <= 0.f ||
                current_frame_.depths[i] >= current_frame_.depth_threshold) {
                continue;
            }
            if (current_frame_.map_points[i] && !current_frame_.outliers[i]) {
                ++tracked_close;
            } else {
                ++non_tracked_close;
            }
        }
    }
    const bool need_close =
        (tracked_close < 100) && (non_tracked_close > 70);

    float th_ref = 0.75f;
    if (n_kfs < 2) {
        th_ref = 0.4f;
    }
    if (options_.sensor == Sensor::kMonocular) {
        th_ref = 0.9f;
    }
    // ORB: fisheye stereo (mpCamera2) keeps thRefRatio = 0.75.
    if (camera2_) {
        th_ref = 0.75f;
    }
    if (options_.sensor == Sensor::kImuMonocular) {
        th_ref = (matches_inliers_ > 350) ? 0.75f : 0.90f;
    }

    const bool c1a =
        current_frame_.id >= last_keyframe_frame_id_ + max_frames_;
    const bool c1b =
        (current_frame_.id >= last_keyframe_frame_id_ + min_frames_) &&
        local_idle;
    const bool c1c =
        !IsMonocularSensor() && !IsImuSensor() &&
        (matches_inliers_ < n_ref_matches * 0.25 || need_close);
    const bool c2 =
        ((matches_inliers_ < n_ref_matches * th_ref) || need_close) &&
        matches_inliers_ > 15;

    bool c3 = false;
    if (IsImuSensor() &&
        (current_frame_.timestamp - last_keyframe_->timestamp) >= 0.5) {
        c3 = true;
    }
    const bool c4 =
        options_.sensor == Sensor::kImuMonocular &&
        (((matches_inliers_ < 75) && (matches_inliers_ > 15)) ||
         state_ == State::kRecentlyLost);

    if (!(((c1a || c1b || c1c) && c2) || c3 || c4)) {
        return false;
    }
    if (local_idle ||
        (local_mapping_ && local_mapping_->IsInitializing())) {
        return true;
    }
    if (!IsMonocularSensor() && local_mapping_ &&
        local_mapping_->KeyframesInQueue() < 3) {
        return true;
    }
    return false;
}

void Tracker::UpdateLastFrame() {
    // Clear previous localization-mode temporal points.
    temporal_points_.clear();

    if (last_keyframe_frame_id_ == last_frame_.id || IsMonocularSensor() ||
        !only_tracking_) {
        return;
    }

    // VO temporal MPs: left features only when dual (ORB UpdateLastFrame).
    const int n_feat = last_frame_.HasDualCameraIndex() ? last_frame_.num_left
                                                        : last_frame_.num_keypoints;
    std::vector<std::pair<float, int>> depth_idx;
    depth_idx.reserve(static_cast<size_t>(std::max(0, n_feat)));
    for (int i = 0; i < n_feat; ++i) {
        if (i < static_cast<int>(last_frame_.depths.size()) &&
            last_frame_.depths[static_cast<size_t>(i)] > 0.f) {
            depth_idx.emplace_back(last_frame_.depths[static_cast<size_t>(i)], i);
        }
    }
    if (depth_idx.empty()) {
        return;
    }
    std::sort(depth_idx.begin(), depth_idx.end());

    int n_points = 0;
    for (const auto& [z, i] : depth_idx) {
        bool create = false;
        auto& map_point = last_frame_.map_points[static_cast<size_t>(i)];
        if (!map_point) {
            create = true;
        } else if (map_point->Observations() < 1) {
            create = true;
        }
        if (create) {
            Vec3 x3d;
            if (!last_frame_.UnprojectStereo(i, &x3d)) {
                continue;
            }
            auto p_new =
                std::make_shared<MapPoint>(x3d, last_keyframe_, map_.get());
            last_frame_.map_points[static_cast<size_t>(i)] = p_new;
            temporal_points_.push_back(p_new);
            ++n_points;
        } else {
            ++n_points;
        }
        if (z > last_frame_.depth_threshold && n_points > 100) {
            break;
        }
    }
}

void Tracker::UpdateFrameIMU(float scale, const sensor::imu::Bias& bias,
                             const std::shared_ptr<KeyFrame>& current_keyframe) {
    (void)scale;
    last_bias_ = bias;
    if (current_keyframe) {
        last_keyframe_ = current_keyframe;
    }
    last_frame_.imu_bias = last_bias_;
    current_frame_.imu_bias = last_bias_;
    map_updated_for_inertial_ = true;

    const Vec3 g(0.0, 0.0, -sensor::imu::kGravity);
    auto PropagateFromKeyFrame =
        [&](Frame* frame,
            const std::shared_ptr<sensor::imu::Preintegrator>& pre) {
            if (!frame || !last_keyframe_ || !pre || pre->delta_t < 1e-4) {
                return;
            }
            const SE3 Twb1 = last_keyframe_->GetImuPose();
            const Mat33 Rwb1 = Twb1.rotation();
            const Vec3 twb1 = Twb1.translation();
            const Vec3 vwb1 = last_keyframe_->has_velocity
                                  ? last_keyframe_->velocity_world
                                  : Vec3::Zero();
            const double t12 = pre->delta_t;
            SE3 Twb = SE3Identity();
            Twb.linear() = Rwb1 * pre->GetUpdatedDeltaRotation();
            Twb.translation() = twb1 + vwb1 * t12 + 0.5 * g * t12 * t12 +
                                Rwb1 * pre->GetUpdatedDeltaPosition();
            frame->SetImuPose(Twb);
            frame->velocity_world =
                vwb1 + g * t12 + Rwb1 * pre->GetUpdatedDeltaVelocity();
            frame->has_velocity = true;
        };

    if (last_frame_.has_pose()) {
        if (last_keyframe_ && last_frame_.id == last_keyframe_->frame_id) {
            last_frame_.SetImuPose(last_keyframe_->GetImuPose());
            if (last_keyframe_->has_velocity) {
                last_frame_.velocity_world = last_keyframe_->velocity_world;
                last_frame_.has_velocity = true;
            }
        } else {
            PropagateFromKeyFrame(&last_frame_,
                                  imu_preintegrated_from_last_kf_);
        }
    }
    if (current_frame_.has_pose() && imu_preintegrated_from_last_kf_) {
        PropagateFromKeyFrame(&current_frame_,
                              imu_preintegrated_from_last_kf_);
    }
}

void Tracker::CreateNewKeyFrame() {
    if (local_mapping_ && local_mapping_->IsInitializing() && map_ &&
        !map_->isImuInitialized()) {
        return;
    }
    if (local_mapping_ && !local_mapping_->SetNotStop(true)) {
        return;
    }

    auto keyframe = std::make_shared<KeyFrame>(current_frame_, map_.get());
    map_->AddKeyFrame(keyframe);

    if (IsImuSensor() && imu_preintegrated_from_last_kf_) {
        keyframe->imu_preintegrated = imu_preintegrated_from_last_kf_;
        keyframe->imu_bias = last_bias_;
        keyframe->imu_ready = map_ && map_->isImuInitialized();
        if (current_frame_.has_velocity) {
            keyframe->velocity_world = current_frame_.velocity_world;
            keyframe->has_velocity = true;
        }
        if (last_keyframe_) {
            keyframe->previous_keyframe = last_keyframe_;
            last_keyframe_->next_keyframe = keyframe;
        }
        EnsureImuCalib();
        imu_preintegrated_from_last_kf_ =
            std::make_shared<sensor::imu::Preintegrator>(last_bias_,
                                                         imu_calib_);
        map_updated_for_inertial_ = true;
    }

    // Bind already-tracked MapPoints (observations); new stereo close MPs below.
    const int n_total = current_frame_.TotalFeatures();
    for (int i = 0; i < n_total; ++i) {
        auto map_point = current_frame_.map_points[static_cast<size_t>(i)];
        if (!map_point || map_point->isBad()) {
            continue;
        }
        keyframe->AddMapPoint(map_point, i);
        map_point->AddObservation(keyframe, i);
    }

    // Stereo / RGB-D: create close MapPoints (depth < thDepth, fill to 100).
    if (!IsMonocularSensor()) {
        const int n_feat = current_frame_.HasDualCameraIndex()
                               ? current_frame_.num_left
                               : current_frame_.num_keypoints;
        constexpr int kMaxClosePoints = 100;
        std::vector<std::pair<float, int>> depth_idx;
        depth_idx.reserve(static_cast<size_t>(std::max(0, n_feat)));
        for (int i = 0; i < n_feat; ++i) {
            if (i < static_cast<int>(current_frame_.depths.size()) &&
                current_frame_.depths[static_cast<size_t>(i)] > 0.f) {
                depth_idx.emplace_back(
                    current_frame_.depths[static_cast<size_t>(i)], i);
            }
        }
        if (!depth_idx.empty()) {
            std::sort(depth_idx.begin(), depth_idx.end());
            int n_points = 0;
            for (const auto& [z, i] : depth_idx) {
                bool create = false;
                auto map_point =
                    current_frame_.map_points[static_cast<size_t>(i)];
                if (!map_point) {
                    create = true;
                } else if (map_point->Observations() < 1) {
                    create = true;
                    current_frame_.map_points[static_cast<size_t>(i)].reset();
                    map_point.reset();
                }
                if (create) {
                    Vec3 point_world;
                    if (!current_frame_.UnprojectStereo(i, &point_world)) {
                        continue;
                    }
                    map_point = std::make_shared<MapPoint>(point_world, keyframe,
                                                           map_.get());
                    map_point->AddObservation(keyframe, i);
                    keyframe->AddMapPoint(map_point, i);
                    LinkFisheyeRightObservation(&current_frame_, keyframe,
                                               map_point, i);
                    map_point->ComputeDistinctiveDescriptors();
                    map_point->UpdateNormalAndDepth();
                    map_->AddMapPoint(map_point);
                    current_frame_.map_points[static_cast<size_t>(i)] =
                        map_point;
                    ++n_points;
                } else {
                    ++n_points;
                }
                if (z > current_frame_.depth_threshold &&
                    n_points > kMaxClosePoints) {
                    break;
                }
            }
        }
    }

    last_keyframe_ = keyframe;
    reference_keyframe_ = keyframe;
    last_keyframe_frame_id_ = current_frame_.id;
    pending_system_keyframe_ = MakeSystemKeyframe(keyframe);

    RegisterKeyFrame(keyframe);

    if (local_mapping_) {
        local_mapping_->InterruptBA();
        local_mapping_->InsertKeyFrame(keyframe);
        local_mapping_->SetNotStop(false);
    }
}

bool Tracker::Relocalization() {
    if (!keyframe_database_ || !current_frame_.HasBoW()) {
        return false;
    }
    const auto candidates =
        keyframe_database_->DetectRelocalizationCandidates(current_frame_);
    if (candidates.empty()) {
        return false;
    }

    feature::OrbMatcher matcher(0.75f, true);
    feature::OrbMatcher matcher_proj(0.9f, true);
    const int n_feat = current_frame_.TotalFeatures();

    for (const auto& candidate : candidates) {
        if (!candidate || candidate->isBad()) {
            continue;
        }
        std::vector<std::shared_ptr<MapPoint>> matches;
        const int nmatches =
            matcher.SearchByBoW(candidate, current_frame_, &matches);
        if (nmatches < 15) {
            continue;
        }

        // Bearing PnP RANSAC (ORB-SLAM3 MLPnPsolver role) then PoseOpt.
        frontend::MlpnpSolver solver(current_frame_, matches);
        solver.SetRansacParameters(0.99, 10, 300, 5.991f);
        SE3 Tcw;
        std::vector<bool> inliers;
        int num_inliers = 0;
        if (!solver.Find(&Tcw, &inliers, &num_inliers) || num_inliers < 10) {
            continue;
        }
        current_frame_.map_points.assign(static_cast<size_t>(n_feat), nullptr);
        if (static_cast<int>(current_frame_.outliers.size()) < n_feat) {
            current_frame_.outliers.assign(static_cast<size_t>(n_feat), false);
        }
        std::set<std::shared_ptr<MapPoint>> found;
        const auto& corr = solver.indices();
        for (size_t i = 0; i < inliers.size() && i < corr.size(); ++i) {
            if (!inliers[i]) {
                continue;
            }
            const int fi = corr[i];
            if (fi < 0 || fi >= n_feat ||
                fi >= static_cast<int>(matches.size()) || !matches[static_cast<size_t>(fi)]) {
                continue;
            }
            current_frame_.map_points[static_cast<size_t>(fi)] =
                matches[static_cast<size_t>(fi)];
            found.insert(matches[static_cast<size_t>(fi)]);
        }
        current_frame_.SetPose(Tcw);
        int n_good = OptimizeCurrentPose();

        // Clear outliers (ORB Relocalization).
        for (int i = 0; i < n_feat; ++i) {
            if (i < static_cast<int>(current_frame_.outliers.size()) &&
                current_frame_.outliers[static_cast<size_t>(i)]) {
                current_frame_.map_points[static_cast<size_t>(i)].reset();
                current_frame_.outliers[static_cast<size_t>(i)] = false;
            }
        }
        if (n_good < 10) {
            continue;
        }

        // Coarse then fine projection search when few inliers (ORB).
        if (n_good < 50) {
            int n_add = matcher_proj.SearchByProjection(
                current_frame_, candidate, found, 10.f, 100);
            if (n_add + n_good >= 50) {
                n_good = OptimizeCurrentPose();
                for (int i = 0; i < n_feat; ++i) {
                    if (current_frame_.map_points[static_cast<size_t>(i)]) {
                        found.insert(
                            current_frame_.map_points[static_cast<size_t>(i)]);
                    }
                }
                if (n_good > 30 && n_good < 50) {
                    n_add = matcher_proj.SearchByProjection(
                        current_frame_, candidate, found, 3.f, 64);
                    if (n_good + n_add >= 50) {
                        n_good = OptimizeCurrentPose();
                    }
                }
            }
        }

        int nmatches_map = 0;
        for (int i = 0; i < n_feat; ++i) {
            if (!current_frame_.map_points[static_cast<size_t>(i)]) {
                continue;
            }
            if (i < static_cast<int>(current_frame_.outliers.size()) &&
                current_frame_.outliers[static_cast<size_t>(i)]) {
                current_frame_.map_points[static_cast<size_t>(i)].reset();
                current_frame_.outliers[static_cast<size_t>(i)] = false;
            } else {
                ++nmatches_map;
            }
        }
        if (nmatches_map >= 50 || (n_good >= 50)) {
            reference_keyframe_ = candidate;
            last_keyframe_ = candidate;
            matches_inliers_ = nmatches_map;
            last_reloc_frame_id_ = current_frame_.id;
            return true;
        }
        // Soft accept near ORB's ≥50 after proj; keep prior ≥20 for sparse maps.
        if (nmatches_map >= 20 && n_good >= 20) {
            reference_keyframe_ = candidate;
            last_keyframe_ = candidate;
            matches_inliers_ = nmatches_map;
            last_reloc_frame_id_ = current_frame_.id;
            return true;
        }
    }
    return false;
}

Keyframe Tracker::MakeSystemKeyframe(
    const std::shared_ptr<KeyFrame>& keyframe) const {
    Keyframe system_keyframe;
    if (!keyframe) {
        return system_keyframe;
    }
    system_keyframe.id = static_cast<int>(keyframe->id);
    system_keyframe.timestamp =
        static_cast<TimeStamp>(keyframe->timestamp * 1e9);
    system_keyframe.pose_world_body = keyframe->GetPose().inverse();
    for (const auto& map_point : keyframe->GetMapPoints()) {
        if (!map_point || map_point->isBad()) {
            continue;
        }
        Landmark landmark;
        landmark.id = static_cast<int>(map_point->id);
        landmark.position = map_point->GetWorldPos();
        system_keyframe.landmarks.push_back(landmark);
    }
    return system_keyframe;
}

void Tracker::PublishResult(bool valid) {
    last_result_ = OdometryResult{};
    last_result_.timestamp =
        static_cast<TimeStamp>(current_frame_.timestamp * 1e9);
    if (current_frame_.has_pose()) {
        last_result_.pose_world_body = current_frame_.PoseWorldBody();
    }
    last_result_.valid = valid;
    if (current_frame_.has_velocity) {
        last_result_.velocity = current_frame_.velocity_world;
    }
    last_result_.gyro_bias = current_frame_.imu_bias.gyroscope;
    last_result_.accel_bias = current_frame_.imu_bias.accelerometer;

    last_result_.landmarks.clear();
    last_result_.landmarks.reserve(
        static_cast<size_t>(current_frame_.num_keypoints));
    for (int i = 0; i < current_frame_.num_keypoints; ++i) {
        const auto& mp = current_frame_.map_points[i];
        if (!mp || mp->isBad() || current_frame_.outliers[i]) {
            continue;
        }
        Landmark lm;
        lm.id = static_cast<int>(mp->id);
        lm.position = mp->GetWorldPos();
        if (i < static_cast<int>(current_frame_.keypoints_undistorted.size())) {
            lm.uv = Vec2(current_frame_.keypoints_undistorted[i].pt.x,
                         current_frame_.keypoints_undistorted[i].pt.y);
            lm.has_uv = true;
        }
        last_result_.landmarks.push_back(lm);
    }

    last_result_.local_map.clear();
    last_result_.local_map.reserve(local_map_points_.size());
    for (const auto& mp : local_map_points_) {
        if (!mp || mp->isBad()) {
            continue;
        }
        const Vec3 p = mp->GetWorldPos();
        PointXYZI pt;
        pt.x = static_cast<float>(p.x());
        pt.y = static_cast<float>(p.y());
        pt.z = static_cast<float>(p.z());
        pt.intensity = 1.f;
        pt.timestamp = current_frame_.timestamp;
        last_result_.local_map.push_back(pt);
    }
}

bool Tracker::GetResult(OdometryResult* out) const {
    if (out == nullptr) {
        return false;
    }
    *out = last_result_;
    return last_result_.valid;
}

bool Tracker::ConsumePendingKeyframe(Keyframe* out) {
    if (out == nullptr || !pending_system_keyframe_.has_value()) {
        return false;
    }
    *out = *pending_system_keyframe_;
    pending_system_keyframe_.reset();
    return true;
}

}  // namespace tracking
}  // namespace atlas
}  // namespace localization
}  // namespace autonomy
