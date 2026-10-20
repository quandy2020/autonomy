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

#include "argus/odometry/livo_odometry.hpp"

#include <cmath>
#include <iostream>
#include <utility>
#include <vector>

#include "argus/backend/state_filter/esikf_state_filter.hpp"
#include "argus/backend/state_filter/esikf_update_options.hpp"
#include "argus/common/camera/camera_loader.hpp"
#include "argus/common/logger.hpp"
#include "argus/common/math/so3.hpp"
#include "argus/common/math/transform/transform.hpp"
#include "argus/frontend/semantic_dynamic_filter.hpp"
#include "argus/sensor/image_data.hpp"

namespace argus {
namespace odometry {

LivoOdometry::LivoOdometry() : LivoOdometry(LivoOdometryConfig{}) {}

LivoOdometry::LivoOdometry(const LivoOdometryConfig& config) {
    SetConfig(config);
    Initialize();
}

void LivoOdometry::SetConfig(const LivoOdometryConfig& config) {
    config_ = config;
    health_monitor_.SetTimeoutSec(config_.sensor_timeout_sec);
    if (initialized_) {
        lidar_dynamic_filter_ =
            frontend::CreateLidarDynamicFilter(config_.dynamic_filter);
        visual_dynamic_filter_ =
            frontend::CreateVisualDynamicFilter(config_.dynamic_filter);
        if (loop_closure_manager_) {
            loop_closure_manager_->Stop();
        }
        loop_closure_manager_ = std::make_unique<loop_closure::LoopClosureManager>(
            config_.loop_closure);
        loop_closure_manager_->Start();
        if (map_cleaning_) {
            map_cleaning_->Stop();
        }
        map_cleaning_ =
            std::make_unique<map::MapSelfCleaning>(config_.map_cleaning);
        map_cleaning_->Start();
        if (relocalization_manager_) {
            relocalization_manager_->Stop();
        }
        relocalization_manager_ = std::make_unique<relocalization::RelocalizationManager>(
            config_.relocalization);
        relocalization_manager_->Start();
        cross_session_reloc_requested_ = false;
        if (!config_.relocalization.prior_map_path.empty()) {
            LoadPriorMap(config_.relocalization.prior_map_path);
        }
        if (config_.enable_mapping) {
            map_builder_ =
                std::make_unique<mapping::MapBuilder>(config_.mapping);
        } else {
            map_builder_.reset();
        }
    }
}

void LivoOdometry::InitCoreModules() {
    preprocess_ = frontend::Preprocess::make_shared();
    imu_process_ = frontend::ImuProcess::make_shared();

    map::VoxelMapConfig voxel_cfg = config_.voxel;
    voxel_manager_ = map::VoxelMapManager::make_shared(voxel_cfg, voxel_hash_);
    voxel_manager_->config_setting_ = config_.voxel;
    voxel_manager_->extR_ = config_.ext_R;
    voxel_manager_->extT_ = config_.ext_T;
    lio_frame_.EnsureDefaultClouds();
    voxel_manager_->BindFrameBuffers(&lio_frame_);

    sync_.SetSlamMode(config_.slam_mode);
    sync_.SetLidarEnabled(config_.enable_lidar);
    sync_.SetImuEnabled(config_.enable_imu);
    sync_.SetImageEnabled(config_.enable_image);
    sync_.SetExposureTimeInit(config_.exposure_time_init);

    downsample_.SetLeafSize(static_cast<float>(config_.surface_voxel_size),
                            static_cast<float>(config_.surface_voxel_size),
                            static_cast<float>(config_.surface_voxel_size));

    lidar_dynamic_filter_ =
        frontend::CreateLidarDynamicFilter(config_.dynamic_filter);
    visual_dynamic_filter_ =
        frontend::CreateVisualDynamicFilter(config_.dynamic_filter);
    health_monitor_.SetTimeoutSec(config_.sensor_timeout_sec);
}

void LivoOdometry::ConfigureImuFromConfig() {
    imu_process_->SetExtrinsic(config_.ext_T, config_.ext_R);
    imu_process_->SetGyrCovScale(
        Eigen::Vector3d(config_.gyr_cov, config_.gyr_cov, config_.gyr_cov));
    imu_process_->SetAccCovScale(
        Eigen::Vector3d(config_.acc_cov, config_.acc_cov, config_.acc_cov));
    imu_process_->SetInvExpoCov(config_.inv_expo_cov);
    imu_process_->SetGyrBiasCov(Eigen::Vector3d(1e-4, 1e-4, 1e-4));
    imu_process_->SetAccBiasCov(Eigen::Vector3d(1e-4, 1e-4, 1e-4));
    imu_process_->SetImuInitFrameNum(config_.imu_init_frames);
    imu_process_->SetMotionCompensate(config_.enable_motion_compensate);
    if (!config_.imu_log_dir.empty()) {
        imu_process_->SetLogDir(config_.imu_log_dir);
    }
    if (!config_.enable_imu) {
        imu_process_->DisableImu();
    }
    if (!config_.enable_gravity_estimate) {
        imu_process_->DisableGravityEstimate();
    }
    if (!config_.enable_bias_estimate) {
        imu_process_->DisableBiasEstimate();
    }
    if (!config_.enable_exposure_estimate) {
        imu_process_->DisableExposureEstimate();
    }
}

void LivoOdometry::RestartBackgroundWorkers() {
    if (loop_closure_manager_) {
        loop_closure_manager_->Stop();
    }
    loop_closure_manager_ = std::make_unique<loop_closure::LoopClosureManager>(
        config_.loop_closure);
    loop_closure_manager_->Start();
    if (map_cleaning_) {
        map_cleaning_->Stop();
    }
    map_cleaning_ =
        std::make_unique<map::MapSelfCleaning>(config_.map_cleaning);
    map_cleaning_->Start();
    if (relocalization_manager_) {
        relocalization_manager_->Stop();
    }
    relocalization_manager_ = std::make_unique<relocalization::RelocalizationManager>(
        config_.relocalization);
    relocalization_manager_->Start();
    next_keyframe_id_ = 0;
    has_keyframe_ = false;
    cross_session_reloc_requested_ = false;
    if (!config_.relocalization.prior_map_path.empty()) {
        LoadPriorMap(config_.relocalization.prior_map_path);
    }
    if (config_.enable_mapping) {
        map_builder_ = std::make_unique<mapping::MapBuilder>(config_.mapping);
    } else {
        map_builder_.reset();
    }
}

void LivoOdometry::InitVioIfEnabled() {
    if (config_.enable_image) {
        EnsureVioManager();
        if (!config_.vio.camera_calib_file.empty()) {
            InitializeVio();
        }
    } else {
        vio_manager_.reset();
        camera_.reset();
    }
}

void LivoOdometry::Initialize() {
    InitCoreModules();
    ConfigureImuFromConfig();
    RestartBackgroundWorkers();
    InitVioIfEnabled();
    initialized_ = true;
}

VIOManager::SharedPtr LivoOdometry::EnsureVioManager() {
    if (!vio_manager_) {
        vio_manager_ = VIOManager::make_shared();
    }
    return vio_manager_;
}

bool LivoOdometry::vio_ready() const {
    return vio_manager_ != nullptr && vio_manager_->cam != nullptr &&
           vio_manager_->width > 0;
}

void LivoOdometry::ApplyVioConfigToManager() {
    if (!vio_manager_) {
        return;
    }
    const VioConfig& v = config_.vio;
    vio_manager_->grid_size = v.grid_size;
    vio_manager_->grid_n_width = v.grid_n_width;
    vio_manager_->grid_n_height = v.grid_n_height;
    vio_manager_->patch_size = v.patch_size;
    vio_manager_->patch_pyramid_level = v.patch_pyramid_level;
    vio_manager_->max_iterations = v.max_iterations;
    vio_manager_->outlier_threshold = v.outlier_threshold;
    vio_manager_->image_point_cov = v.image_point_cov;
    vio_manager_->enable_normal = v.enable_normal;
    vio_manager_->enable_inverse_composition = v.enable_inverse_composition;
    vio_manager_->enable_raycast = v.enable_raycast;
    vio_manager_->enable_exposure_estimate = v.enable_exposure_estimate;
    vio_manager_->enable_colmap_output = v.enable_colmap_output;
    vio_manager_->log_dir = v.log_dir;
    vio_manager_->SetImuToLidarExtrinsic(config_.ext_T, config_.ext_R);
    std::vector<double> Rcl = v.Rcl;
    std::vector<double> Pcl = v.Pcl;
    if (Rcl.size() != 9) {
        Rcl = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    }
    if (Pcl.size() != 3) {
        Pcl = {0, 0, 0};
    }
    vio_manager_->SetLidarToCameraExtrinsic(Rcl, Pcl);
    vio_manager_->state = &state_;
    vio_manager_->state_propagated = &state_propagated_;
}

bool LivoOdometry::InitializeVio() {
    if (!config_.enable_image) {
        return false;
    }
    EnsureVioManager();
    if (vio_ready()) {
        return true;
    }
    if (config_.vio.camera_calib_file.empty()) {
        ARGUS_WARN << "[ VIO ]: camera_calib_file empty; skip InitializeVio";
        return false;
    }
    camera_ = camera::LoadFromYaml(config_.vio.camera_calib_file);
    if (!camera_) {
        ARGUS_WARN << "[ VIO ]: failed to load camera "
                   << config_.vio.camera_calib_file;
        return false;
    }
    vio_manager_->cam = camera_.get();
    ApplyVioConfigToManager();
    vio_manager_->InitializeVIO();
    return vio_ready();
}

bool LivoOdometry::LoadPriorMap(const std::string& path_or_dir) {
    if (!relocalization_manager_ || path_or_dir.empty()) {
        return false;
    }
    auto bundle = relocalization::PriorMapLoader::LoadPathBundle(
        path_or_dir, config_.relocalization.prior_resolution);
    if (!bundle.ok()) {
        return false;
    }
    // Prefer full .xyz for SC++ place index (better than voxel centers alone).
    relocalization_manager_->SetPriorMap(bundle);
    cross_session_reloc_requested_ = false;
    return true;
}

bool LivoOdometry::LoadBuiltMap(const std::string& directory) {
    if (directory.empty()) {
        return false;
    }
    if (!map_builder_) {
        map_builder_ = std::make_unique<mapping::MapBuilder>(config_.mapping);
    }
    if (!map_builder_->Load(directory)) {
        return false;
    }
    config_.mapping = map_builder_->config();
    config_.enable_mapping = true;
    config_.relocalization.prior_resolution = config_.mapping.prior_resolution;
    if (relocalization_manager_) {
        auto bundle = relocalization::PriorMapLoader::LoadPathBundle(
            directory, config_.relocalization.prior_resolution);
        if (bundle.ok()) {
            relocalization_manager_->SetPriorMap(bundle);
            cross_session_reloc_requested_ = false;
        } else if (auto prior = map_builder_->ExportPriorMap()) {
            relocalization_manager_->SetPriorMap(std::move(prior));
            cross_session_reloc_requested_ = false;
        }
    }
    if (loop_closure_manager_ && config_.loop_closure.enable) {
        const std::string sid = config_.mapping.session_id;
        const map::SessionMeta* meta = map_builder_->session().Get(sid);
        if (meta != nullptr) {
            // Package keyframes/ → SC + prior LTA buffer (body frame, global ids).
            const auto scan_context_keyframes =
                map_builder_->ExportLoopKeyFrames();
            loop_closure_manager_->RegisterPriorSession(
                *meta, map_builder_->session().Trajectory(sid),
                scan_context_keyframes);
            ARGUS_INFO << "[map] registered prior session " << sid
                       << " scan_context_keyframes="
                       << scan_context_keyframes.size()
                       << " prior_lta_clouds="
                       << loop_closure_manager_->prior_cloud_count();
        }
    }
    return true;
}

void LivoOdometry::AddLidar(const sensor::PointCloudXYZI::Ptr& cloud,
                            double header_time) {
    health_monitor_.RecordLidar(header_time);
    sync_.PushLidar(cloud, header_time);
}

void LivoOdometry::AddLidar(
    const ::automsgs::msgs::sensor_msgs::PointCloud2& msg, double header_time) {
    auto cloud = sensor::PointCloudXYZI::make_shared();
    auto msg_ptr =
        std::shared_ptr<const ::automsgs::msgs::sensor_msgs::PointCloud2>(
            &msg, [](const ::automsgs::msgs::sensor_msgs::PointCloud2*) {});
    preprocess_->Process(msg_ptr, cloud);
    AddLidar(cloud, header_time);
}

void LivoOdometry::AddImu(const sensor::ImuData& imu) {
    health_monitor_.RecordImu(imu.timestamp);
    sync_.PushImu(imu);
}

void LivoOdometry::AddImage(const sensor::ImageData& image) {
    health_monitor_.RecordCamera(image.timestamp);
    sync_.PushImage(image);
    if (!image.image.empty()) {
        last_image_ = image.image;
    }
    // Update semantic keep-mask early so ONLY_LO packages (no img in
    // MeasureGroup) still gate LiDAR when images are fed.
    if (auto* sem = dynamic_cast<frontend::SemanticDynamicFilter*>(
            lidar_dynamic_filter_.get())) {
        ConfigureSemanticProjection();
        if (!image.image.empty()) {
            sem->FilterVisual(image.image, state_);
        }
    }
}

void LivoOdometry::ConfigureSemanticProjection() {
    auto* sem = dynamic_cast<frontend::SemanticDynamicFilter*>(
        lidar_dynamic_filter_.get());
    if (sem == nullptr) {
        return;
    }
    Eigen::Matrix3d R_cl = Eigen::Matrix3d::Identity();
    Eigen::Vector3d t_cl = Eigen::Vector3d::Zero();
    const auto& R = config_.vio.Rcl;
    const auto& P = config_.vio.Pcl;
    if (R.size() == 9) {
        R_cl << R[0], R[1], R[2], R[3], R[4], R[5], R[6], R[7], R[8];
    }
    if (P.size() == 3) {
        t_cl << P[0], P[1], P[2];
    }
    sem->SetLidarCameraExtrinsic(R_cl, t_cl);
    if (camera_) {
        sem->SetCameraIntrinsics(camera_->fx(), camera_->fy(), camera_->cx(),
                                 camera_->cy(), camera_->width(),
                                 camera_->height());
    } else if (vio_manager_ && vio_manager_->cam) {
        sem->SetCameraIntrinsics(vio_manager_->cam->fx(), vio_manager_->cam->fy(),
                                 vio_manager_->cam->cx(), vio_manager_->cam->cy(),
                                 vio_manager_->cam->width(),
                                 vio_manager_->cam->height());
    }
}

bool LivoOdometry::Process(OdometryOutput* out) {
    if (!initialized_ || out == nullptr) {
        return false;
    }
    out->status = ProcessStatus::kOk;
    out->soft_skip = false;
    out->map_updated = false;
    if (!sync_.Sync(measures_)) {
        out->status = ProcessStatus::kNoPackage;
        return false;
    }

    if (first_lidar_time_ < 0 && !measures_.measures.empty()) {
        first_lidar_time_ = measures_.last_lio_update_time > 0
                                ? measures_.last_lio_update_time
                                : measures_.lidar_frame_beg_time;
        imu_process_->first_lidar_time = first_lidar_time_;
    }

    // IMU propagate for every package (LIO / LO / VIO), matching FAST-LIVO2.
    imu_process_->Process(measures_, state_, lio_frame_.feats_undistort);
    if (config_.enable_gravity_align && !gravity_align_finished_ &&
        !imu_process_->need_imu_init) {
        const Eigen::Vector3d ez(0, 0, -1);
        const Eigen::Vector3d gz(state_.gravity);
        const Eigen::Quaterniond G_q_I0 =
            Eigen::Quaterniond::FromTwoVectors(gz, ez);
        const Eigen::Matrix3d G_R_I0 = G_q_I0.toRotationMatrix();
        state_.pos_end = G_R_I0 * state_.pos_end;
        state_.rot_end = G_R_I0 * state_.rot_end;
        state_.vel_end = G_R_I0 * state_.vel_end;
        state_.gravity = G_R_I0 * state_.gravity;
        gravity_align_finished_ = true;
    }
    state_propagated_ = state_;
    voxel_manager_->state_ = state_;

    if (measures_.ekf_phase == frontend::kLio ||
        measures_.ekf_phase == frontend::kLo) {
        return ProcessLio(out);
    }

    if (measures_.ekf_phase == frontend::kVio) {
        return ProcessVio(out);
    }

    out->ekf_phase = measures_.ekf_phase;
    out->state = state_;
    out->timestamp = measures_.last_lio_update_time;
    out->undistorted = lio_frame_.feats_undistort;
    out->downsampled_body = lio_frame_.feats_down_body;
    out->map_updated = false;
    out->status = ProcessStatus::kPropagateOnly;
    out->soft_skip = true;
    return true;
}

bool LivoOdometry::ProcessVio(OdometryOutput* out) {
    if (!vio_ready()) {
        out->ekf_phase = frontend::kVio;
        out->state = state_;
        out->timestamp = measures_.last_lio_update_time;
        out->undistorted = lio_frame_.feats_undistort;
        out->downsampled_body = lio_frame_.feats_down_body;
        out->map_updated = false;
        out->status = ProcessStatus::kVioNotReady;
        out->soft_skip = true;
        return true;
    }

    if (measures_.measures.empty() || measures_.measures.back().image.empty()) {
        ARGUS_WARN << "[ VIO ]: Empty Image";
        out->ekf_phase = frontend::kVio;
        out->state = state_;
        out->timestamp = measures_.last_lio_update_time;
        out->undistorted = lio_frame_.feats_undistort;
        out->downsampled_body = lio_frame_.feats_down_body;
        out->map_updated = false;
        out->status = ProcessStatus::kEmptyImage;
        out->soft_skip = true;
        return true;
    }

    vio_manager_->state = &state_;
    vio_manager_->state_propagated = &state_propagated_;

    measures_.measures.back().image.copyTo(vio_image_work_);
    last_visual_dynamic_ratio_ = 0.0;
    if (visual_dynamic_filter_ && !vio_image_work_.empty()) {
        const frontend::Mask vmask =
            visual_dynamic_filter_->FilterVisual(vio_image_work_, state_);
        last_visual_dynamic_ratio_ = vmask.RejectedRatio();
        frontend::DynamicFilterBase::ApplyVisualMask(vmask, &vio_image_work_);
    }
    const double img_time =
        (first_lidar_time_ >= 0.0)
            ? (measures_.last_lio_update_time - first_lidar_time_)
            : 0.0;
    vio_manager_->ProcessFrame(vio_image_work_, lio_frame_.pv_list, voxel_hash_,
                           img_time);

    // ProcessFrame mutates *state via pointer; keep voxel manager in sync.
    voxel_manager_->state_ = state_;

    if (map_builder_ && config_.enable_mapping) {
        ImportVisualMap(vio_manager_->visual_map);
    }

    TryInjectRelocalization();
    TryInjectLoopCorrection();

    out->ekf_phase = frontend::kVio;
    out->state = state_;
    out->timestamp = measures_.last_lio_update_time;
    out->undistorted = lio_frame_.feats_undistort;
    out->downsampled_body = lio_frame_.feats_down_body;
    out->map_updated = true;
    out->status = ProcessStatus::kOk;
    out->soft_skip = false;
    return true;
}

transform::Rigid3d LivoOdometry::ReportedPose() const {
    const transform::Rigid3d T_esikf(
        state_.pos_end, Eigen::Quaterniond(state_.rot_end));
    if (!config_.synthetic_drift.enable) {
        return T_esikf;
    }
    return eval_drift_T_ * T_esikf;
}

void LivoOdometry::FillLioOutput(OdometryOutput* out, bool map_updated) {
    out->ekf_phase = measures_.ekf_phase;
    out->state = state_;
    if (config_.synthetic_drift.enable) {
        const transform::Rigid3d T = ReportedPose();
        out->state.pos_end = T.translation();
        out->state.rot_end = T.rotation().toRotationMatrix();
    }
    out->timestamp = measures_.last_lio_update_time;
    out->undistorted = lio_frame_.feats_undistort;
    out->downsampled_body = lio_frame_.feats_down_body;
    out->map_updated = map_updated;
}

bool LivoOdometry::ApplyLioDynamicFilter(OdometryOutput* out) {
    last_lidar_dynamic_ratio_ = 0.0;
    if (lidar_dynamic_filter_ && !lio_frame_.feats_down_body->empty()) {
        if (auto* sem = dynamic_cast<frontend::SemanticDynamicFilter*>(
                lidar_dynamic_filter_.get())) {
            ConfigureSemanticProjection();
            cv::Mat image;
            if (!measures_.measures.empty() &&
                !measures_.measures.back().image.empty()) {
                image = measures_.measures.back().image;
            } else {
                image = last_image_;
            }
            if (!image.empty() && !sem->has_keep_mask()) {
                sem->FilterVisual(image, state_);
            } else if (!image.empty() &&
                       config_.dynamic_filter.semantic_backend != "mask_file") {
                sem->FilterVisual(image, state_);
            }
        }
        const frontend::Mask mask = lidar_dynamic_filter_->FilterLidar(
            *lio_frame_.feats_down_body, state_);
        last_lidar_dynamic_ratio_ = mask.RejectedRatio();
        frontend::DynamicFilterBase::ApplyLidarMask(
            mask, lio_frame_.feats_down_body.get());
    }
    if (lio_frame_.feats_down_body->empty()) {
        ARGUS_WARN << "[ LIO ]: All points rejected by dynamic filter";
        last_observation_confidence_.lidar_valid = false;
        TryRequestRelocalization();
        TryInjectRelocalization();
        TryInjectLoopCorrection();
        FillLioOutput(out, false);
        out->status = ProcessStatus::kAllDynamicRejected;
        out->soft_skip = true;
        return false;
    }
    return true;
}

void LivoOdometry::SeedInitialVoxelMap() {
    if (lidar_map_inited_) {
        return;
    }
    auto& world_buf = lio_frame_.world_lidar;
    voxel_manager_->TransformLidar(state_.rot_end, state_.pos_end,
                                   lio_frame_.feats_down_body, world_buf);
    lio_frame_.feats_down_world->clear();
    lio_frame_.feats_down_world->resize(world_buf->size());
    for (std::size_t i = 0; i < world_buf->size(); ++i) {
        const auto& p = world_buf->points[i];
        sensor::PointType& q = lio_frame_.feats_down_world->points[i];
        q.x = p.x;
        q.y = p.y;
        q.z = p.z;
        q.intensity = p.intensity;
    }
    lidar_map_inited_ = true;
    voxel_manager_->BuildVoxelMap();
}

void LivoOdometry::BuildLioEsikfOptions(
    double now_sec, backend::EsikfUpdateOptions* options) {
    options->enable_degeneracy = config_.enable_degeneracy;
    options->degeneracy = config_.degeneracy;
    options->degeneracy_result = &last_degeneracy_;
    options->enable_gating = config_.enable_confidence_gating;
    options->gating = config_.confidence_gating;
    options->health = health_monitor_.Evaluate(now_sec);
    options->dynamic_ratio_lidar = last_lidar_dynamic_ratio_;
    options->dynamic_ratio_visual = last_visual_dynamic_ratio_;
    options->observation_confidence = &last_observation_confidence_;
    options->enable_intensity_assist = config_.enable_intensity_assist;
    options->intensity_assist = config_.intensity_assist;
    options->intensity_assist_result = &last_intensity_assist_;
}

void LivoOdometry::UpdateLioMapAfterEsikf(double now_sec) {
    auto& world_buf = lio_frame_.world_lidar;
    voxel_manager_->TransformLidar(state_.rot_end, state_.pos_end,
                                   lio_frame_.feats_down_body, world_buf);
    for (std::size_t i = 0;
         i < world_buf->points.size() && i < lio_frame_.pv_list.size(); ++i) {
        lio_frame_.pv_list[i].point_w << world_buf->points[i].x,
            world_buf->points[i].y, world_buf->points[i].z;
        const Eigen::Matrix3d point_crossmat = lio_frame_.cross_mat_list[i];
        Eigen::Matrix3d var = lio_frame_.body_cov_list[i];
        var = (state_.rot_end * config_.ext_R) * var *
                  (state_.rot_end * config_.ext_R).transpose() +
              (-point_crossmat) * state_.cov.block<3, 3>(0, 0) *
                  (-point_crossmat).transpose() +
              state_.cov.block<3, 3>(3, 3);
        lio_frame_.pv_list[i].var = var;
    }
    voxel_manager_->UpdateVoxelMap(lio_frame_.pv_list);
    if (map_cleaning_) {
        map_cleaning_->UpdateConfidence(*voxel_manager_, last_degeneracy_);
        map_cleaning_->MaybeRequestPrune(voxel_manager_.get(), now_sec);
    }
    if (voxel_manager_->config_setting_.map_sliding_en_) {
        voxel_manager_->MapSliding();
    }
    MaybeApplySyntheticDrift();
    TrySubmitKeyFrame();
}

void LivoOdometry::MaybeApplySyntheticDrift() {
    if (!config_.synthetic_drift.enable || synthetic_drift_frozen_) {
        return;
    }
    // Grow an external bias \(T_d\); do NOT touch ESIKF state_ (lidar would
    // cancel an in-filter bias within one update).
    if (!has_drift_anchor_) {
        drift_anchor_pos_ = state_.pos_end;
        has_drift_anchor_ = true;
        return;
    }
    const double step = std::max(0.1, config_.synthetic_drift.step_m);
    const double dist = (state_.pos_end - drift_anchor_pos_).norm();
    if (dist < step) {
        return;
    }
    const double yaw = config_.synthetic_drift.yaw_rad_per_m * dist;
    const double tx = config_.synthetic_drift.tx_m_per_m * dist;
    const transform::Rigid3d dT =
        transform::FromXyzRpy(tx, 0.0, 0.0, 0.0, 0.0, yaw);
    eval_drift_T_ = eval_drift_T_ * dT;
    drift_anchor_pos_ = state_.pos_end;
    synthetic_drift_distance_m_ += dist;
}

bool LivoOdometry::ProcessLio(OdometryOutput* out) {
    // IMU + gravity align already applied in Process().

    if (lio_frame_.feats_undistort->empty()) {
        ARGUS_WARN << "[ LIO ]: No point";
        FillLioOutput(out, false);
        out->status = ProcessStatus::kEmptyLidar;
        out->soft_skip = true;
        return true;
    }

    downsample_.SetInputCloud(lio_frame_.feats_undistort);
    downsample_.Filter(*lio_frame_.feats_down_body);

    // P1: dynamic filter before residual / ESIKF (ESIKF never sees dynamics).
    if (!ApplyLioDynamicFilter(out)) {
        return true;
    }

    lio_frame_.feats_down_size =
        static_cast<int>(lio_frame_.feats_down_body->points.size());

    // Initial map seed only: ESIKF owns subsequent world transforms via
    // world_lidar.
    SeedInitialVoxelMap();

    const double now_sec = measures_.last_lio_update_time > 0
                               ? measures_.last_lio_update_time
                               : measures_.lidar_frame_beg_time;
    backend::EsikfUpdateOptions options;
    BuildLioEsikfOptions(now_sec, &options);
    backend::EsikfStateFilter::Update(voxel_manager_.get(), &state_propagated_,
                                      options);
    state_ = voxel_manager_->state_;

    const bool lidar_applied = !config_.enable_confidence_gating ||
                               last_observation_confidence_.lidar_valid;
    if (lidar_applied) {
        UpdateLioMapAfterEsikf(now_sec);
    }

    TryRequestRelocalization();
    TryInjectRelocalization();
    TryInjectLoopCorrection();

    FillLioOutput(out, lidar_applied);
    out->status = ProcessStatus::kOk;
    out->soft_skip = false;
    return true;
}

void LivoOdometry::TryRequestRelocalization() {
    if (!relocalization_manager_ || !config_.relocalization.enable) {
        return;
    }
    TryRequestCrossSessionRelocalization();

    if (config_.relocalization.trigger != "tracking_lost") {
        return;
    }
    const double now_sec = measures_.last_lio_update_time > 0
                               ? measures_.last_lio_update_time
                               : measures_.lidar_frame_beg_time;
    const perception::SensorHealth health = health_monitor_.Evaluate(now_sec);

    relocalization::TrackingLostSignals signals;
    signals.enable_gating = config_.enable_confidence_gating;
    signals.lidar_valid = last_observation_confidence_.lidar_valid;
    signals.lidar_healthy = health.lidar_healthy;
    signals.enable_degeneracy = config_.enable_degeneracy;
    signals.is_degenerated = last_degeneracy_.is_degenerated;
    signals.cloud_empty =
        (lio_frame_.feats_down_body == nullptr || lio_frame_.feats_down_body->empty());

    const bool lost = relocalization::IsTrackingLost(signals);

    // Prefer downsampled body; fall back to undistorted if filter emptied
    // cloud.
    const sensor::PointCloudXYZI* query = nullptr;
    if (lio_frame_.feats_down_body && !lio_frame_.feats_down_body->empty()) {
        query = lio_frame_.feats_down_body.get();
    } else if (lio_frame_.feats_undistort && !lio_frame_.feats_undistort->empty()) {
        query = lio_frame_.feats_undistort.get();
    }
    if (query == nullptr) {
        relocalization_manager_->TryAutoTrigger(lost, now_sec, {});
        return;
    }

    relocalization::RelocRequest req;
    req.query_cloud = *query;
    if (config_.relocalization.seed_with_current_pose) {
        req.has_initial_guess = true;
        req.initial_guess = transform::Rigid3d(
            state_.pos_end, Eigen::Quaterniond(state_.rot_end));
    }
    relocalization_manager_->TryAutoTrigger(lost, now_sec, std::move(req));
}

void LivoOdometry::TryRequestCrossSessionRelocalization() {
    if (cross_session_reloc_requested_) {
        return;
    }
    if (config_.relocalization.trigger != "cross_session_load") {
        return;
    }
    if (!relocalization_manager_->HasPriorMap()) {
        return;
    }
    const sensor::PointCloudXYZI* query = nullptr;
    if (lio_frame_.feats_down_body && !lio_frame_.feats_down_body->empty()) {
        query = lio_frame_.feats_down_body.get();
    } else if (lio_frame_.feats_undistort && !lio_frame_.feats_undistort->empty()) {
        query = lio_frame_.feats_undistort.get();
    }
    if (query == nullptr) {
        return;
    }
    relocalization::RelocRequest req;
    req.query_cloud = *query;
    if (config_.relocalization.seed_with_current_pose) {
        req.has_initial_guess = true;
        req.initial_guess = transform::Rigid3d(
            state_.pos_end, Eigen::Quaterniond(state_.rot_end));
    }
    if (relocalization_manager_->RequestRelocalization(std::move(req))) {
        cross_session_reloc_requested_ = true;
    }
}

void LivoOdometry::TryInjectRelocalization() {
    if (!relocalization_manager_ || !config_.relocalization.enable) {
        return;
    }
    // Atomic inject on the estimate thread only (BnB already finished on pool).
    for (const relocalization::RelocResult& r : relocalization_manager_->TakeResults()) {
        if (!r.success) {
            continue;
        }
        state_.pos_end = r.best.T_w_b.translation();
        state_.rot_end = r.best.T_w_b.rotation().toRotationMatrix();
        state_propagated_ = state_;
        // Inflate position covariance after global relocalization.
        state_.cov.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 0.25;
    }
}

void LivoOdometry::TryInjectLoopCorrection() {
    if (!loop_closure_manager_ || !config_.loop_closure.enable) {
        return;
    }
    if (config_.loop_closure.enable_pose_inject) {
        bool jumped = false;
        for (const loop_closure::LoopCorrection& c :
             loop_closure_manager_->TakeCorrections()) {
            if (c.apply_to_state) {
                // Apply in reported frame (still includes \(T_d\) if any).
                transform::Rigid3d T = ReportedPose();
                T = c.delta_w * T;
                state_.rot_end = T.rotation().toRotationMatrix();
                state_.pos_end = T.translation();
                state_propagated_ = state_;
                state_.cov.block<3, 3>(3, 3) +=
                    Eigen::Matrix3d::Identity() * 0.05;
                jumped = true;
            }
            // Verified recovery: stop growing \(T_d\). If we skipped state yank,
            // absorb current \(T_d\) into ESIKF so the logged trajectory stays
            // continuous (Umeyama then removes the residual rigid offset).
            if (config_.synthetic_drift.enable && c.clear_eval_drift) {
                if (!c.apply_to_state) {
                    const transform::Rigid3d T_rep = ReportedPose();
                    state_.pos_end = T_rep.translation();
                    state_.rot_end = T_rep.rotation().toRotationMatrix();
                    state_propagated_ = state_;
                }
                eval_drift_T_ = transform::Rigid3d::Identity();
                synthetic_drift_frozen_ = true;
            }
        }
        if (jumped) {
            loop_closure_manager_->NotifyPoseJump();
        }
    }

    // LTA: drain corrected history points into mapping ikd (estimate thread).
    if (config_.loop_closure.lta.enable && map_builder_) {
        sensor::PointCloudXYZI chunk;
        const std::size_t n = loop_closure_manager_->map_reload().Drain(
            &chunk, config_.loop_closure.lta.points_per_frame);
        if (n > 0) {
            map_builder_->ikd().AddPoints(chunk);
        }
    }
}

bool LivoOdometry::SaveBuiltMap(const std::string& directory) const {
    if (!map_builder_) {
        return false;
    }
    const std::string dir =
        directory.empty() ? config_.mapping_output_dir : directory;
    if (dir.empty()) {
        return false;
    }
    return map_builder_->Save(dir);
}

void LivoOdometry::ImportVisualMap(const map::VisualMap& visual) {
    if (!map_builder_ || !config_.enable_mapping) {
        return;
    }
    map_builder_->ImportVisualLandmarks(visual);
}

void LivoOdometry::TrySubmitKeyFrame() {
    const bool need_loop = loop_closure_manager_ && config_.loop_closure.enable;
    const bool need_map = map_builder_ && config_.enable_mapping;
    if ((!need_loop && !need_map) || lio_frame_.feats_down_body == nullptr ||
        lio_frame_.feats_down_body->empty()) {
        return;
    }
    const transform::Rigid3d T = ReportedPose();

    const double kf_dist = config_.loop_closure.keyframe_distance_m;
    const double kf_yaw = config_.loop_closure.keyframe_yaw_rad;
    if (has_keyframe_) {
        const transform::Rigid3d dT = last_keyframe_T_.inverse() * T;
        const double dist = dT.translation().norm();
        const double yaw = transform::GetYaw(dT);
        if (dist < kf_dist && std::abs(yaw) < kf_yaw) {
            return;
        }
    }

    const int kid = next_keyframe_id_++;
    if (need_loop) {
        loop_closure::KeyFrame kf;
        kf.id = kid;
        kf.stamp_sec = measures_.last_lio_update_time;
        kf.T_w_b = T;
        kf.cloud = *lio_frame_.feats_down_body;
        kf.session_id = config_.loop_closure.current_session_id;
        kf.session_local_id = kf.id;
        // Non-blocking enqueue — detection / Optimize / cross-session on async
        // worker.
        loop_closure_manager_->SubmitKeyFrame(std::move(kf));
    }
    if (need_map) {
        mapping::MapBuilderKeyFrame mk;
        mk.local_id = kid;
        mk.stamp_sec = measures_.last_lio_update_time;
        mk.T_w_b = T;
        mk.cloud = *lio_frame_.feats_down_body;
        mk.cloud_in_world = false;
        map_builder_->AddKeyFrame(mk);
    }
    last_keyframe_T_ = T;
    has_keyframe_ = true;
}

}  // namespace odometry
}  // namespace argus
