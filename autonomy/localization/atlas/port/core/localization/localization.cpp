#include <pcl/common/transforms.h>

#include "autonomy/localization/atlas/port/core/localization/lidar_loc/lidar_loc.hpp"
#include "autonomy/localization/atlas/port/core/localization/localization.hpp"
#include "autonomy/localization/atlas/port/core/localization/pose_graph/pgo.hpp"
#include "autonomy/localization/atlas/port/io/yaml_io.hpp"

#include <glog/logging.h>

namespace atlas_lio::loc {

Localization::Localization(Options options) { options_ = options; }

bool Localization::Init(const std::string& yaml_path, const std::string& global_map_path) {
    UL lock(global_mutex_);
    if (lidar_loc_ != nullptr) {
        Finish();
    }

    YAML_IO yaml(yaml_path);

    LaserMapping::Options opt_lio;
    opt_lio.is_in_slam_mode_ = false;

    lio_ = std::make_shared<LaserMapping>(opt_lio);
    if (!lio_->Init(yaml_path)) {
        LOG(ERROR) << "failed to init lio";
        return false;
    }

    LidarLoc::Options lidar_loc_options;
    lidar_loc_options.update_dynamic_cloud_ =
        yaml.GetValue<bool>("lidar_loc", "update_dynamic_cloud");
    lidar_loc_options.force_2d_ = yaml.GetValue<bool>("lidar_loc", "force_2d");
    lidar_loc_options.map_option_.enable_dynamic_polygon_ = false;
    lidar_loc_options.map_option_.map_path_ = global_map_path;
    lidar_loc_ = std::make_shared<LidarLoc>(lidar_loc_options);
    lidar_loc_->Init(yaml_path);

    pgo_ = std::make_shared<PGO>();
    pgo_->SetDebug(false);

    options_.enable_lidar_loc_skip_ = yaml.GetValue<bool>("system", "enable_lidar_loc_skip");
    options_.enable_lidar_loc_rviz_ = yaml.GetValue<bool>("system", "enable_lidar_loc_rviz");
    options_.lidar_loc_skip_num_ = yaml.GetValue<int>("system", "lidar_loc_skip_num");
    options_.enable_lidar_odom_skip_ = yaml.GetValue<bool>("system", "enable_lidar_odom_skip");
    options_.lidar_odom_skip_num_ = yaml.GetValue<int>("system", "lidar_odom_skip_num");
    options_.loc_on_kf_ = yaml.GetValue<bool>("lidar_loc", "loc_on_kf");

    lidar_odom_proc_cloud_.SetMaxSize(1);
    lidar_loc_proc_cloud_.SetMaxSize(1);
    lidar_odom_proc_cloud_.SetName("激光里程计");
    lidar_loc_proc_cloud_.SetName("激光定位");
    lidar_loc_proc_cloud_.SetSkipParam(options_.enable_lidar_loc_skip_,
                                       options_.lidar_loc_skip_num_);
    lidar_odom_proc_cloud_.SetSkipParam(options_.enable_lidar_odom_skip_,
                                        options_.lidar_odom_skip_num_);
    lidar_odom_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarOdomProcCloud(cloud); });
    lidar_loc_proc_cloud_.SetProcFunc([this](CloudPtr cloud) { LidarLocProcCloud(cloud); });

    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.Start();
        lidar_loc_proc_cloud_.Start();
    }

    pgo_->SetHighFrequencyGlobalOutputHandleFunction([this](const LocalizationResult& res) {
        loc_result_ = res;
        if (tf_callback_ && loc_result_.valid_) {
            tf_callback_(loc_result_);
        }
    });

    preprocess_.reset(new PointCloudPreprocess());
    preprocess_->Blind() = yaml.GetValue<double>("fasterlio", "blind");
    preprocess_->TimeScale() = yaml.GetValue<double>("fasterlio", "time_scale");
    int lidar_type = yaml.GetValue<int>("fasterlio", "lidar_type");
    preprocess_->NumScans() = yaml.GetValue<int>("fasterlio", "scan_line");
    preprocess_->PointFilterNum() = yaml.GetValue<int>("fasterlio", "point_filter_num");
    float height_max = yaml.GetValue<float>("roi", "height_max");
    float height_min = yaml.GetValue<float>("roi", "height_min");
    preprocess_->SetHeightROI(height_max, height_min);
    LidarType parsed_lidar_type = LidarType::GENERIC;
    if (!LidarTypeFromInt(lidar_type, &parsed_lidar_type)) {
        LOG(WARNING) << "unknown lidar_type " << lidar_type << ", fallback GENERIC";
        parsed_lidar_type = LidarType::GENERIC;
    }
    preprocess_->SetLidarType(parsed_lidar_type);
    LOG(INFO) << "lidar_type " << lidar_type;
    return true;
}

void Localization::ProcessLidarMsg(CloudPtr cloud) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr || !cloud) {
        return;
    }

    CloudPtr laser_cloud(new PointCloudType);
    preprocess_->Process(cloud, laser_cloud);

    if (options_.online_mode_) {
        lidar_odom_proc_cloud_.AddMessage(laser_cloud);
    } else {
        LidarOdomProcCloud(laser_cloud);
    }
}

void Localization::LidarOdomProcCloud(CloudPtr cloud) {
    if (lio_ == nullptr) {
        return;
    }

    lio_->ProcessPointCloud2(cloud);
    if (!lio_->Run()) {
        return;
    }

    auto lo_state = lio_->GetState();
    lidar_loc_->ProcessLO(lo_state);
    pgo_->ProcessLidarOdom(lo_state);

    auto scan = lio_->GetProjCloud();
    if (options_.loc_on_kf_) {
        auto kf = lio_->GetKeyframe();
        if (kf == lio_kf_) {
            return;
        }
        lio_kf_ = kf;
    }

    if (options_.online_mode_) {
        lidar_loc_proc_cloud_.AddMessage(scan);
    } else {
        LidarLocProcCloud(scan);
    }
}

void Localization::LidarLocProcCloud(CloudPtr scan_undist) {
    lidar_loc_->ProcessCloud(scan_undist);
    auto res = lidar_loc_->GetLocalizationResult();
    pgo_->ProcessLidarLoc(res);
    if (loc_state_callback_) {
        loc_state_callback_(static_cast<int>(res.status_));
    }
}

void Localization::ProcessIMUMsg(IMUPtr imu) {
    UL lock(global_mutex_);
    if (lidar_loc_ == nullptr || lio_ == nullptr || pgo_ == nullptr) {
        return;
    }

    double this_imu_time = imu->timestamp;
    if (last_imu_time_ > 0 && this_imu_time < last_imu_time_) {
        LOG(WARNING) << "IMU 时间异常：" << this_imu_time << ", last: " << last_imu_time_;
    }
    last_imu_time_ = this_imu_time;

    lio_->ProcessIMU(imu);
    auto dr_state = lio_->GetIMUState();
    if (!dr_state.pose_is_ok_) {
        return;
    }
    lidar_loc_->ProcessDR(dr_state);
    pgo_->ProcessDR(dr_state);
}

void Localization::Finish() {
    if (lidar_loc_) {
        lidar_loc_->Finish();
    }
    lidar_loc_proc_cloud_.Quit();
    lidar_odom_proc_cloud_.Quit();
}

void Localization::SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t) {
    UL lock(global_mutex_);
    if (lidar_loc_) {
        lidar_loc_->SetInitialPose(SE3(q, t));
    }
}

void Localization::SetTFCallback(Localization::TFCallback&& callback) {
    tf_callback_ = std::move(callback);
}

CloudPtr Localization::LatestScanWorld() const {
    if (!lio_) {
        return nullptr;
    }
    return lio_->GetScanWorld();
}

}  // namespace atlas_lio::loc
