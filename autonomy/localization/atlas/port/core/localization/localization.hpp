#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "autonomy/localization/atlas/port/common/imu.hpp"
#include "autonomy/localization/atlas/port/common/point_def.hpp"
#include "autonomy/localization/atlas/port/core/lio/laser_mapping.hpp"
#include "autonomy/localization/atlas/port/core/localization/localization_result.hpp"
#include "autonomy/localization/atlas/port/core/system/async_message_process.hpp"

namespace atlas_lio {
namespace loc {

class LidarLoc;
class PGO;

class Localization {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = false;

        SE3 T_body_lidar_;

        bool enable_lidar_odom_skip_ = false;
        int lidar_odom_skip_num_ = 1;
        bool enable_lidar_loc_skip_ = true;
        bool enable_lidar_loc_rviz_ = false;
        int lidar_loc_skip_num_ = 4;
        bool loc_on_kf_ = false;
    };

    Localization(Options options = Options());
    ~Localization() = default;

    bool Init(const std::string& yaml_path, const std::string& global_map_path);

    void ProcessLidarMsg(CloudPtr cloud);
    void ProcessIMUMsg(IMUPtr imu);

    void SetExternalPose(const Eigen::Quaterniond& q, const Eigen::Vector3d& t);

    void Finish();

    void LidarOdomProcCloud(CloudPtr);
    void LidarLocProcCloud(CloudPtr);

    using TFCallback = std::function<void(const LocalizationResult& result)>;
    using LocStateCallback = std::function<void(int state)>;

    void SetTFCallback(TFCallback&& callback);

    const LocalizationResult& result() const { return loc_result_; }

    /// Deskewed scan in the localizer world frame. Empty before the first scan.
    CloudPtr LatestScanWorld() const;

   private:
    std::mutex global_mutex_;
    Options options_;

    std::shared_ptr<PointCloudPreprocess> preprocess_ = nullptr;

    std::shared_ptr<LaserMapping> lio_ = nullptr;
    Keyframe::Ptr lio_kf_ = nullptr;

    std::shared_ptr<PGO> pgo_ = nullptr;
    std::shared_ptr<LidarLoc> lidar_loc_;

    sys::AsyncMessageProcess<CloudPtr> lidar_odom_proc_cloud_;
    sys::AsyncMessageProcess<CloudPtr> lidar_loc_proc_cloud_;

    LocalizationResult loc_result_;

    TFCallback tf_callback_;
    LocStateCallback loc_state_callback_;

    double last_imu_time_ = 0;
    double last_odom_time_ = 0;
    double last_cloud_time_ = 0;
};
}  // namespace loc

}  // namespace atlas_lio
