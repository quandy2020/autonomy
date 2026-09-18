//
// Created by xiang on 25-5-6.
//

#ifndef LIGHTNING_SLAM_H
#define LIGHTNING_SLAM_H

#include <atomic>
#include <memory>
#include <string>

#include "common/eigen_types.hpp"
#include "common/imu.hpp"
#include "common/keyframe.hpp"
#include "common/point_def.hpp"

namespace lightning {

class LaserMapping;
class LoopClosing;

namespace g2p5 {
class G2P5;
}

/**
 * SLAM 系统调用接口（无 ROS：由 autolink bridge 喂 IMU / 点云）
 */
class SlamSystem {
   public:
    struct Options {
        Options() {}

        bool online_mode_ = true;

        bool with_cc_ = true;
        bool with_gridmap_ = true;
        bool with_loop_closing_ = true;
        bool with_2dvisualization_ = false;

        bool step_on_kf_ = false;
    };

    explicit SlamSystem(Options options);
    ~SlamSystem();

    bool Init(const std::string& yaml_path);

    void StartSLAM(std::string map_name);

    void SaveMap(const std::string& path = "");

    void ProcessIMU(const lightning::IMUPtr& imu);

    void ProcessLidar(const CloudPtr& cloud);
    /// LIO only (IMU mutex may be held). Loop / G2P5 run via FlushPendingKeyframe.
    void ProcessLidarFrontend(const CloudPtr& cloud);
    void FlushPendingKeyframe();

    LaserMapping* lio() const { return lio_.get(); }
    LoopClosing* loop_closing() const { return lc_.get(); }
    g2p5::G2P5* g2p5() const { return g2p5_.get(); }

   private:
    Options options_;
    std::atomic_bool running_ = false;

    std::string map_name_;

    std::shared_ptr<LaserMapping> lio_ = nullptr;
    std::shared_ptr<LoopClosing> lc_ = nullptr;
    std::shared_ptr<g2p5::G2P5> g2p5_ = nullptr;

    Keyframe::Ptr cur_kf_ = nullptr;
};
}  // namespace lightning

#endif  // LIGHTNING_SLAM_H
