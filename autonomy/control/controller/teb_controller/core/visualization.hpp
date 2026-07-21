/*********************************************************************
 * Stub TebVisualization for non-ROS Autonomy integration.
 *********************************************************************/
#ifndef VISUALIZATION_H_
#define VISUALIZATION_H_

#include <memory>
#include <mutex>

#include "autonomy/common/macros.hpp"
#include "autonomy/control/controller/teb_controller/core/obstacles.hpp"
#include "autonomy/control/controller/teb_controller/core/robot_footprint_model.hpp"
#include "autonomy/control/controller/teb_controller/core/teb_config.hpp"
#include "autonomy/control/controller/teb_controller/core/teb_core.hpp"
#include "autonomy/control/controller/teb_controller/core/timed_elastic_band.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace teb_controller {

class TebOptimalPlanner;

class TebVisualization
{
public:
    AUTONOMY_SMART_PTR_DEFINITIONS(TebVisualization)
    
    explicit TebVisualization(const TebConfig&) {}
    void publishGlobalPlan(const std::vector<PoseStamped>&) const {}
    void publishLocalPlan(const std::vector<PoseStamped>&) const {}
    void publishLocalPlanAndPoses(const TimedElasticBand&) const {}
    void publishRobotFootprint(const PoseSE2&, const std::vector<Point>&,
                               double, double, double) const {}
    void publishInfeasibleRobotPose(const PoseSE2&,
                                    const BaseRobotFootprintModel&,
                                    const std::vector<Point>&) const {}
    void publishRobotFootprintModel(const PoseSE2&,
                                    const BaseRobotFootprintModel&) const {}
    void publishFeedbackMessage(const TebOptimalPlanner&,
                                const ObstContainer&) const {}
    template <typename T>
    void publishFeedbackMessage(const T&, unsigned int,
                                const ObstContainer&) const {}
    template <typename T>
    void publishTebContainer(const T&) const {}
    template <typename T>
    void publishGraph(const T&) const {}
    template <typename T>
    void publishHomotopyClasses(const T&) const {}
};

}  // namespace teb_controller
}  // namespace controller
}  // namespace control
}  // namespace autonomy

#endif
