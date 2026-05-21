#include "autonomy/control/controller/mppi_controller/critics/path_align_critic.hpp"

#include "autonomy/common/math/math.hpp"
#include "autonomy/control/controller/mppi_controller/tools/utils.hpp"

namespace autonomy {
namespace control {
namespace controller {
namespace mppi {
namespace critics {

void PathAlignCritic::Configure(
    const proto::MPPIControllerOptions& options,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    costmap_wrapper_ = std::move(costmap_wrapper);
    const auto& cfg = options.path_align_critic();
    enabled_ = cfg.enabled();
    power_ = cfg.cost_power() > 0 ? cfg.cost_power() : 1;
    weight_ = static_cast<float>(cfg.cost_weight() > 0.0 ? cfg.cost_weight() : 14.0);
    threshold_ = static_cast<float>(cfg.threshold_to_consider() > 0.0
                                         ? cfg.threshold_to_consider()
                                         : 0.5);
    max_path_occupancy_ratio_ =
        static_cast<float>(cfg.max_path_occupancy_ratio() > 0.0
                               ? cfg.max_path_occupancy_ratio()
                               : 0.05);
    offset_ = cfg.offset_from_furthest() > 0
                  ? static_cast<size_t>(cfg.offset_from_furthest())
                  : 20u;
    trajectory_point_step_ = cfg.trajectory_point_step() > 0
                                 ? cfg.trajectory_point_step()
                                 : 4;
    use_path_orientations_ = cfg.use_path_orientations();
}

void PathAlignCritic::score(CriticData& data) {
    if (!enabled_) {
        return;
    }
    const auto goal = utils::getCriticGoal(data, enforce_path_inversion_);
    if (utils::withinPositionGoalTolerance(threshold_, data.state.pose.pose,
                                           goal)) {
        return;
    }
    utils::setPathFurthestPointIfNotSet(data);
    const size_t path_segments_count = *data.furthest_reached_path_point;
    const float path_segments_flt = static_cast<float>(path_segments_count);
    if (path_segments_count < offset_) {
        return;
    }

    utils::setPathCostsIfNotSet(data, costmap_wrapper_);
    std::vector<bool>& path_pts_valid = *data.path_pts_valid;
    float invalid_ctr = 0.0f;
    for (size_t i = 0; i < path_segments_count; ++i) {
        if (!path_pts_valid[i]) {
            invalid_ctr += 1.0f;
        }
        if (invalid_ctr / path_segments_flt > max_path_occupancy_ratio_ &&
            invalid_ctr > 2.0f) {
            return;
        }
    }

    const size_t batch_size = static_cast<size_t>(data.trajectories.x.rows());
    Eigen::ArrayXf cost(static_cast<Eigen::Index>(data.costs.rows()));
    cost.setZero();

    std::vector<float> path_integrated_distances(path_segments_count, 0.0f);
    std::vector<utils::Pose2D> path(path_segments_count);
    float dx = 0.0f;
    float dy = 0.0f;
    for (unsigned int i = 1; i != path_segments_count; ++i) {
        auto& pose = path[i - 1];
        pose.x = data.path.x(static_cast<Eigen::Index>(i - 1));
        pose.y = data.path.y(static_cast<Eigen::Index>(i - 1));
        pose.theta = data.path.yaws(static_cast<Eigen::Index>(i - 1));
        dx = data.path.x(static_cast<Eigen::Index>(i)) - pose.x;
        dy = data.path.y(static_cast<Eigen::Index>(i)) - pose.y;
        path_integrated_distances[i] =
            path_integrated_distances[i - 1] + std::sqrt(dx * dx + dy * dy);
    }
    auto& final_pose = path[path_segments_count - 1];
    final_pose.x = data.path.x(static_cast<Eigen::Index>(path_segments_count - 1));
    final_pose.y = data.path.y(static_cast<Eigen::Index>(path_segments_count - 1));
    final_pose.theta =
        data.path.yaws(static_cast<Eigen::Index>(path_segments_count - 1));

    const int strided_traj_rows = data.trajectories.x.rows();
    const int strided_traj_cols =
        static_cast<int>(std::floor((data.trajectories.x.cols() - 1) /
                                    trajectory_point_step_)) +
        1;
    const int outer_stride = strided_traj_rows * trajectory_point_step_;

    const auto T_x = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
        data.trajectories.x.data(), strided_traj_rows, strided_traj_cols,
        Eigen::Stride<-1, -1>(outer_stride, 1));
    const auto T_y = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
        data.trajectories.y.data(), strided_traj_rows, strided_traj_cols,
        Eigen::Stride<-1, -1>(outer_stride, 1));
    const auto T_yaw = Eigen::Map<const Eigen::ArrayXXf, 0, Eigen::Stride<-1, -1>>(
        data.trajectories.yaws.data(), strided_traj_rows, strided_traj_cols,
        Eigen::Stride<-1, -1>(outer_stride, 1));
    const auto traj_sampled_size = T_x.cols();

    for (size_t t = 0; t < batch_size; ++t) {
        float summed_path_dist = 0.0f;
        unsigned int num_samples = 0u;
        float traj_integrated_distance = 0.0f;
        unsigned int path_pt = 0u;
        float Tx_m1 = T_x(static_cast<Eigen::Index>(t), 0);
        float Ty_m1 = T_y(static_cast<Eigen::Index>(t), 0);
        for (int p = 1; p < traj_sampled_size; ++p) {
            const float Tx = T_x(static_cast<Eigen::Index>(t), p);
            const float Ty = T_y(static_cast<Eigen::Index>(t), p);
            dx = Tx - Tx_m1;
            dy = Ty - Ty_m1;
            Tx_m1 = Tx;
            Ty_m1 = Ty;
            traj_integrated_distance += std::sqrt(dx * dx + dy * dy);
            path_pt = utils::findClosestPathPt(path_integrated_distances,
                                               traj_integrated_distance, path_pt);
            if (path_pts_valid[path_pt]) {
                const auto& pose = path[path_pt];
                dx = pose.x - Tx;
                dy = pose.y - Ty;
                ++num_samples;
                if (use_path_orientations_) {
                    const float dyaw = static_cast<float>(
                        ::autonomy::common::NormalizeAngleDifference(
                            pose.theta - T_yaw(static_cast<Eigen::Index>(t), p)));
                    summed_path_dist += std::sqrt(dx * dx + dy * dy + dyaw * dyaw);
                } else {
                    summed_path_dist += std::sqrt(dx * dx + dy * dy);
                }
            }
        }
        cost(static_cast<Eigen::Index>(t)) =
            num_samples > 0u ? summed_path_dist / static_cast<float>(num_samples)
                             : 0.0f;
    }

    if (power_ > 1) {
        data.costs += (cost * weight_).pow(power_).eval();
    } else {
        data.costs += (cost * weight_).eval();
    }
}

}  // namespace critics
}  // namespace mppi
}  // namespace controller
}  // namespace control
}  // namespace autonomy
