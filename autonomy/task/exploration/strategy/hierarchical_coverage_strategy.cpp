/* Copyright 2026 The Openbot Authors. */
/** @file hierarchical_coverage_strategy.cpp
 * @brief Hierarchical candidate selection with reusable compatible sweep
 * evidence.
 */
#include "autonomy/task/exploration/strategy/hierarchical_coverage_strategy.hpp"
#include <algorithm>
#include <stdexcept>
#include "autonomy/common/logging.hpp"
#include "autonomy/task/exploration/coverage/local_coverage_planner.hpp"
namespace autonomy::task::exploration {
HierarchicalCoverageStrategy::HierarchicalCoverageStrategy(
    std::shared_ptr<const PathProvider> paths,
    std::shared_ptr<const ViewpointEvaluator> evaluator,
    HierarchicalCoverageOptions options)
    : paths_(std::move(paths)),
      evaluator_(std::move(evaluator)),
      options_(options),
      regions_(options.region_size) {
    if (options_.rolling_field)
        rolling_field_ =
            std::make_unique<RollingViewpointField>(*options_.rolling_field);
    if (options_.use_momentum)
        momentum_tracker_.emplace(options_.momentum);
    if (!paths_ || !evaluator_ || !options.local_views ||
        options.local_views > 32 || !options.local_selection_attempts ||
        options.local_selection_attempts > 64 ||
        options.local_selection_attempts * options.local_views > 256 ||
        !options.maximum_global_views ||
        options.maximum_global_views >
            options.route_optimization.maximum_nodes ||
        options.route_optimization.maximum_nodes > 128 ||
        !std::isfinite(options.local_radius) || options.local_radius <= 0 ||
        !std::isfinite(options.fairness_interval) ||
        options.fairness_interval <= 0)
        throw std::invalid_argument("invalid hierarchical coverage strategy");
    if (!std::isfinite(options.sweep_max_age) || options.sweep_max_age <= 0)
        throw std::invalid_argument("invalid sweep proof lifetime");
}
void HierarchicalCoverageStrategy::Reset() {
    regions_ = RegionManager(options_.region_size);
    queue_.Reset();
    epoch_ = next_goal_ = 0;
    last_fair_selection_ = -1e30;
    sampling_map_revision_ = sampling_coverage_revision_ = 0;
    sampling_sensors_ = sweep_start_sensors_ = 0;
    normal_cursor_ = sweep_cursor_ = region_cursor_ = 0;
    preferred_remote_region_.reset();
    if (momentum_tracker_)
        momentum_tracker_->Reset();
    momentum_state_ = {};
    sweep_completed_ = false;
    sweep_completed_at_ = 0;
    sweep_start_map_revision_ = sweep_start_coverage_revision_ = 0;
    sampling_start_cell_.reset();
    if (rolling_field_)
        rolling_field_->Clear();
    rolling_field_generation_ = 0;
    failed_viewpoints_.clear();
    retained_local_views_.clear();
}

void HierarchicalCoverageStrategy::UpdateMeasuredMotion(
    const Pose2d& pose, double acquisition_stamp,
    const MomentumSampleIdentity& identity) {
    if (!momentum_tracker_) {
        momentum_state_ = {};
        return;
    }
    momentum_state_ =
        momentum_tracker_->Update(pose, acquisition_stamp, identity);
}

void HierarchicalCoverageStrategy::MarkFailed(const ObservationGoal& goal,
                                              double now) {
    failed_viewpoints_.push_back({goal.original_pose, now + 60.0});
    if (failed_viewpoints_.size() > 64) {
        failed_viewpoints_.erase(failed_viewpoints_.begin());
    }
}
StrategyDecision HierarchicalCoverageStrategy::Select(
    const Pose2d& start, const Pose2d& home, const MapReadView& map,
    const CoverageState& state, perception::SensorMask active, double now,
    const std::function<bool()>& cancel_checker) {
    StrategyDecision decision;
    if (cancel_checker && cancel_checker()) {
        decision.canceled = true;
        return decision;
    }
    if (state.epoch() != map.map_epoch || !active || !map.grid) {
        sweep_completed_ = false;
        return decision;
    }
    if (epoch_ != map.map_epoch) {
        epoch_ = map.map_epoch;
        queue_.Reset();
        last_fair_selection_ = -1e30;
        sampling_map_revision_ = 0;
        region_cursor_ = normal_cursor_ = sweep_cursor_ = 0;
        sweep_completed_ = false;
        sampling_start_cell_.reset();
        failed_viewpoints_.clear();
        retained_local_views_.clear();
    }
    std::shared_ptr<const RollingViewpointFieldView> viewpoint_field;
    if (rolling_field_) {
        if (!map.observed_volume) {
            sweep_completed_ = false;
            return decision;
        }
        double focus_height = map.observed_volume->origin.z();
        if (const auto start_cell = map.grid->WorldToCell(start); start_cell) {
            const double floor =
                map.grid->At(start_cell->x, start_cell->y).floor_height;
            if (std::isfinite(floor))
                focus_height = floor;
        }
        const auto update = rolling_field_->Update(
            *map.observed_volume, {start.x, start.y, focus_height},
            cancel_checker);
        if (update.status == ViewpointFieldUpdateStatus::kCanceled) {
            decision.canceled = true;
            return decision;
        }
        if (update.status == ViewpointFieldUpdateStatus::kRejectedInvalid ||
            update.status == ViewpointFieldUpdateStatus::kRejectedStale) {
            sweep_completed_ = false;
            return decision;
        }
        viewpoint_field = rolling_field_->Read();
        if (!viewpoint_field ||
            !viewpoint_field->Compatible(map.map_epoch, map.geometry_revision,
                                         map.frame_id)) {
            sweep_completed_ = false;
            return decision;
        }
        if (rolling_field_generation_ != viewpoint_field->generation) {
            rolling_field_generation_ = viewpoint_field->generation;
            normal_cursor_ = sweep_cursor_ = region_cursor_ = 0;
            retained_local_views_.clear();
            sweep_completed_ = false;
        }
    }
    const auto cell = map.grid->WorldToCell(start);
    const auto origin =
        cell ? std::optional<std::pair<int, int>>{{cell->x, cell->y}}
             : std::nullopt;
    const bool viewpoint_dependencies_changed =
        sampling_map_revision_ != map.geometry_revision ||
        sampling_coverage_revision_ != state.requirements_revision() ||
        sampling_sensors_ != active;
    if (viewpoint_dependencies_changed || sampling_start_cell_ != origin) {
        if (viewpoint_dependencies_changed)
            retained_local_views_.clear();
        // Keep a bounded cursor moving, but do not accept a pass spanning new
        // work, changed geometry/capabilities or a different reachability seed.
        sweep_completed_ = false;
        sampling_map_revision_ = map.geometry_revision;
        sampling_coverage_revision_ = state.requirements_revision();
        sampling_sensors_ = active;
        // The sweep cursor is a pagination offset over map cells, not over the
        // current seed.  Resetting it when the robot crosses a cell boundary
        // can restart every page before a sparse free region is reached.
        sampling_start_cell_ = origin;
    }
    if (options_.use_frontier_points) {
        queue_.ReopenForMapChange(map.geometry_revision);
        queue_.Synchronize(state, now);
    }
    regions_.Update(state);
    regions_.ObserveRobotPose(start, now);
    if (sweep_completed_ &&
        (now < sweep_completed_at_ ||
         now - sweep_completed_at_ > options_.sweep_max_age)) {
        sweep_completed_ = false;
        sweep_cursor_ = 0;
    }
    if (sweep_completed_ && !state.PendingCount()) {
        decision.grid_fallback_checked = decision.sweep_checked = true;
        return decision;
    }
    auto sampled = ViewpointSampler{}.Sample(
        start, map, state, *evaluator_, regions_, active, options_.safety,
        options_.sampling, false, normal_cursor_, cancel_checker,
        viewpoint_field);
    normal_cursor_ = sampled.finished ? 0 : sampled.next_cursor;
    if (sampled.canceled) {
        decision.canceled = true;
        return decision;
    }
    // The oldest pending unit is the one fair selection must serve.  It has to
    // be known before the sweep runs so paging can keep looking for a view that
    // actually sees it; choosing a page without a contributor silently degraded
    // the fair pass into another local selection and kept the robot shuttling
    // inside one room while whole rooms stayed pending.
    const bool fairness_due =
        now - last_fair_selection_ >= options_.fairness_interval;
    std::optional<CoverageId> mandatory;
    if (fairness_due && options_.use_frontier_points)
        mandatory = queue_.OldestDue(now);
    if (sampled.candidates.empty() || fairness_due) {
        const auto contributes = [](const ViewpointSamplingResult& page,
                                    const CoverageId& id) {
            return std::any_of(page.candidates.begin(), page.candidates.end(),
                               [&](const Viewpoint& view) {
                                   return std::any_of(
                                       view.contributions.begin(),
                                       view.contributions.end(),
                                       [&](const auto& contribution) {
                                           return contribution.id == id &&
                                                  contribution.quality > 0;
                                       });
                               });
        };
        const auto sample_page = [&] {
            if (!sweep_cursor_) {
                sweep_start_map_revision_ = map.geometry_revision;
                sweep_start_coverage_revision_ = state.requirements_revision();
                sweep_start_sensors_ = active;
            }
            auto page = ViewpointSampler{}.Sample(
                start, map, state, *evaluator_, regions_, active,
                options_.safety, options_.sampling, true, sweep_cursor_,
                cancel_checker, viewpoint_field);
            if (page.canceled)
                return page;
            if (page.finished) {
                sweep_completed_ =
                    sweep_start_map_revision_ == map.geometry_revision &&
                    sweep_start_coverage_revision_ ==
                        state.requirements_revision() &&
                    sweep_start_sensors_ == active;
                if (sweep_completed_)
                    sweep_completed_at_ = now;
                sweep_cursor_ = 0;
            } else {
                sweep_cursor_ = page.next_cursor;
            }
            return page;
        };
        auto swept = sample_page();
        for (unsigned extra = 0;
             fairness_due && mandatory && !swept.canceled && !swept.finished &&
             !contributes(swept, *mandatory) && extra < 3;
             ++extra) {
            swept = sample_page();
        }
        if (swept.canceled) {
            decision.canceled = true;
            return decision;
        }
        sampled = std::move(swept);
    }
    if (sampled.canceled) {
        decision.canceled = true;
        return decision;
    }
    for (const auto& retained : retained_local_views_) {
        const bool present =
            std::any_of(sampled.candidates.begin(), sampled.candidates.end(),
                        [&](const Viewpoint& candidate) {
                            return candidate.id == retained.id;
                        });
        if (!present)
            sampled.candidates.push_back(retained);
    }
    sampled.candidates.erase(
        std::remove_if(
            sampled.candidates.begin(), sampled.candidates.end(),
            [&](const Viewpoint& candidate) {
                return std::any_of(
                    failed_viewpoints_.begin(), failed_viewpoints_.end(),
                    [&](const FailedViewpoint& failed) {
                        return now < failed.until &&
                               autonomy::navigation::Distance(
                                   candidate.pose, failed.pose) < 0.75;
                    });
            }),
        sampled.candidates.end());
    decision.sampling_limited = sampled.limited;
    decision.grid_fallback_checked = true;
    decision.sweep_checked = sweep_completed_;
    // Local work and remote relocation are separate. A remote representative
    // cannot displace still-valid dense work around the measured robot pose.
    const auto current_region = regions_.RegionFor(start);
    std::map<std::pair<int, int>, Viewpoint> representatives;
    std::vector<Viewpoint> local;
    for (const auto& candidate : sampled.candidates) {
        const bool serves_mandatory =
            mandatory &&
            std::any_of(candidate.contributions.begin(),
                        candidate.contributions.end(),
                        [&](const auto& c) { return c.id == *mandatory; });
        if (candidate.region_id == current_region ||
            autonomy::navigation::Distance(candidate.pose, start) <=
                options_.local_radius ||
            serves_mandatory)
            local.push_back(candidate);
        if (candidate.region_id == current_region ||
            autonomy::navigation::Distance(candidate.pose, start) <=
                options_.local_radius)
            continue;
        auto found = representatives.find(candidate.region_id);
        if (found == representatives.end() ||
            candidate.Utility(candidate.predicted_gain) >
                found->second.Utility(found->second.predicted_gain))
            representatives[candidate.region_id] = candidate;
    }
    std::vector<Viewpoint> global;
    for (const auto& representative : representatives)
        global.push_back(representative.second);
    bool keep_remote_target = false;
    if (preferred_remote_region_) {
        const auto preferred = std::find_if(
            global.begin(), global.end(), [&](const Viewpoint& viewpoint) {
                return viewpoint.region_id == *preferred_remote_region_;
            });
        if (preferred != global.end()) {
            std::rotate(global.begin(), preferred, std::next(preferred));
            keep_remote_target = true;
        } else {
            preferred_remote_region_.reset();
        }
    }
    if (momentum_state_.active && momentum_state_.preferred_direction) {
        const auto direction = *momentum_state_.preferred_direction;
        const auto begin =
            keep_remote_target ? std::next(global.begin()) : global.begin();
        std::stable_sort(begin, global.end(),
                         [&](const Viewpoint& left, const Viewpoint& right) {
                             const Eigen::Vector2d left_delta(
                                 left.pose.x - start.x, left.pose.y - start.y);
                             const Eigen::Vector2d right_delta(
                                 right.pose.x - start.x,
                                 right.pose.y - start.y);
                             const double left_alignment =
                                 left_delta.norm() > 0
                                     ? direction.dot(left_delta.normalized())
                                     : -1.0;
                             const double right_alignment =
                                 right_delta.norm() > 0
                                     ? direction.dot(right_delta.normalized())
                                     : -1.0;
                             return left_alignment > right_alignment;
                         });
        keep_remote_target = !global.empty();
    }
    const std::size_t global_limit = options_.maximum_global_views;
    if (global.size() > global_limit) {
        std::vector<Viewpoint> batch;
        batch.reserve(global_limit);
        if (keep_remote_target)
            batch.push_back(global.front());
        for (std::size_t offset = 0; batch.size() < global_limit; ++offset) {
            const auto index = (region_cursor_ + offset) % global.size();
            if (keep_remote_target && index == 0)
                continue;
            batch.push_back(global[index]);
        }
        region_cursor_ = (region_cursor_ + global_limit) % global.size();
        global = std::move(batch);
        decision.sampling_limited = true;
    }
    decision.global_route = VisitOrderOptimizer{}.Optimize(
        start, global, *paths_, options_.sampling.nominal_speed,
        options_.sampling.angular_speed, home, keep_remote_target,
        cancel_checker, options_.route_optimization);
    if (!decision.global_route.views.empty()) {
        preferred_remote_region_ =
            decision.global_route.views.front().region_id;
        regions_.SetRepresentative(*preferred_remote_region_,
                                   decision.global_route.views.front().id);
    } else {
        preferred_remote_region_.reset();
    }
    const std::size_t selection_attempts =
        local.empty()
            ? 1
            : std::min(options_.local_selection_attempts, local.size());
    bool have_local_route = false;
    for (std::size_t attempt = 0; attempt < selection_attempts; ++attempt) {
        auto variant = local;
        if (!variant.empty())
            std::rotate(variant.begin(), variant.begin() + attempt,
                        variant.end());
        auto selected = LocalCoveragePlanner{}.Select(
            variant, state, options_.local_views, mandatory);
        const bool mandatory_selected = !selected.empty() && mandatory;
        if (selected.empty() && mandatory)
            selected = LocalCoveragePlanner{}.Select(variant, state,
                                                     options_.local_views);
        auto route = VisitOrderOptimizer{}.Optimize(
            start, selected, *paths_, options_.sampling.nominal_speed,
            options_.sampling.angular_speed, {}, mandatory_selected,
            cancel_checker, options_.route_optimization);
        if (!have_local_route ||
            (route.reachable && !route.views.empty() &&
             (!decision.local_route.reachable ||
              decision.local_route.views.empty() ||
              route.distance < decision.local_route.distance))) {
            decision.local_route = std::move(route);
            have_local_route = true;
        }
    }
    if (decision.local_route.reachable)
        retained_local_views_ = decision.local_route.views;
    else
        retained_local_views_.clear();
    AINFO_EVERY(5) << "EXPLORATION_SELECT candidates="
                   << sampled.candidates.size() << " local=" << local.size()
                   << " local_reachable=" << decision.local_route.reachable
                   << " selected_views=" << decision.local_route.views.size()
                   << " global_views=" << decision.global_route.views.size()
                   << " pending=" << state.PendingCount()
                   << " map_revision=" << map.geometry_revision
                   << " sampling_revision=" << sampling_map_revision_
                   << " sweep=" << sweep_completed_
                   << " limited=" << decision.sampling_limited
                   << " examined=" << sampled.examined
                   << " evaluated=" << sampled.evaluated
                   << " page_finished=" << sampled.finished;
    if (sampled.candidates.empty() && state.PendingCount()) {
        std::size_t frontiers = 0, surfaces = 0, directions = 0;
        double minimum_z = 1e30, maximum_z = -1e30;
        for (const auto& item : state.units()) {
            if (!item.second.pending)
                continue;
            frontiers += item.first.kind == CoverageKind::kFrontier;
            surfaces += item.first.kind == CoverageKind::kSurface;
            directions += item.first.kind == CoverageKind::kDirection;
            minimum_z = std::min(minimum_z, item.second.center.z);
            maximum_z = std::max(maximum_z, item.second.center.z);
        }
        AINFO_EVERY(5) << "EXPLORATION_PENDING frontiers=" << frontiers
                       << " surfaces=" << surfaces
                       << " directions=" << directions << " z=[" << minimum_z
                       << "," << maximum_z << "]"
                       << " candidates=" << sampled.candidates.size();
    }
    if (cancel_checker && cancel_checker()) {
        decision.canceled = true;
        return decision;
    }
    const ExplorationRoute* selected_route = nullptr;
    if (!decision.local_route.views.empty() && decision.local_route.reachable) {
        selected_route = &decision.local_route;
    } else if (!decision.global_route.views.empty() &&
               decision.global_route.reachable) {
        // A local page can be empty or disconnected while a coarse regional
        // representative still has a valid route. Returning no goal here
        // falsely declared a global dead end and stalled the session.
        selected_route = &decision.global_route;
        AINFO_EVERY(5) << "EXPLORATION_GLOBAL_FALLBACK local_reachable="
                       << decision.local_route.reachable << " global_views="
                       << decision.global_route.views.size();
    }
    if (selected_route) {
        const auto& view = selected_route->views.front();
        ObservationGoal goal;
        if (mandatory &&
            std::any_of(view.contributions.begin(), view.contributions.end(),
                        [&](const auto& c) {
                            return c.id == *mandatory && c.quality > 0;
                        }))
            last_fair_selection_ = now;
        goal.goal_id = ++next_goal_;
        goal.goal_version = 1;
        goal.map_epoch = map.map_epoch;
        goal.coverage_revision = state.revision();
        goal.original_pose = view.pose;
        goal.region_id = view.region_id;
        goal.sensor_mode = options_.mode;
        for (const auto& c : view.contributions)
            if (c.quality > 0 &&
                std::find(goal.required_coverage_units.begin(),
                          goal.required_coverage_units.end(),
                          c.id) == goal.required_coverage_units.end())
                goal.required_coverage_units.push_back(c.id);
        goal.required_views = {view.pose.yaw};
        goal.minimum_gain = view.predicted_gain * .5;
        decision.goal = std::move(goal);
    }
    return decision;
}
}  // namespace autonomy::task::exploration
