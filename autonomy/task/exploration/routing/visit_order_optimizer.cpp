/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/routing/visit_order_optimizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <set>
#include <stdexcept>

namespace autonomy::task::exploration {
namespace {

using planning::planner::vista::VistaRoute;

bool ValidRoute(const VistaRoute& route) {
    if (!route.success || route.canceled || route.attempted ||
        !std::isfinite(route.length) || route.length < 0) {
        return false;
    }
    if (!route.evidence)
        return true;
    return route.map_epoch == route.evidence->map_epoch &&
           route.geometry_revision == route.evidence->geometry_revision &&
           !route.evidence->frame_id.empty();
}

bool SameGeometry(const VistaRoute& left, const VistaRoute& right) {
    if (left.map_epoch != right.map_epoch ||
        left.geometry_revision != right.geometry_revision ||
        left.evidence.has_value() != right.evidence.has_value()) {
        return false;
    }
    if (!left.evidence)
        return true;
    return left.evidence->map_epoch == right.evidence->map_epoch &&
           left.evidence->geometry_revision ==
               right.evidence->geometry_revision &&
           left.evidence->frame_id == right.evidence->frame_id;
}

std::size_t BitCount(std::uint32_t value) {
    std::size_t count = 0;
    while (value) {
        value &= value - 1;
        ++count;
    }
    return count;
}

}  // namespace

ExplorationRoute VisitOrderOptimizer::Optimize(
    const Pose2d& start, const std::vector<Viewpoint>& views,
    const PathProvider& provider, double speed, double angular_speed,
    const std::optional<Pose2d>& return_pose, bool keep_first,
    const std::function<bool()>& cancel_checker,
    VisitOrderOptimizerOptions options) const {
    if (!std::isfinite(speed) || speed <= 0 || !std::isfinite(angular_speed) ||
        angular_speed <= 0 || !options.maximum_nodes ||
        options.maximum_nodes > 128 || options.exact_maximum_nodes > 18 ||
        options.exact_maximum_nodes > options.maximum_nodes ||
        views.size() > options.maximum_nodes) {
        throw std::invalid_argument("invalid route optimization limits");
    }

    ExplorationRoute result;
    if (views.empty())
        return result;

    const std::size_t count = views.size();
    const std::size_t origin = count;
    const std::size_t destination = count + 1;
    std::vector<Pose2d> poses;
    poses.reserve(count + 2);
    for (const auto& view : views)
        poses.push_back(view.pose);
    poses.push_back(start);
    poses.push_back(return_pose.value_or(start));

    std::vector<std::vector<VistaRoute>> routes(
        count + 2, std::vector<VistaRoute>(count + 2));
    for (std::size_t from = 0; from < poses.size(); ++from) {
        for (std::size_t to = 0; to < poses.size(); ++to) {
            if (from == to)
                continue;
            if (cancel_checker && cancel_checker()) {
                result.reachable = false;
                return result;
            }
            routes[from][to] =
                provider.Route(poses[from], poses[to], cancel_checker);
            if (routes[from][to].canceled) {
                result.reachable = false;
                return result;
            }
        }
    }

    const auto edge_time = [&](std::size_t from, std::size_t to) {
        const auto& route = routes[from][to];
        if (!ValidRoute(route))
            return std::numeric_limits<double>::infinity();
        return route.length / speed +
               std::abs(autonomy::navigation::NormalizeAngle(poses[to].yaw -
                                                             poses[from].yaw)) /
                   angular_speed;
    };
    const auto route_compatible = [&](const std::vector<std::size_t>& order,
                                      const VistaRoute& candidate) {
        if (!ValidRoute(candidate))
            return false;
        if (order.empty())
            return true;
        std::size_t previous = origin;
        for (const auto index : order) {
            if (!SameGeometry(routes[previous][index], candidate))
                return false;
            previous = index;
        }
        return true;
    };

    std::vector<std::size_t> order;
    std::optional<VistaRoute> return_leg;
    bool exact_used = false;
    bool canceled = false;

    if (options.exact_maximum_nodes && count <= options.exact_maximum_nodes) {
        struct ExactResult {
            std::vector<std::size_t> order;
            double time = std::numeric_limits<double>::infinity();
        };
        std::vector<const VistaRoute*> geometries;
        for (std::size_t index = 0; index < count; ++index) {
            const auto& route = routes[origin][index];
            if (!ValidRoute(route))
                continue;
            const auto duplicate =
                std::find_if(geometries.begin(), geometries.end(),
                             [&](const VistaRoute* known) {
                                 return SameGeometry(*known, route);
                             });
            if (duplicate == geometries.end())
                geometries.push_back(&route);
        }

        const auto solve_exact = [&](bool force_first) -> ExactResult {
            ExactResult best;
            const std::uint32_t subset_count = std::uint32_t{1} << count;
            const std::size_t state_count =
                static_cast<std::size_t>(subset_count) * count;
            for (const VistaRoute* geometry : geometries) {
                if (cancel_checker && cancel_checker()) {
                    canceled = true;
                    return {};
                }
                std::vector<double> costs(
                    state_count, std::numeric_limits<double>::infinity());
                std::vector<std::int8_t> parents(state_count, -2);
                for (std::size_t first = 0; first < count; ++first) {
                    if (force_first && first != 0)
                        continue;
                    const auto& route = routes[origin][first];
                    if (!ValidRoute(route) || !SameGeometry(*geometry, route))
                        continue;
                    const std::uint32_t mask = std::uint32_t{1} << first;
                    const std::size_t state =
                        static_cast<std::size_t>(mask) * count + first;
                    costs[state] = edge_time(origin, first) +
                                   views[first].observation_time;
                    parents[state] = -1;
                }
                for (std::uint32_t mask = 1; mask < subset_count; ++mask) {
                    if (force_first && !(mask & 1U))
                        continue;
                    if (cancel_checker && cancel_checker()) {
                        canceled = true;
                        return {};
                    }
                    for (std::size_t last = 0; last < count; ++last) {
                        if (!(mask & (std::uint32_t{1} << last)))
                            continue;
                        const std::size_t state =
                            static_cast<std::size_t>(mask) * count + last;
                        if (!std::isfinite(costs[state]))
                            continue;
                        for (std::size_t next = 0; next < count; ++next) {
                            const std::uint32_t bit = std::uint32_t{1} << next;
                            if (mask & bit)
                                continue;
                            const auto& route = routes[last][next];
                            if (!ValidRoute(route) ||
                                !SameGeometry(*geometry, route)) {
                                continue;
                            }
                            const std::uint32_t next_mask = mask | bit;
                            const std::size_t next_state =
                                static_cast<std::size_t>(next_mask) * count +
                                next;
                            const double candidate =
                                costs[state] + edge_time(last, next) +
                                views[next].observation_time;
                            if (candidate + 1e-9 < costs[next_state]) {
                                costs[next_state] = candidate;
                                parents[next_state] =
                                    static_cast<std::int8_t>(last);
                            }
                        }
                    }
                }
                for (std::uint32_t mask = 1; mask < subset_count; ++mask) {
                    const std::size_t cardinality = BitCount(mask);
                    if (cardinality < best.order.size())
                        continue;
                    for (std::size_t last = 0; last < count; ++last) {
                        if (!(mask & (std::uint32_t{1} << last)))
                            continue;
                        const std::size_t state =
                            static_cast<std::size_t>(mask) * count + last;
                        double candidate = costs[state];
                        if (!std::isfinite(candidate))
                            continue;
                        if (return_pose) {
                            const auto& home = routes[last][destination];
                            if (!ValidRoute(home) ||
                                !SameGeometry(*geometry, home))
                                continue;
                            candidate += edge_time(last, destination);
                        }
                        if (cardinality == best.order.size() &&
                            candidate + 1e-9 >= best.time) {
                            continue;
                        }
                        std::vector<std::size_t> candidate_order(cardinality);
                        std::uint32_t cursor_mask = mask;
                        std::size_t cursor = last;
                        for (std::size_t position = cardinality; position > 0;
                             --position) {
                            candidate_order[position - 1] = cursor;
                            const std::size_t cursor_state =
                                static_cast<std::size_t>(cursor_mask) * count +
                                cursor;
                            const std::int8_t parent = parents[cursor_state];
                            cursor_mask &= ~(std::uint32_t{1} << cursor);
                            if (parent < 0)
                                break;
                            cursor = static_cast<std::size_t>(parent);
                        }
                        best.order = std::move(candidate_order);
                        best.time = candidate;
                    }
                }
            }
            return best;
        };

        ExactResult exact;
        if (keep_first)
            exact = solve_exact(true);
        if (canceled) {
            result.reachable = false;
            return result;
        }
        if (exact.order.empty())
            exact = solve_exact(false);
        if (canceled) {
            result.reachable = false;
            return result;
        }
        order = std::move(exact.order);
        if (!order.empty() && return_pose) {
            return_leg = routes[order.back()][destination];
        }
        exact_used = true;
    }

    if (!exact_used) {
        std::set<std::size_t> remaining;
        for (std::size_t index = 0; index < count; ++index) {
            remaining.insert(index);
        }
        std::size_t previous = origin;
        if (keep_first && ValidRoute(routes[origin][0]) &&
            (!return_pose ||
             (ValidRoute(routes[0][destination]) &&
              SameGeometry(routes[origin][0], routes[0][destination])))) {
            order.push_back(0);
            remaining.erase(0);
            previous = 0;
        }

        while (!remaining.empty()) {
            if (cancel_checker && cancel_checker()) {
                result.reachable = false;
                return result;
            }
            auto best = remaining.end();
            double best_time = std::numeric_limits<double>::infinity();
            for (auto candidate = remaining.begin();
                 candidate != remaining.end(); ++candidate) {
                const auto& route = routes[previous][*candidate];
                if (!route_compatible(order, route))
                    continue;
                const double candidate_time = edge_time(previous, *candidate);
                if (candidate_time < best_time) {
                    best = candidate;
                    best_time = candidate_time;
                }
            }
            if (best == remaining.end())
                break;
            previous = *best;
            order.push_back(previous);
            remaining.erase(best);
        }

        if (return_pose) {
            while (!order.empty()) {
                const auto& candidate = routes[order.back()][destination];
                if (route_compatible(order, candidate)) {
                    return_leg = candidate;
                    break;
                }
                order.pop_back();
            }
            if (order.empty()) {
                double best_time = std::numeric_limits<double>::infinity();
                std::optional<std::size_t> best;
                for (std::size_t index = 0; index < count; ++index) {
                    const auto& first = routes[origin][index];
                    const auto& home = routes[index][destination];
                    if (!ValidRoute(first) || !ValidRoute(home) ||
                        !SameGeometry(first, home)) {
                        continue;
                    }
                    const double candidate_time = edge_time(origin, index);
                    if (candidate_time < best_time) {
                        best = index;
                        best_time = candidate_time;
                    }
                }
                if (best) {
                    order.push_back(*best);
                    return_leg = routes[*best][destination];
                }
            }
        }
    }

    if (order.empty() || (return_pose && !return_leg)) {
        result.reachable = false;
        return result;
    }

    const auto order_time = [&](const std::vector<std::size_t>& candidate) {
        double total = 0.0;
        std::size_t from = origin;
        const VistaRoute* baseline = nullptr;
        for (const auto index : candidate) {
            const auto& route = routes[from][index];
            if (!ValidRoute(route) ||
                (baseline && !SameGeometry(*baseline, route))) {
                return std::numeric_limits<double>::infinity();
            }
            if (!baseline)
                baseline = &route;
            total += edge_time(from, index) + views[index].observation_time;
            from = index;
        }
        if (return_pose) {
            const auto& route = routes[from][destination];
            if (!ValidRoute(route) ||
                (baseline && !SameGeometry(*baseline, route))) {
                return std::numeric_limits<double>::infinity();
            }
            total += edge_time(from, destination);
        }
        return total;
    };
    if (!exact_used && options.use_two_opt && options.improvement_iterations &&
        order.size() > 2) {
        double best_time = order_time(order);
        for (std::size_t pass = 0; pass < options.improvement_iterations;
             ++pass) {
            bool improved = false;
            const std::size_t first = keep_first ? 1 : 0;
            for (std::size_t left = first; left + 1 < order.size(); ++left) {
                for (std::size_t right = left + 1; right < order.size();
                     ++right) {
                    if (cancel_checker && cancel_checker()) {
                        result.reachable = false;
                        return result;
                    }
                    auto candidate = order;
                    std::reverse(candidate.begin() + left,
                                 candidate.begin() + right + 1);
                    const double candidate_time = order_time(candidate);
                    if (candidate_time + 1e-9 < best_time) {
                        order = std::move(candidate);
                        best_time = candidate_time;
                        improved = true;
                    }
                }
            }
            if (!improved)
                break;
        }
        if (return_pose)
            return_leg = routes[order.back()][destination];
    }

    std::size_t previous = origin;
    for (const auto index : order) {
        const auto& leg = routes[previous][index];
        result.views.push_back(views[index]);
        result.legs.push_back(leg);
        result.distance += leg.length;
        result.total_time +=
            edge_time(previous, index) + views[index].observation_time;
        previous = index;
    }
    if (return_leg) {
        result.distance += return_leg->length;
        result.total_time += edge_time(previous, destination);
        result.return_leg = std::move(return_leg);
    }
    result.reachable = true;
    return result;
}

}  // namespace autonomy::task::exploration
