/* Copyright 2026 The Openbot Authors. */
#include <cmath>
#include <initializer_list>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

#include "autonomy/task/exploration/integration/task_domain_path_provider.hpp"
#include "autonomy/task/exploration/routing/visit_order_optimizer.hpp"

namespace e = autonomy::task::exploration;
namespace v = autonomy::planning::planner::vista;
namespace {

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

class DirectedPaths final : public e::PathProvider
{
public:
    void Add(int from, int to, double length) {
        lengths_[{from, to}] = length;
    }

    v::VistaRoute Route(const e::Pose2d& start,
                        const e::Pose2d& goal) const override {
        v::VistaRoute route;
        const auto found =
            lengths_.find({static_cast<int>(std::lround(start.x)),
                           static_cast<int>(std::lround(goal.x))});
        if (found == lengths_.end())
            return route;
        route.success = true;
        route.length = found->second;
        route.map_epoch = 7;
        route.geometry_revision = 11;
        v::VistaMapEvidence evidence;
        evidence.map_epoch = 7;
        evidence.geometry_revision = 11;
        evidence.frame_id = "map";
        route.evidence = evidence;
        return route;
    }

private:
    std::map<std::pair<int, int>, double> lengths_;
};

class FixedRouteProvider final : public e::PathProvider
{
public:
    explicit FixedRouteProvider(v::VistaRoute route)
        : route_(std::move(route)) {}

    v::VistaRoute Route(const e::Pose2d&, const e::Pose2d&) const override {
        return route_;
    }

    std::optional<e::RouteMapEvidence> Evidence() const override {
        return route_.evidence;
    }

private:
    v::VistaRoute route_;
};

::automsgs::msgs::geometry_msgs::Polygon Polygon(
    std::initializer_list<std::pair<float, float>> vertices) {
    ::automsgs::msgs::geometry_msgs::Polygon polygon;
    for (const auto& [x, y] : vertices) {
        auto* point = polygon.add_points();
        point->set_x(x);
        point->set_y(y);
    }
    return polygon;
}

e::Viewpoint View(std::uint64_t id, double x, double observation = 0) {
    e::Viewpoint view;
    view.id = id;
    view.pose = {x, 0, 0};
    view.observation_time = observation;
    return view;
}

void MissingFutureEdgeNeverBecomesFinite() {
    DirectedPaths paths;
    paths.Add(0, 1, 1);
    paths.Add(0, 2, 2);
    e::VisitOrderOptimizer optimizer;
    const auto result =
        optimizer.Optimize({0, 0, 0}, {View(1, 1), View(2, 2)}, paths, 1, 1);
    Require(result.reachable && result.views.size() == 1,
            "a feasible prefix remains available");
    Require(result.views.front().id == 1 && result.legs.size() == 1,
            "missing directed edge is never replaced");
    Require(std::abs(result.distance - 1) < 1e-12,
            "aggregate distance contains only the retained Vista leg");
}

void RequiredHomeMustBeReachable() {
    DirectedPaths paths;
    paths.Add(0, 1, 1);
    e::VisitOrderOptimizer optimizer;
    const auto result = optimizer.Optimize({0, 0, 0}, {View(1, 1)}, paths, 1, 1,
                                           e::Pose2d{9, 0, 0});
    Require(!result.reachable && result.views.empty(),
            "unreachable required home rejects the order");
}

void DirectedCostsAndEvidenceArePreserved() {
    DirectedPaths paths;
    paths.Add(0, 1, 8);
    paths.Add(0, 2, 1.5);
    paths.Add(2, 1, 2.5);
    paths.Add(1, 2, 20);
    paths.Add(1, 9, 3);
    paths.Add(2, 9, 30);
    e::VisitOrderOptimizer optimizer;
    const auto result =
        optimizer.Optimize({0, 0, 0}, {View(1, 1, 0.25), View(2, 2, 0.5)},
                           paths, 2, 1, e::Pose2d{9, 0, 0});
    Require(result.reachable && result.views.size() == 2,
            "directed feasible order is retained");
    Require(result.views[0].id == 2 && result.views[1].id == 1,
            "ordering uses the queried direction");
    Require(result.legs.size() == 2 && result.return_leg,
            "all Vista legs remain attached to their endpoints");
    Require(std::abs(result.distance - 7) < 1e-12,
            "distance is the exact sum of Vista legs");
    Require(std::abs(result.total_time - 4.25) < 1e-12,
            "time is composed from exact legs and observation dwell");
    for (const auto& leg : result.legs) {
        Require(leg.map_epoch == 7 && leg.geometry_revision == 11 &&
                    leg.evidence && leg.evidence->frame_id == "map",
                "route provenance is preserved");
    }
}

/** @brief Exact directed ordering beats nearest-neighbor greedy ordering. */
void ExactDirectedOrderBeatsGreedy() {
    DirectedPaths paths;
    paths.Add(0, 1, 1);
    paths.Add(0, 2, 2);
    paths.Add(0, 3, 50);
    paths.Add(1, 2, 1);
    paths.Add(1, 3, 2);
    paths.Add(2, 1, 1);
    paths.Add(2, 3, 100);
    paths.Add(3, 2, 100);
    paths.Add(1, 9, 100);
    paths.Add(2, 9, 100);
    paths.Add(3, 9, 1);
    const std::vector<e::Viewpoint> views{View(1, 1), View(2, 2), View(3, 3)};
    e::VisitOrderOptimizer optimizer;
    e::VisitOrderOptimizerOptions greedy_options;
    greedy_options.exact_maximum_nodes = 0;
    greedy_options.use_two_opt = false;
    const auto greedy =
        optimizer.Optimize({0, 0, 0}, views, paths, 1, 1, e::Pose2d{9, 0, 0},
                           false, {}, greedy_options);
    e::VisitOrderOptimizerOptions exact_options;
    exact_options.exact_maximum_nodes = 3;
    exact_options.use_two_opt = false;
    const auto exact =
        optimizer.Optimize({0, 0, 0}, views, paths, 1, 1, e::Pose2d{9, 0, 0},
                           false, {}, exact_options);
    Require(greedy.reachable && exact.reachable && greedy.views.size() == 3 &&
                exact.views.size() == 3,
            "both directed regimes must preserve the full feasible visit set");
    Require(exact.views[0].id == 2 && exact.views[1].id == 1 &&
                exact.views[2].id == 3,
            "Held-Karp did not recover the exact asymmetric visit order");
    Require(std::abs(greedy.distance - 103) < 1e-12 &&
                std::abs(exact.distance - 6) < 1e-12,
            "exact directed ordering did not beat nearest-neighbor greedy");
    Require(exact.legs.size() == 3 && exact.return_leg &&
                exact.return_leg->length == 1,
            "exact result lost immutable Vista leg evidence or return leg");
}

/** @brief Exact task domains reject unsafe Vista polylines without mutation. */
void TaskDomainValidatesCompleteVistaPolyline() {
    const auto allowed =
        Polygon({{0, 0}, {5, 0}, {5, 2}, {2, 2}, {2, 5}, {0, 5}});
    v::VistaRoute owned;
    owned.success = true;
    owned.length = 5;
    owned.map_epoch = 7;
    owned.geometry_revision = 11;
    v::VistaMapEvidence evidence;
    evidence.map_epoch = 7;
    evidence.geometry_revision = 11;
    evidence.frame_id = "map";
    owned.evidence = evidence;
    auto delegate = std::make_shared<FixedRouteProvider>(owned);
    e::TaskDomainPathProvider allowed_paths(delegate, {}, allowed, {});

    const auto concave_chord = allowed_paths.Route({1, 4, 0}, {4, 1, 0});
    Require(!concave_chord.success && concave_chord.length == 5 &&
                concave_chord.evidence &&
                concave_chord.evidence->geometry_revision == 11,
            "concave-domain chord was admitted or Vista evidence was mutated");
    Require(allowed_paths.Route({2, 4, 0}, {2, 2, 0}).success,
            "allowed-polygon boundary traversal was rejected");

    e::TaskDomainPathProvider bounded_paths(
        delegate, autonomy::perception::Bounds2d{0, 0, 5, 5}, {}, {});
    Require(!bounded_paths.Route({1, 1, 0}, {6, 1, 0}).success,
            "legacy rectangular task scope did not reject an endpoint escape");
    Require(bounded_paths.Route({0, 1, 0}, {5, 1, 0}).success,
            "closed rectangular task boundary traversal was rejected");

    const auto no_go = Polygon({{2, 2}, {3, 2}, {3, 3}, {2, 3}});
    e::TaskDomainPathProvider excluded_paths(delegate, {}, {}, {no_go});
    Require(!excluded_paths.Route({1, 2, 0}, {4, 2, 0}).success,
            "closed no-go boundary contact was admitted");
    Require(!excluded_paths.Route({1, 2.5, 0}, {4, 2.5, 0}).success,
            "no-go interior crossing was admitted");
    Require(excluded_paths.Route({1, 1, 0}, {4, 1, 0}).success,
            "route disjoint from no-go polygon was rejected");

    owned.path.poses.push_back(
        {std::numeric_limits<double>::quiet_NaN(), 1, 0});
    e::TaskDomainPathProvider malformed_paths(
        std::make_shared<FixedRouteProvider>(owned), {}, {}, {});
    const auto malformed = malformed_paths.Route({0, 0, 0}, {1, 1, 0});
    Require(
        !malformed.success && malformed.path.poses.size() == 1 &&
            malformed.evidence,
        "nonfinite Vista geometry did not fail closed with evidence intact");
}

void KeepFirstCannotForceMissingEdgeAndCancellationStops() {
    DirectedPaths paths;
    paths.Add(0, 2, 1);
    e::VisitOrderOptimizer optimizer;
    auto result = optimizer.Optimize({0, 0, 0}, {View(1, 1), View(2, 2)}, paths,
                                     1, 1, {}, true);
    Require(result.reachable && result.views.size() == 1 &&
                result.views.front().id == 2,
            "keep-first does not fabricate an infeasible first edge");
    std::size_t checks = 0;
    result = optimizer.Optimize({0, 0, 0}, {View(1, 1), View(2, 2)}, paths, 1,
                                1, {}, false, [&] { return ++checks > 1; });
    Require(!result.reachable && result.views.empty(),
            "cancellation clears partial ordering output");
}

}  // namespace

int main() {
    try {
        MissingFutureEdgeNeverBecomesFinite();
        RequiredHomeMustBeReachable();
        DirectedCostsAndEvidenceArePreserved();
        ExactDirectedOrderBeatsGreedy();
        TaskDomainValidatesCompleteVistaPolyline();
        KeepFirstCannotForceMissingEdgeAndCancellationStops();
        std::cout << "PASS visit_order_optimizer\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL visit_order_optimizer: " << error.what() << '\n';
        return 1;
    }
}
