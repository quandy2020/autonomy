/* Copyright 2026 The Navigation Authors (navigation) */
#include <chrono>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include "navigation/planning/plugins/policy/simulation/policy.hpp"
using namespace navigation;
void Check(bool c, const char* m) { if (!c) throw std::runtime_error(m); }

// Stands in for the ONNX depth detector so the fallback logic is testable.
class FakeSource : public planning::FrontierSourceInterface {
public:
    bool Ready() const override { return ready; }
    std::vector<planning::Waypoint> Detect(const observation::Observation&,
                                           const common::Pose2D&) override {
        ++calls;
        return waypoints;
    }
    bool ready{true};
    int calls{0};
    std::vector<planning::Waypoint> waypoints;
};
int main() {
    try {
        auto now = std::chrono::steady_clock::time_point{};
        planning::simulation::Policy policy({}, [&] { return now; });
        config::PolicyOptions options; options.set_name("simulation");
        options.mutable_exploration()->set_max_history_entries(2);
        Check(policy.Configure(options), "configuration failed");
        observation::Observation obs; obs.set_map_frame("map");
        auto* pose = obs.mutable_odom()->mutable_pose()->mutable_pose()->mutable_pose();
        pose->mutable_orientation()->set_w(1);
        auto* depth = obs.mutable_depth(); depth->set_width(10); depth->set_height(10); depth->set_encoding("32FC1");
        const float values[100] = {0};
        depth->set_data(reinterpret_cast<const char*>(values), sizeof(values));
        // Pose arrival still needs bounded, prompt final-heading alignment.
        GoalSpec align; align.set_type(GoalSpec::TYPE_POSE);
        align.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(1);
        double yaw = .6;
        for (int i = 0; i < 10; ++i) {
            pose->mutable_orientation()->set_z(std::sin(yaw / 2));
            pose->mutable_orientation()->set_w(std::cos(yaw / 2));
            auto result = policy.Query(obs, {}, align, {});
            Check(result.Ok(), "heading query failed");
            const auto action = result.Value().chunk().actions(0);
            Check(action.x() == 0 && std::abs(action.theta()) <= .800001,
                  "heading alignment translated or exceeded angular limit");
            yaw += action.theta() * .1;
        }
        Check(std::abs(yaw) < .1, "final heading did not converge within one second");
        pose->mutable_orientation()->set_z(std::sin(1.5));
        pose->mutable_orientation()->set_w(std::cos(1.5));
        align.mutable_pose()->mutable_pose()->mutable_orientation()->set_z(std::sin(-1.5));
        align.mutable_pose()->mutable_pose()->mutable_orientation()->set_w(std::cos(-1.5));
        auto wrapped = policy.Query(obs, {}, align, {}).Value().chunk().actions(0);
        Check(wrapped.x() == 0 && wrapped.theta() > 0 && wrapped.theta() <= .800001,
              "final heading took the long turn across pi");
        pose->mutable_orientation()->set_z(0); pose->mutable_orientation()->set_w(1);
        GoalSpec goal; goal.set_type(GoalSpec::TYPE_EXPLORATION);
        const auto query = [&] {
            auto result = policy.Query(obs, {}, goal, {});
            Check(result.Ok(), "query failed");
            return result.Value().chunk().actions(0);
        };
        Check(query().x() == 0 && query().theta() > 0, "empty frontier did not scan in place");
        // Simulate a full signed odometry turn, rather than elapsing wall time.
        for (int i = 1; i <= 14; ++i) {
            pose->mutable_orientation()->set_z(std::sin(i * .25)); pose->mutable_orientation()->set_w(std::cos(i * .25));
            const auto action = query(); Check(action.x() == 0, "invalid depth allowed translation");
        }
        std::vector<float> far(100, 5); depth->set_data(reinterpret_cast<const char*>(far.data()), far.size() * sizeof(float));
        Check(query().x() > 0 && policy.ExplorationMode() == "depth_cruise", "full scan did not release cruise");
        std::vector<float> near(100, .3F); depth->set_data(reinterpret_cast<const char*>(near.data()), near.size() * sizeof(float));
        Check(query().x() == 0 && query().theta() > 0, "blocked fallback reversed/translated");
        depth->set_data(reinterpret_cast<const char*>(far.data()), far.size() * sizeof(float));
        policy.Reset(); pose->mutable_orientation()->set_z(0); pose->mutable_orientation()->set_w(1);
        auto* grid = obs.mutable_map(); grid->mutable_header()->set_frame_id("map");
        grid->mutable_info()->set_width(24); grid->mutable_info()->set_height(24); grid->mutable_info()->set_resolution(.5);
        grid->mutable_info()->mutable_origin()->mutable_position()->set_x(-6);
        grid->mutable_info()->mutable_origin()->mutable_position()->set_y(-6);
        grid->mutable_info()->mutable_origin()->mutable_orientation()->set_w(1);
        for (int y = 0; y < 24; ++y) for (int x = 0; x < 24; ++x)
            grid->add_data(x >= 6 && x < 18 && y >= 6 && y < 18 ? 0 : -1);
        Check(query().x() == 0 && policy.ExplorationMode() == "invalid_map", "unvalidated map allowed motion");
        obs.set_validity(observation::OBSERVATION_FLAG_MAP_VALID);
        query(); Check(policy.ExplorationTarget().has_value(), "map target not selected");
        // A stuck target expires; it is not immediately replayed from a static map.
        now += std::chrono::seconds(9); query();
        Check(policy.ExplorationHistorySize() == 1 && !policy.ExplorationTarget(), "stuck target replayed");
        now += std::chrono::seconds(121); query();
        Check(policy.ExplorationTarget().has_value(), "expired history suppressed target forever");
        // Arrival retires a frontier; changing free boundary yields a new target.
        for (int k = 0; k < 6; ++k) {
            if (policy.ExplorationTarget()) {
                pose->mutable_position()->set_x(policy.ExplorationTarget()->x());
                pose->mutable_position()->set_y(policy.ExplorationTarget()->y());
                query();
            }
            Check(policy.ExplorationHistorySize() <= 2, "frontier history grew beyond configured bound");
            for (int y = std::max(1, 5-k); y < std::min(23, 19+k); ++y) for (int x = std::max(1, 5-k); x < std::min(23, 19+k); ++x) grid->set_data(y * 24 + x, 0);
            now += std::chrono::seconds(1); query();
        }
        Check(policy.ExplorationHistorySize() == 2, "history eviction scenario did not fill its capacity");
        grid->mutable_header()->set_frame_id("odom");
        Check(query().x() == 0 && policy.ExplorationMode() == "map_frame_mismatch", "map frame mismatch ignored");
        policy.Reset(); Check(policy.ExplorationHistorySize() == 0 && !policy.ExplorationTarget(), "reset retained old goal state");
        // An absolute budget also expires a target before the idle timeout.
        pose->mutable_position()->set_x(0); pose->mutable_position()->set_y(0);
        grid->mutable_header()->set_frame_id("map"); grid->clear_data();
        for (int y=0; y<24; ++y) for (int x=0; x<24; ++x)
            grid->add_data(x>=6 && x<18 && y>=6 && y<18 ? 0 : -1);
        options.mutable_exploration()->set_no_progress_timeout_s(100);
        options.mutable_exploration()->set_target_timeout_s(1);
        Check(policy.Configure(options), "budget configuration failed");
        query(); Check(policy.ExplorationTarget().has_value(), "budget target missing");
        now += std::chrono::seconds(2); query();
        Check(!policy.ExplorationTarget() && policy.ExplorationHistorySize()==1, "absolute target budget ignored");
        // An exhausted map falls back to the depth model; map candidates still win.
        {
            planning::simulation::Policy fallback({}, [&] { return now; });
            config::PolicyOptions fallback_options; fallback_options.set_name("simulation");
            Check(fallback.Configure(fallback_options), "fallback configuration failed");
            auto fake = std::make_unique<FakeSource>();
            auto* fake_ptr = fake.get();
            common::Pose2D ahead; ahead.set_x(3); ahead.set_y(0);
            fake_ptr->waypoints.push_back(planning::Waypoint{ahead, 0.15});
            fallback.SetFrontierSourceForTest(std::move(fake));

            observation::Observation scene; scene.set_map_frame("map");
            auto* scene_pose = scene.mutable_odom()->mutable_pose()->mutable_pose()->mutable_pose();
            scene_pose->mutable_orientation()->set_w(1);
            auto* scene_depth = scene.mutable_depth();
            scene_depth->set_width(10); scene_depth->set_height(10); scene_depth->set_encoding("32FC1");
            std::vector<float> open(100, 5);
            scene_depth->set_data(reinterpret_cast<const char*>(open.data()), open.size() * sizeof(float));
            auto* scene_grid = scene.mutable_map(); scene_grid->mutable_header()->set_frame_id("map");
            scene_grid->mutable_info()->set_width(24); scene_grid->mutable_info()->set_height(24);
            scene_grid->mutable_info()->set_resolution(.5);
            scene_grid->mutable_info()->mutable_origin()->mutable_position()->set_x(-6);
            scene_grid->mutable_info()->mutable_origin()->mutable_position()->set_y(-6);
            scene_grid->mutable_info()->mutable_origin()->mutable_orientation()->set_w(1);
            for (int i = 0; i < 24 * 24; ++i) scene_grid->add_data(0);
            scene.set_validity(observation::OBSERVATION_FLAG_MAP_VALID);
            GoalSpec explore; explore.set_type(GoalSpec::TYPE_EXPLORATION);
            const auto step = [&] {
                auto result = fallback.Query(scene, {}, explore, {});
                Check(result.Ok(), "fallback query failed");
                return result.Value().chunk().actions(0);
            };
            const auto action = step();
            Check(fallback.ExplorationFrontierSource() ==
                      planning::simulation::Policy::FrontierSourceKind::kModel,
                  "fully explored map did not fall back to the depth model");
            Check(fallback.ExplorationMode() == "model_target" && fallback.ExplorationTarget(),
                  "fallback target not selected");
            Check(fake_ptr->calls == 1, "fallback did not run the depth detector");
            Check(action.x() > 0, "fallback target did not command motion");
            step();
            Check(fake_ptr->calls == 1, "fallback detector ran without pose or period change");

            // Unknown cells in the map keep the map authoritative.
            planning::simulation::Policy mapped({}, [&] { return now; });
            Check(mapped.Configure(fallback_options), "mapped configuration failed");
            auto map_fake = std::make_unique<FakeSource>();
            auto* map_fake_ptr = map_fake.get();
            map_fake_ptr->waypoints.push_back(planning::Waypoint{ahead, 0.15});
            mapped.SetFrontierSourceForTest(std::move(map_fake));
            for (int y = 0; y < 24; ++y) for (int x = 0; x < 24; ++x)
                scene_grid->set_data(y * 24 + x, x >= 6 && x < 18 && y >= 6 && y < 18 ? 0 : -1);
            auto result = mapped.Query(scene, {}, explore, {});
            Check(result.Ok(), "mapped query failed");
            Check(mapped.ExplorationFrontierSource() ==
                      planning::simulation::Policy::FrontierSourceKind::kMap,
                  "map frontier was overridden by the depth model");
            Check(mapped.ExplorationMode() == "map_target", "map target not selected");
            Check(map_fake_ptr->calls == 0, "depth detector ran while the map had candidates");

            // A start cell inside the clearance inflation must not discard a depth
            // target: the map planner rejects the robot's own cell when it is
            // closer than clearance to an obstacle, which is normal mid-exploration.
            planning::simulation::Policy tight({}, [&] { return now; });
            Check(tight.Configure(fallback_options), "tight configuration failed");
            auto tight_fake = std::make_unique<FakeSource>();
            auto* tight_ptr = tight_fake.get();
            tight_ptr->waypoints.push_back(planning::Waypoint{ahead, 0.15});
            tight.SetFrontierSourceForTest(std::move(tight_fake));
            // A solid wall keeps the depth waypoint unreachable on the map, which
            // is the case the direct-steering fallback exists for.
            for (int i = 0; i < 24 * 24; ++i) scene_grid->set_data(i, 0);
            for (int y = 0; y < 24; ++y) scene_grid->set_data(y * 24 + 15, 100);
            auto tight_result = tight.Query(scene, {}, explore, {});
            Check(tight_result.Ok(), "tight query failed");
            Check(tight.ExplorationMode() == "model_direct" && tight.ExplorationTarget(),
                  "missing map path discarded a depth target");
            Check(tight_result.Value().chunk().actions(0).x() > 0,
                  "depth target without a map path did not command motion");
            // A low obstacle below the camera axis must stop translation too.
            std::vector<float> low(100, 5);
            for (int v = 5; v < 10; ++v) for (int u = 0; u < 10; ++u) low[v * 10 + u] = 0.3F;
            scene_depth->set_data(reinterpret_cast<const char*>(low.data()),
                                  low.size() * sizeof(float));
            Check(tight.Query(scene, {}, explore, {}).Value().chunk().actions(0).x() == 0,
                  "low obstacle allowed translation");
        }
        std::cout << "PASS scan, depth gate, map target, depth fallback, timeout, expiry, "
                     "bounded history and reset\n";
    } catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
