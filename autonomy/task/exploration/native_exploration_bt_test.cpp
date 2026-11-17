/* Copyright 2026 The Openbot Authors. */
#include <iostream>
#include <string>
#include "exploration_test_support.hpp"
#include "autonomy/task/exploration/plugins/exploration_tree.hpp"

namespace e = autonomy::task::exploration;
namespace b = autonomy::task::plugins::exploration;
namespace p = autonomy::perception;
using namespace exploration_test;

namespace {
struct Nodes {
    explicit Nodes(Fixture& fixture) {
        context = std::make_shared<b::ExplorationBtContext>();
        context->session = std::shared_ptr<e::ExplorationSession>(
            fixture.session.get(), [](e::ExplorationSession*) {});
        context->clock = [this] { return now; };
        board = BT::Blackboard::create();
        board->set("exploration_context", context);
        autonomy::task::RegisterBuiltinBtNodes(factory);
    }
    BT::Tree Make(const std::string& tag) {
        return factory.createTreeFromText(
            "<root BTCPP_format=\"4\"><BehaviorTree ID=\"Test\">" + tag +
            "</BehaviorTree></root>", board);
    }
    BT::NodeStatus Await(Fixture& fixture, BT::Tree& tree, double stamp) {
        now = stamp;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(3);
        auto status = BT::NodeStatus::RUNNING;
        while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() < deadline) {
            fixture.session->Tick(now);
            status = tree.tickOnce();
            if (status == BT::NodeStatus::RUNNING)
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        Require(status != BT::NodeStatus::RUNNING, "native phase reaches a bounded result");
        return status;
    }
    double now = 1.01;
    std::shared_ptr<b::ExplorationBtContext> context;
    BT::Blackboard::Ptr board;
    BT::BehaviorTreeFactory factory;
};

void RegistryAndNativeFirstTick() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32, e::ExplorationDriver::kBehaviorTree);
    Nodes nodes(fixture);
    for (const auto* name : {"SensorReady", "UpdateExploration", "SelectObservationPose",
        "AdaptObservationPose", "ObserveAtPose", "CheckExplorationCompletion",
        "ReturnToStart", "SaveExplorationMap"})
        Require(nodes.factory.manifests().count(name) == 1, "all eight native ids are registered");
    auto select = nodes.Make("<SelectObservationPose observation_goal=\"{selected}\"/>");
    Require(dynamic_cast<autonomy::task::plugins::BtStatefulAction*>(select.rootNode()) != nullptr,
            "node inherits the actual native stateful base");
    Require(select.tickOnce() == BT::NodeStatus::RUNNING &&
            !fixture.session->Snapshot().active_phase && fixture.submissions.empty(),
            "native first tick does not request a phase or submit motion");
    Require(nodes.Await(fixture, select, 1.02) == BT::NodeStatus::SUCCESS,
            "later native ticks select an observation intent");
    e::ObservationGoal selected;
    Require(nodes.board->get("selected", selected) && selected.goal_id == 42 &&
            fixture.submissions.empty(), "selection publishes a typed goal without execution");
    auto condition = nodes.Make("<SensorReady/>");
    Require(dynamic_cast<autonomy::task::plugins::BtCondition*>(condition.rootNode()) != nullptr &&
            condition.tickOnce() == BT::NodeStatus::SUCCESS, "condition uses actual native lifecycle");
    auto update = nodes.Make("<UpdateExploration/>");
    Require(dynamic_cast<autonomy::task::plugins::BtSyncAction*>(update.rootNode()) != nullptr &&
            update.tickOnce() == BT::NodeStatus::SUCCESS, "bounded update is a native synchronous action");
}

void SeparateStagesAndActualObservation() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32, e::ExplorationDriver::kBehaviorTree);
    Nodes nodes(fixture);
    auto select = nodes.Make("<SelectObservationPose/>");
    auto adapt = nodes.Make("<AdaptObservationPose adapted_goal=\"{adapted}\"/>");
    auto observe = nodes.Make("<ObserveAtPose phase_status=\"{phase}\"/>");
    Require(nodes.Await(fixture, select, 1.02) == BT::NodeStatus::SUCCESS &&
            nodes.Await(fixture, adapt, 1.03) == BT::NodeStatus::SUCCESS &&
            fixture.submissions.empty(), "select and adapt remain independent of execution");
    nodes.now = 1.04;
    Require(observe.tickOnce() == BT::NodeStatus::RUNNING && fixture.submissions.empty(),
            "observe first native tick only starts its lifecycle");
    Require(observe.tickOnce() == BT::NodeStatus::RUNNING && fixture.submissions.size() == 1,
            "only observe submits navigation");
    for (unsigned i = 0; i < 5; ++i) observe.tickOnce();
    Require(fixture.submissions.size() == 1, "repeated ticks do not duplicate navigation");
    fixture.Arrive(1.05);
    Require(observe.tickOnce() == BT::NodeStatus::RUNNING, "arrival alone is not observation success");
    fixture.Observe(1.06); fixture.session->Tick(1.06);
    Require(observe.tickOnce() == BT::NodeStatus::RUNNING, "one frame cannot replace required dwell");
    fixture.Observe(1.12); fixture.session->Tick(1.12);
    Require(observe.tickOnce() == BT::NodeStatus::SUCCESS &&
            nodes.board->get<std::string>("phase") == "SUCCEEDED",
            "fresh independent observation and dwell complete the native phase");
    auto completion = nodes.Make("<CheckExplorationCompletion/>");
    Require(completion.tickOnce() == BT::NodeStatus::FAILURE &&
            fixture.session->Snapshot().state != e::ExplorationState::kFinished,
            "completion condition failure does not fail the task");
}

void HaltOwnershipAndBusyInitialization() {
    Fixture fixture({0}, .05, {}, false, true, 10, 32, e::ExplorationDriver::kBehaviorTree);
    Nodes nodes(fixture);
    auto select = nodes.Make("<SelectObservationPose/>");
    auto adapt = nodes.Make("<AdaptObservationPose/>");
    Require(nodes.Await(fixture, select, 1.02) == BT::NodeStatus::SUCCESS &&
            nodes.Await(fixture, adapt, 1.03) == BT::NodeStatus::SUCCESS, "prepare an observation");
    auto observe = nodes.Make("<ObserveAtPose/>");
    nodes.now = 1.04; observe.tickOnce(); observe.tickOnce();
    const auto owned = *fixture.session->Snapshot().active_phase;
    auto unrelated = nodes.Make("<ReturnToStart/>");
    Require(unrelated.tickOnce() == BT::NodeStatus::RUNNING &&
            unrelated.tickOnce() == BT::NodeStatus::RUNNING, "busy initialization waits without a ticket");
    unrelated.haltTree(); fixture.session->Tick(1.05);
    Require(fixture.cancellation_count == 0 && fixture.session->Snapshot().active_phase &&
            fixture.session->Snapshot().active_phase->SameRequest(owned),
            "halting a waiting node cannot cancel another node's phase");
    observe.haltTree(); fixture.session->Tick(1.06);
    Require(fixture.cancellation_count == 1 &&
            fixture.session->PhaseState(owned).status == e::PhaseStatus::kCanceled,
            "halting the owner cancels only its navigation and ticket");
}

void ConditionsRejectStalePoseAndMissingContext() {
    Fixture fixture({0}, .05, {}, false, true, .05, 32, e::ExplorationDriver::kBehaviorTree);
    Nodes nodes(fixture);
    auto ready = nodes.Make("<SensorReady/>");
    fixture.Observe(1.1); fixture.session->Tick(1.1);
    Require(fixture.session->Snapshot().sensors.ready && ready.tickOnce() == BT::NodeStatus::FAILURE,
            "fresh camera input cannot hide stale localization");
    nodes.board->unset("exploration_context");
    Require(ready.tickOnce() == BT::NodeStatus::FAILURE, "missing context is a condition failure");
    auto select = nodes.Make("<SelectObservationPose/>");
    Require(select.tickOnce() == BT::NodeStatus::RUNNING &&
            select.tickOnce() == BT::NodeStatus::FAILURE && fixture.submissions.empty(),
            "missing context cannot submit a phase or motion");
}

void OldHaltCannotCancelSaving() {
    Fixture fixture({}, .05, std::make_shared<EmptyComplete>(), true, false,
                    10, 32, e::ExplorationDriver::kBehaviorTree);
    Nodes nodes(fixture);
    auto select = nodes.Make("<SelectObservationPose/>");
    Require(nodes.Await(fixture, select, 1.02) == BT::NodeStatus::FAILURE &&
            fixture.session->Snapshot().completion_confirmed,
            "empty complete selection has a separate completion result");
    auto save = nodes.Make("<SaveExplorationMap/>");
    nodes.now = 1.03; save.tickOnce(); save.tickOnce();
    const auto saving = *fixture.session->Snapshot().active_phase;
    // Deliver a delayed halt hook from the old native action invocation.
    dynamic_cast<autonomy::task::plugins::BtStatefulAction*>(select.rootNode())->onHalted();
    fixture.session->Tick(1.04);
    Require(fixture.session->Snapshot().active_phase &&
            fixture.session->Snapshot().active_phase->SameRequest(saving) && fixture.refresh_request,
            "old select halt preserves the new save owner");
    e::MappingReply refreshed{*fixture.refresh_request, true, true, {}};
    refreshed.fused_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    fixture.refresh_reply = refreshed; fixture.session->Tick(1.05);
    Require(fixture.save_request.has_value(), "refresh ack starts the explicit save");
    e::MappingReply saved{*fixture.save_request, true, true, {}};
    saved.fused_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
    fixture.save_reply = saved; fixture.session->Tick(1.06);
    Require(save.tickOnce() == BT::NodeStatus::SUCCESS &&
            fixture.session->Snapshot().result.saved == e::MapSaveOutcome::kSaved,
            "native save succeeds only after a matching acknowledgement");
}

class ObserveThenComplete final : public e::ExplorationStrategy {
public:
    ObserveThenComplete() : first_({0, .3}) {}
    void Reset() override { first_.Reset(); selected_ = false; }
    e::StrategyDecision Select(const e::Pose2d& robot, const e::Pose2d& home,
        const e::MapReadView& map, const e::CoverageState& coverage, p::SensorMask sensors,
        double now, const std::function<bool()>& canceled) override {
        if (!selected_) {
            selected_ = true;
            return first_.Select(robot, home, map, coverage, sensors, now, canceled);
        }
        e::StrategyDecision result;
        result.grid_fallback_checked = result.sweep_checked = true;
        return result;
    }
private:
    OneGoal first_;
    bool selected_ = false;
};

void FullNativeTreeObservationReturnSave() {
    Fixture fixture({0, .3}, .05, std::make_shared<ObserveThenComplete>(), true, true,
                    10, 32, e::ExplorationDriver::kBehaviorTree, true);
    Nodes nodes(fixture);
    e::ExplorationTree tree(nodes.context);
    std::uint64_t replied_navigation = 0, replied_refresh = 0, replied_save = 0;
    bool returned_with_navigation = false;
    for (unsigned i = 0; i < 500 &&
         fixture.session->Snapshot().state != e::ExplorationState::kFinished; ++i) {
        nodes.now = 1.02 + i * .01;
        // The camera has a finite field of view and an independent acquisition
        // rate. Before the observation action, healthy rays face elsewhere;
        // they must not pre-complete the unit that this test asks it to observe.
        if (i % 10 == 0) {
            fixture.Observe(nodes.now, !fixture.submissions.empty());
            auto map = fixture.reader.Read(fixture.grid, nodes.now, 0, fixture.sequence);
            map.mapping_synchronized = true;
            map.observation_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
            fixture.session->PushMap(map);
        }
        if (!fixture.submissions.empty()) {
            const auto& submitted = fixture.submissions.back();
            fixture.Pose(nodes.now, submitted.first.yaw);
            if (submitted.second.request_id != replied_navigation) {
                returned_with_navigation |= fixture.session->Snapshot().finishing;
                fixture.reply = e::NavigationReply{submitted.second, true};
                replied_navigation = submitted.second.request_id;
            }
        } else fixture.Pose(nodes.now, 0);
        if (fixture.refresh_request && fixture.refresh_request->request_id != replied_refresh) {
            e::MappingReply reply{*fixture.refresh_request, true, true, {}};
            reply.fused_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
            fixture.refresh_reply = reply;
            replied_refresh = reply.version.request_id;
        }
        if (fixture.save_request && fixture.save_request->request_id != replied_save) {
            e::MappingReply reply{*fixture.save_request, true, true, {}};
            reply.fused_cut.Include(p::SensorKind::kRgbd, fixture.sequence);
            fixture.save_reply = reply;
            replied_save = reply.version.request_id;
        }
        tree.Tick();
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    const auto snapshot = fixture.session->Snapshot();
    if (snapshot.state != e::ExplorationState::kFinished ||
        !snapshot.result.Success(true, true) || snapshot.observed_view_count != 2 ||
        !returned_with_navigation || replied_save == 0) {
        std::cerr << "full native tree state=" << static_cast<int>(snapshot.state)
                  << " exploration=" << static_cast<int>(snapshot.result.exploration)
                  << " return=" << static_cast<int>(snapshot.result.returned)
                  << " save=" << static_cast<int>(snapshot.result.saved)
                  << " views=" << snapshot.observed_view_count
                  << " submissions=" << fixture.submissions.size()
                  << " reason=" << snapshot.reason << '\n';
    }
    Require(snapshot.state == e::ExplorationState::kFinished &&
            snapshot.result.Success(true, true) && snapshot.observed_view_count == 2 &&
            returned_with_navigation && replied_save != 0,
            "native tree completes actual views, explicit return and acknowledged save");
    Require(fixture.submissions.size() == 3, "two views and one return use the same submission owner");
}
}  // namespace

int main() {
    try {
        RegistryAndNativeFirstTick(); std::cout << "PASS native_registry_first_tick\n";
        SeparateStagesAndActualObservation(); std::cout << "PASS native_separate_stages_actual_observation\n";
        HaltOwnershipAndBusyInitialization(); std::cout << "PASS native_halt_owner_busy_init\n";
        ConditionsRejectStalePoseAndMissingContext(); std::cout << "PASS native_conditions_pose_context\n";
        OldHaltCannotCancelSaving(); std::cout << "PASS native_old_halt_save_ack\n";
        FullNativeTreeObservationReturnSave(); std::cout << "PASS native_full_tree_views_return_save\n";
    } catch (const std::exception& error) {
        std::cerr << "FAIL native exploration BT: " << error.what() << '\n'; return 1;
    }
}
