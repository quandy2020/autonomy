/* Copyright 2026 The Openbot Authors. */
/** @file native_route_service_test.cpp
 * @brief Real PlannerServer/client routes, process boundaries and reply lifetime regressions.
 * @details Controlled map prefixes validate propagation, not real mapper fusion.
 */
#include <atomic>
#include <chrono>
#include <future>
#include <iostream>
#include <limits>
#include <mutex>
#include <spawn.h>
#include <stdexcept>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

#include "autolink/autolink.hpp"
#include "autolink/transport/receiver/hybrid_receiver.hpp"
#include "autolink/transport/transmitter/shm_transmitter.hpp"
#include "autonomy/planning/planner_server.hpp"
#include "autonomy/planning/planner/vista/native_route_protocol.hpp"
#include "autonomy/planning/planner/vista/vista_planner.hpp"
#include "autonomy/task/exploration/integration/native_vista_path_provider.hpp"

extern char** environ;
namespace {
namespace v = autonomy::planning::planner::vista;
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace n = autonomy::navigation;
using Query = automsgs::srvs::GetPlan;
using namespace std::chrono_literals;
const n::Pose2d kStart{1, 1, -.2}, kGoal{5, 1, .7};  ///< Exact native query endpoints.
std::string executable;  ///< Real CTest binary used for the separate client process.

/** @brief Fail an actual native state or reply invariant. */
void Require(bool value, const char* reason) { if (!value) throw std::runtime_error(reason); }
/** @brief Await an actual transition with a finite steady-clock bound. */
template <typename Predicate>
void Await(Predicate predicate, const char* reason) {
    const auto deadline = std::chrono::steady_clock::now() + 5s;
    while (!predicate()) {
        Require(std::chrono::steady_clock::now() < deadline, reason);
        std::this_thread::sleep_for(1ms);
    }
}

/** @brief Make the actual server with a single configured graph, without map layers. */
autonomy::planning::proto::PlannerOptions Options() {
    autonomy::planning::proto::PlannerOptions options;
    options.add_planner_plugins("vista_planner"); options.set_default_planner_id("vista_planner");
    options.mutable_vista()->set_horizontal_margin(0);
    options.mutable_vista()->set_maximum_nodes(8);
    auto* map = options.mutable_costmap();
    map->set_enabled(true); map->set_name("route_service_costmap"); map->set_frame_id("map");
    map->set_resolution(.2); map->set_width(8); map->set_height(6);
    map->set_robot_radius(.1); map->add_plugins("none");
    return options;
}

/** @brief Controlled source versions belong to a real immutable map/graph owner. */
struct MapSource {
    std::mutex mutex;  ///< Serializes test map replacement against server reads.
    p::MapReadView view;  ///< Current immutable owned view.
    MapSource() {
        auto grid = std::make_shared<n::GridMap>(40, 30, .2);
        grid->Fill({n::CellState::kFree, 0});
        view.grid = grid; view.frame_id = "map"; view.source_stamp = 1;
        view.map_epoch = 4; view.geometry_revision = 7; view.observation_watermark = 99;
        view.observation_cut.Include(p::SensorKind::kLidar, 17);
        view.observation_cut.Include(p::SensorKind::kRgbd, 31);
        view.observation_baseline.Include(p::SensorKind::kLidar, 2);
        view.observation_baseline.Include(p::SensorKind::kRgbd, 3);
        view.observation_recovery_generation = 6;
    }
    /** @brief Read one independently owned source snapshot. */
    p::MapReadView Read() { std::lock_guard<std::mutex> lock(mutex); return view; }
    /** @brief Install blocked geometry without modifying the native map domain. */
    void BlockStart() {
        std::lock_guard<std::mutex> lock(mutex);
        auto grid = std::make_shared<n::GridMap>(*view.grid);
        const auto cell = grid->WorldToCell(kStart);
        Require(cell.has_value(), "source start is inside geometry");
        grid->Set(cell->x, cell->y, {n::CellState::kOccupied, 1});
        view.grid = grid; ++view.geometry_revision; view.dirty_regions = {{.5, .5, 1.5, 1.5}};
    }
};

/** @brief Verify that metadata is the source's real independent sensor prefix. */
void CheckEvidence(const v::VistaMapEvidence& evidence) {
    Require(evidence.frame_id == "map" && evidence.map_epoch == 4 && evidence.geometry_revision == 7,
            "route frame/epoch/revision came from the actual graph owner");
    Require(evidence.watermark == 99 && evidence.observation_cut.present == 3 &&
            evidence.observation_cut.sequence == std::array<std::uint64_t, 2>{17, 31} &&
            evidence.observation_baseline.sequence == std::array<std::uint64_t, 2>{2, 3} &&
            evidence.observation_recovery_generation == 6,
            "two source prefixes and recovered baseline were propagated without a scalar substitute");
}

/** @brief Guard a real separately spawned client against failure/timeout leaks. */
struct Child {
    pid_t pid = -1;  ///< Actual spawned client process id.
    ~Child() { if (pid > 0) { kill(pid, SIGKILL); (void)waitpid(pid, nullptr, 0); } }
};

/** @brief Separate task client process uses native discovery, serialization and transport. */
void CrossProcess() {
    Child child;
    char mode[] = "--route-client";
    char* args[] = {executable.data(), mode, nullptr};
    Require(posix_spawn(&child.pid, executable.c_str(), nullptr, nullptr, args, environ) == 0,
            "actual native route client process spawned");
    int status = 0;
    const auto deadline = std::chrono::steady_clock::now() + 12s;
    for (;;) {
        const auto waited = waitpid(child.pid, &status, WNOHANG);
        if (waited == child.pid) { child.pid = -1; break; }
        Require(waited >= 0 && std::chrono::steady_clock::now() < deadline,
                "separate native route client exited within its transport bound");
        std::this_thread::sleep_for(1ms);
    }
    Require(WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "separate native client consumed a certified route from PlannerServer");
}

/** @brief Exercise the production server endpoint and preserve shared graph ownership. */
void ServerAndProcess() {
    auto source = std::make_shared<MapSource>();
    autonomy::planning::PlannerServer server(Options());
    auto plugin = std::dynamic_pointer_cast<v::VistaPlanner>(server.GetPlanner("vista_planner"));
    Require(plugin != nullptr, "actual registry holds the Vista plugin");
    const auto graph = plugin->SharedRoutePlanner();
    plugin->SetMapViewSource([source] { return std::optional<p::MapReadView>{source->Read()}; });
    auto node = autolink::CreateNode("native_vista_route_client");
    e::NativeVistaPathProvider provider(node);
    std::optional<v::VistaMapEvidence> evidence;
    Await([&] { evidence = provider.Evidence(); return evidence.has_value(); }, "actual route service discovered");
    CheckEvidence(*evidence);
    const auto route = provider.Route(kStart, kGoal);
    Require(route.success && route.evidence && !route.execution_verified &&
            route.path.poses.front().x == kStart.x && route.path.poses.back().x == kGoal.x &&
            std::abs(n::NormalizeAngle(route.path.poses.back().yaw - kGoal.yaw)) < 1e-9,
            "native route retains exact endpoints and final observation yaw without execution permit");
    CheckEvidence(*route.evidence);
    Require(plugin->SharedRoutePlanner() == graph && !plugin->LastRoute(),
            "read-only queries do not create another graph or overwrite navigation's LastRoute");
    CrossProcess();
    std::cout << "PASS separate_process_shared_graph\n";
    Require(provider.Route(kStart, kGoal, [] { return true; }).canceled,
            "local cancellation does not turn into a successful route");
    e::NativeVistaPathOptions wrong; wrong.planner_id = "missing";
    e::NativeVistaPathProvider unsupported(node, wrong);
    bool rejected = false;
    try { (void)unsupported.Route(kStart, kGoal); } catch (const std::runtime_error&) { rejected = true; }
    Require(rejected, "unsupported plugin is a query fault rather than geometric unreachability");
    source->BlockStart();
    const auto blocked = provider.Route(kStart, kGoal);
    Require(!blocked.success && blocked.evidence && blocked.geometry_revision == 8 && blocked.path.poses.empty(),
            "blocked exact start remains blocked and returns its actual geometry certificate");
}

/** @brief Release an intentionally held real service worker even when assertions fail. */
struct Gate {
    std::atomic<bool> armed{false}, entered{false}, finished{false}, released{false};
    std::promise<void> release_promise;  ///< Independent worker exit permit.
    std::shared_future<void> release = release_promise.get_future().share();
    void Enter() { if (armed.exchange(false)) { entered = true; release.wait(); finished = true; } }
    void Release() { if (!released.exchange(true)) release_promise.set_value(); }
    ~Gate() { Release(); }
};

/** @brief Give malformed replies a real transport while controlling only their wire contents. */
struct Peer {
    std::shared_ptr<autolink::Node> node = autolink::CreateNode("native_vista_protocol_peer");
    std::shared_ptr<Gate> gate = std::make_shared<Gate>();
    std::shared_ptr<std::atomic<int>> fault = std::make_shared<std::atomic<int>>(0);
    std::shared_ptr<std::atomic<unsigned>> calls = std::make_shared<std::atomic<unsigned>>(0);
    std::shared_ptr<autolink::Service<Query::Request, Query::Response>> service;
    /** @brief Install a real service with controlled metadata/correlation faults. */
    Peer() {
        service = node->CreateService<Query::Request, Query::Response>("/get_plan_protocol_test",
            [gate = gate, fault = fault, calls = calls](const auto& request, auto& response) {
                ++*calls; gate->Enter();
                response->set_request_id(request->request_id());
                response->set_planner_id(request->planner_id()); response->set_status(Query::SUCCESS);
                v::VistaMapEvidence evidence{4, 7, 99, {}, {}, 6, "map"};
                evidence.observation_cut.Include(p::SensorKind::kLidar, 17);
                evidence.observation_cut.Include(p::SensorKind::kRgbd, 31);
                v::EncodeEvidence(evidence, response->mutable_graph_evidence());
                if (!request->evidence_only()) {
                    auto* path = response->mutable_plan(); path->mutable_header()->set_frame_id("map");
                    *path->add_poses() = request->start(); *path->add_poses() = request->goal();
                    response->set_length(4);
                }
                switch (fault->load()) {
                    case 1: response->set_request_id("stale-query"); break;
                    case 2: response->clear_graph_evidence(); break;
                    case 3: response->mutable_graph_evidence()->set_frame_id("odom"); break;
                    case 4: response->mutable_graph_evidence()->set_map_epoch(0); break;
                    case 5:
                        if (response->plan().poses_size()) response->mutable_plan()->mutable_poses(1)->
                            mutable_pose()->mutable_position()->set_x(std::numeric_limits<double>::quiet_NaN());
                        break;
                    case 6: response->set_length(5); break;
                    case 7: response->set_status(Query::UNSPECIFIED); break;
                    case 8: response->set_status(Query::NO_ROUTE); break;
                    default: break;
                }
            });
        Require(service != nullptr, "actual protocol service created");
    }
    ~Peer() { gate->Release(); service.reset(); }
};

/** @brief Reject stale tokens, missing evidence and malformed native geometry. */
void ProtocolFaults() {
    Peer peer;
    auto node = autolink::CreateNode("native_vista_protocol_client");
    e::NativeVistaPathOptions options; options.service_name = "/get_plan_protocol_test";
    e::NativeVistaPathProvider provider(node, options);
    Await([&] { return provider.Evidence().has_value(); }, "actual protocol service discovered");
    Require(provider.Route(kStart, kGoal).success, "controlled valid wire reply is accepted");
    for (int fault = 1; fault <= 8; ++fault) {
        *peer.fault = fault;
        bool rejected = false;
        try { (void)provider.Route(kStart, kGoal); } catch (const std::runtime_error&) { rejected = true; }
        Require(rejected, "malformed or uncorrelated reply never becomes route success/no-route evidence");
    }
}

/** @brief Timeout/cancel retain one pending RPC and discard its actual late response. */
void OutstandingRequest(bool cancel) {
    Peer peer;
    auto node = autolink::CreateNode(cancel ? "native_vista_cancel_client" : "native_vista_timeout_client");
    e::NativeVistaPathOptions options; options.service_name = "/get_plan_protocol_test"; options.timeout = 80ms;
    e::NativeVistaPathProvider provider(node, options);
    Await([&] { return provider.Evidence().has_value(); }, "timeout peer discovered");
    peer.gate->armed = true;
    std::atomic<bool> canceled{false};
    auto call = std::async(std::launch::async, [&] {
        try { return provider.Route(kStart, kGoal, [&] { return canceled.load(); }).canceled ? 1 : 0; }
        catch (const std::runtime_error&) { return 2; }
    });
    Await([&] { return peer.gate->entered.load(); }, "actual request entered held service callback");
    if (cancel) canceled = true;
    Require(call.wait_for(2s) == std::future_status::ready && call.get() == (cancel ? 1 : 2),
            "local route wait exits with explicit cancellation or transport timeout");
    const auto calls = peer.calls->load();
    for (unsigned i = 0; i < 20; ++i) Require(!provider.Evidence(), "pending response supplies no new evidence");
    Require(peer.calls->load() == calls, "retries cannot accumulate native pending requests");
    peer.gate->Release();
    Await([&] { return peer.gate->finished.load(); }, "held service worker exited");
    Await([&] { return provider.Evidence().has_value(); }, "late result retired before fresh evidence query");
    Require(provider.Route(kStart, kGoal).success, "fresh token-matched request succeeds after actual retirement");
}

/** @brief Actual retransmission reuses a pending RPC rather than manufacturing new futures. */
void IdempotentRetry() {
    Peer peer;
    auto node = autolink::CreateNode("native_vista_retry_client");
    auto client = node->CreateClient<Query::Request, Query::Response>("/get_plan_protocol_test");
    Require(client && client->Init(), "actual retry client initialized");
    Await([&] { return client->ServiceIsReady(); }, "retry service discovered");
    auto request = std::make_shared<Query::Request>();
    request->set_request_id("same-native-pending-identity");
    request->set_planner_id("vista_planner"); request->set_evidence_only(true);
    peer.gate->armed = true;
    auto pending = client->AsyncSendRequest(request);
    Require(pending.valid(), "original request created a real native future");
    Await([&] { return peer.gate->entered.load(); }, "original request entered the real service worker");
    for (unsigned i = 0; i < 4; ++i)
        Require(client->RetransmitRequest(request), "same outstanding native request retransmitted");
    Require(pending.wait_for(0ms) != std::future_status::ready,
            "retransmission cannot satisfy an unresolved original promise");
    peer.gate->Release();
    Require(pending.wait_for(2s) == std::future_status::ready && pending.get() &&
            pending.get()->request_id() == request->request_id(),
            "original future consumes a reply for the same native request");
    Await([&] { return peer.calls->load() >= 5; }, "queued duplicate requests reached the actual service");
    Require(!client->RetransmitRequest(request), "completed request has no native slot to retransmit");
}

/** @brief Unwind a held callback before joining its concurrent client retirement. */
struct CallbackRetirement {
    std::shared_ptr<Gate> gate;  ///< Real callback barrier, released on every test exit.
    std::thread worker;  ///< Actual Destroy caller; never detached.
    ~CallbackRetirement() { gate->Release(); if (worker.joinable()) worker.join(); }
};

/** @brief Expose the same owned listener copied by actual transport dispatchers. */
class HistoryReceiver : public autolink::transport::HybridReceiver<Query::Response> {
public:
    using HybridReceiver::HybridReceiver;
    using HybridReceiver::SnapshotListener;
};

/** @brief Actual SHM history callbacks drain on retirement; already copied listeners stay closed. */
void ReceiverRetirement() {
    using autolink::proto::RoleAttributes;
    using autolink::transport::MessageInfo;
    RoleAttributes original;
    original.set_host_ip(autolink::common::GlobalData::Instance()->HostIp());
    original.set_channel_name("native_history_retirement_" + std::to_string(getpid()));
    original.set_channel_id(autolink::common::GlobalData::RegisterChannel(original.channel_name()));
    original.mutable_qos_profile()->set_durability(autolink::proto::DURABILITY_TRANSIENT_LOCAL);
    original.mutable_qos_profile()->set_depth(8);
    auto gate = std::make_shared<Gate>(); gate->armed = true;
    auto calls = std::make_shared<std::atomic<unsigned>>(0);
    auto receiver = std::make_shared<HistoryReceiver>(original,
        [gate, calls, name = original.channel_name()](const auto&, const auto&, const auto& attr) {
            Require(attr.channel_name() == name, "history forwards the original receiver identity");
            ++*calls; gate->Enter();
        }, nullptr);
    std::atomic<bool> retiring{false}, retired{false};
    // Install cleanup before the first transport/await can fail. Unwinding
    // must release the callback before destroying its receiver or joining.
    CallbackRetirement retirement{gate, {}};
    auto copied_listener = receiver->SnapshotListener();
    auto writer = receiver->attributes();
    writer.set_id(writer.id() ^ 0x55AA55AAu);
    writer.set_process_id(writer.process_id() + 1);  // Select actual SHM rather than INTRA.
    receiver->Enable(writer);
    auto replay = receiver->attributes();
    replay.set_channel_name(std::to_string(writer.id()) + std::to_string(replay.id()));
    replay.set_channel_id(autolink::common::GlobalData::RegisterChannel(replay.channel_name()));
    autolink::transport::ShmTransmitter<Query::Response> transmitter(replay);
    transmitter.Enable();
    auto message = std::make_shared<Query::Response>(); message->set_request_id("real-history-retirement");
    std::uint64_t sequence = 0;
    Await([&] {
        if (gate->entered.load()) return true;
        Require(transmitter.Transmit(message, MessageInfo(transmitter.id(), ++sequence)),
                "real history message sent through SHM");
        return false;
    }, "real asynchronous history receiver entered its callback");
    retirement.worker = std::thread([receiver = std::move(receiver), &retiring, &retired]() mutable {
        retiring = true;
        receiver.reset(); retired = true;
    });
    Await([&] { return retiring.load(); }, "receiver retirement thread actually started");
    std::this_thread::sleep_for(10ms);
    Require(!retired, "receiver destruction waits for an admitted real history callback");
    gate->Release();
    Await([&] { return retired.load(); }, "real history callback drained before receiver retirement");
    retirement.worker.join();
    const auto completed = calls->load();
    for (unsigned i = 0; i < 8; ++i) {
        copied_listener(message, MessageInfo(transmitter.id(), ++sequence), replay);
        Require(transmitter.Transmit(message, MessageInfo(transmitter.id(), ++sequence)),
                "history transmitter remains live after receiver destruction");
        std::this_thread::sleep_for(5ms);
    }
    Require(calls->load() == completed,
            "copied listeners and late SHM history messages cannot reopen retired callbacks");
}

/** @brief Native client destruction waits for actual callbacks and permits a fresh Init generation. */
void ClientRetirement() {
    Peer peer;
    auto node = autolink::CreateNode("native_vista_retirement_client");
    auto client = node->CreateClient<Query::Request, Query::Response>("/get_plan_protocol_test");
    Require(client && client->Init(), "retirement client initialized");
    Await([&] { return client->ServiceIsReady(); }, "retirement service discovered");
    auto gate = std::make_shared<Gate>(); gate->armed = true;
    std::atomic<bool> retiring{false}, retired{false};
    CallbackRetirement retirement{gate, {}};
    auto request = std::make_shared<Query::Request>();
    request->set_request_id("client-callback-retirement");
    request->set_planner_id("vista_planner"); request->set_evidence_only(true);
    auto pending = client->AsyncSendRequest(request, [gate](auto) { gate->Enter(); });
    Require(pending.valid(), "callback request accepted");
    Await([&] { return gate->entered.load(); }, "native response callback entered barrier");
    retirement.worker = std::thread([client, &retiring, &retired] {
        retiring = true; client->Destroy(); retired = true;
    });
    Await([&] { return retiring.load(); }, "client retirement thread actually started");
    std::this_thread::sleep_for(10ms);
    Require(!retired, "Destroy cannot release native client state while its callback is executing");
    gate->Release();
    Await([&] { return retired.load(); }, "Destroy completed after the actual callback exited");
    retirement.worker.join();
    Require(pending.get() && client->Init(), "retired native client can start a fresh callback generation");
    Await([&] { return client->ServiceIsReady(); }, "reinitialized native client discovered service");
    auto fresh = client->AsyncSendRequest(request);
    Require(fresh.valid() && fresh.wait_for(2s) == std::future_status::ready && fresh.get() &&
            fresh.get()->request_id() == request->request_id(),
            "fresh client generation receives its actual correlated response");
}

/** @brief Server retirement drains real work before releasing transport/queue state. */
void ServiceRetirement() {
    Peer peer;
    auto node = autolink::CreateNode("native_vista_service_retirement_client");
    auto client = node->CreateClient<Query::Request, Query::Response>("/get_plan_protocol_test");
    Require(client && client->Init(), "service retirement client initialized");
    Await([&] { return client->ServiceIsReady(); }, "service retirement endpoint discovered");
    std::atomic<bool> retiring{false}, retired{false};
    CallbackRetirement retirement{peer.gate, {}};
    auto request = std::make_shared<Query::Request>();
    request->set_request_id("service-worker-retirement");
    request->set_planner_id("vista_planner"); request->set_evidence_only(true);
    peer.gate->armed = true;
    auto pending = client->AsyncSendRequest(request);
    Require(pending.valid(), "service retirement request accepted");
    Await([&] { return peer.gate->entered.load(); }, "actual service worker entered callback");
    retirement.worker = std::thread([service = peer.service, &retiring, &retired] {
        retiring = true;
        service->destroy(); retired = true;
    });
    Await([&] { return retiring.load(); }, "service retirement thread actually started");
    std::this_thread::sleep_for(10ms);
    Require(!retired, "service destroy cannot release transport while its worker executes");
    peer.gate->Release();
    Await([&] { return retired.load(); }, "service worker drained before transport destruction");
    retirement.worker.join();
    Require(pending.wait_for(2s) == std::future_status::ready && pending.get() && peer.service->Init(),
            "admitted request completed before the service started a fresh generation");
    Await([&] { return client->ServiceIsReady(); }, "reinitialized service discovered");
    auto fresh = client->AsyncSendRequest(request);
    Require(fresh.valid() && fresh.wait_for(2s) == std::future_status::ready && fresh.get() &&
            fresh.get()->request_id() == request->request_id(),
            "fresh service generation consumes an actual request");
}

/** @brief Run only a native client in the separately spawned process. */
void ClientProcess() {
    auto node = autolink::CreateNode("native_vista_separate_process_client");
    e::NativeVistaPathOptions options; options.timeout = 1000ms;
    e::NativeVistaPathProvider provider(node, options);
    std::optional<v::VistaMapEvidence> evidence;
    Await([&] { evidence = provider.Evidence(); return evidence.has_value(); }, "cross-process service discovered");
    CheckEvidence(*evidence);
    const auto route = provider.Route(kStart, kGoal);
    Require(route.success && route.evidence && !route.execution_verified, "separate process got actual route");
    CheckEvidence(*route.evidence);
}
}  // namespace

/** @brief Initialize actual native middleware and run each finite regression. */
int main(int argc, char** argv) {
    if (argc < 1 || !autolink::Init(argv[0])) return 2;
    executable = argv[0];
    int failed = 0;
    try {
        if (argc == 2 && std::string(argv[1]) == "--route-client") ClientProcess();
        else {
            ReceiverRetirement(); std::cout << "PASS native_history_receiver_retirement\n";
            ServerAndProcess(); std::cout << "PASS actual_server_exact_routes\n";
            ProtocolFaults(); std::cout << "PASS protocol_faults\n";
            OutstandingRequest(false); std::cout << "PASS bounded_timeout_and_late_reply\n";
            OutstandingRequest(true); std::cout << "PASS bounded_cancel_and_late_reply\n";
            IdempotentRetry(); std::cout << "PASS same_identity_native_retransmission\n";
            ClientRetirement(); std::cout << "PASS native_client_callback_retirement\n";
            ServiceRetirement(); std::cout << "PASS native_service_worker_retirement\n";
        }
    } catch (const std::exception& error) { failed = 1; std::cerr << "FAIL native_route_service: " << error.what() << '\n'; }
    autolink::Clear();
    return failed;
}
