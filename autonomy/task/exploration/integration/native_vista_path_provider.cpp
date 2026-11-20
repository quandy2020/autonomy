/* Copyright 2026 The Openbot Authors. */
/** @file native_vista_path_provider.cpp
 * @brief Token-matched exact routes with bounded native RPC lifetime and provenance checks.
 */
#include "autonomy/task/exploration/integration/native_vista_path_provider.hpp"

#include <algorithm>
#include <future>
#include <limits>
#include <mutex>
#include <random>
#include <sstream>
#include <stdexcept>
#include <thread>
#include "autolink/node/node.hpp"
#include "autolink/service/client.hpp"
#include "autonomy/planning/planner/vista/native_route_protocol.hpp"

namespace autonomy::task::exploration {
namespace {
namespace v = planning::planner::vista;
using Query = automsgs::srvs::GetPlan;
using Client = autolink::Client<Query::Request, Query::Response>;
using Clock = std::chrono::steady_clock;

/** @brief Generate a process-independent client token namespace, unrelated to map epochs. */
std::string TokenPrefix() {
    std::random_device source;
    std::ostringstream token;
    token << "vista-query-" << std::hex;
    for (unsigned i = 0; i < 4; ++i) { token.width(8); token.fill('0'); token << source(); }
    return token.str();
}

/** @brief Decode a finite native pose without treating an invalid quaternion as yaw zero. */
std::optional<Pose2d> PlanarPose(const automsgs::msgs::geometry_msgs::PoseStamped& pose,
                               const std::string& frame) {
    const auto& p = pose.pose().position(); const auto& q = pose.pose().orientation();
    const double norm = q.x()*q.x()+q.y()*q.y()+q.z()*q.z()+q.w()*q.w();
    if (pose.header().frame_id() != frame || !std::isfinite(p.x()) ||
        !std::isfinite(p.y()) || !std::isfinite(p.z()) || !std::isfinite(norm) || norm < 1e-12) return {};
    return Pose2d{p.x(), p.y(), std::atan2(2*(q.w()*q.z()+q.x()*q.y()),
                                        norm-2*(q.y()*q.y()+q.z()*q.z()))};
}

/** @brief A route certificate must preserve both endpoint positions and observation yaw. */
bool SamePose(const Pose2d& a, const Pose2d& b) {
    return autonomy::navigation::Distance(a, b) <= 1e-6 &&
        std::abs(autonomy::navigation::NormalizeAngle(a.yaw - b.yaw)) <= 1e-6;
}
}  // namespace

class NativeVistaPathProvider::Impl {
public:
    /** @brief Keep each wait outcome separate from mutable transport diagnostics. */
    struct Reply {
        std::shared_ptr<Query::Response> response;  ///< Actual token-matched native reply.
        bool canceled = false;  ///< Local cancellation observed before consuming the reply.
        std::string error;  ///< Transport/protocol failure, never geometric unreachability.
    };

    /** @brief Validate settings before constructing the actual native client. */
    Impl(std::shared_ptr<autolink::Node> node, NativeVistaPathOptions options)
        : node_(std::move(node)), options_(std::move(options)), prefix_(TokenPrefix()) {
        if (!node_ || options_.service_name.empty() || options_.planner_id.empty() ||
            options_.planner_id.size() > 256 || options_.frame_id.empty() ||
            options_.timeout.count() <= 0 || options_.timeout > std::chrono::seconds(5) ||
            options_.retry_interval.count() < 0 || options_.retry_interval > std::chrono::seconds(5) ||
            options_.maximum_poses < 2 || options_.maximum_poses > 2000000)
            throw std::invalid_argument("invalid native Vista route client settings");
        client_ = node_->CreateClient<Query::Request, Query::Response>(options_.service_name);
        if (!client_ || !client_->Init()) throw std::runtime_error("native Vista client initialization failed");
    }

    /** @brief Serialize requests under a finite wait bound and retain unresolved RPC ownership. */
    Reply Send(bool evidence_only, const Pose2d& start, const Pose2d& goal,
               const std::function<bool()>& canceled) const {
        const auto deadline = Clock::now() + options_.timeout;
        auto stop = [&] { return canceled && canceled(); };
        std::unique_lock<std::mutex> lock(mutex_, std::defer_lock);
        while (!lock.try_lock()) {
            if (stop()) return {{}, true, {}};
            if (Clock::now() >= deadline) return {{}, false, "native Vista client serialization timed out"};
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
        if (stop()) return {{}, true, {}};
        try {
            if (pending_.valid()) {
                // Read-only route queries share one native client. A prior
                // query can finish just after a new request arrives; wait for
                // that bounded operation instead of turning ordinary serialized
                // client ownership into an exploration failure.
                while (pending_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                    if (stop()) return {{}, true, {}};
                    if (Clock::now() >= deadline)
                        return {{}, false, "previous native Vista request exceeded this query budget"};
                    std::this_thread::sleep_for(std::chrono::milliseconds(1));
                }
                try { (void)pending_.get(); } catch (...) { /* Retire a completed broken promise. */ }
                pending_ = {};
            }
            if (!client_->ServiceIsReady()) return {{}, false, "native Vista route service is unavailable"};
            if (Clock::now() >= deadline) return {{}, false, "native Vista query budget expired before send"};
            if (sequence_ == std::numeric_limits<std::uint64_t>::max())
                return {{}, false, "native Vista request identity exhausted"};
            auto request = std::make_shared<Query::Request>();
            const auto token = prefix_ + "-" + std::to_string(++sequence_);
            request->set_request_id(token); request->set_planner_id(options_.planner_id);
            request->set_evidence_only(evidence_only); request->set_require_graph_evidence(true);
            request->set_timeout_ms(static_cast<std::uint32_t>(options_.timeout.count()));
            if (!evidence_only) {
                *request->mutable_start() = v::RoutePose(start, options_.frame_id);
                *request->mutable_goal() = v::RoutePose(goal, options_.frame_id);
            }
            // Use the mutable shared-request overload supported by the native transport.
            pending_ = client_->AsyncSendRequest(request);
            if (!pending_.valid()) return {{}, false, "native Vista transport refused the request"};
            auto retry_at = Clock::now() + options_.retry_interval;
            while (pending_.wait_for(std::chrono::milliseconds(0)) != std::future_status::ready) {
                if (stop()) return {{}, true, {}};
                if (Clock::now() >= deadline) return {{}, false, "native Vista reply timed out"};
                if (options_.retry_interval.count() && Clock::now() >= retry_at) {
                    // QueryRoute is read-only. Reuse the native sequence/promise;
                    // no retry can add another pending request or extend the deadline.
                    (void)client_->RetransmitRequest(request);
                    retry_at = Clock::now() + options_.retry_interval;
                }
                pending_.wait_for(std::chrono::milliseconds(1));
            }
            if (stop()) return {{}, true, {}};
            if (Clock::now() >= deadline) return {{}, false, "native Vista reply arrived after the query deadline"};
            auto response = pending_.get(); pending_ = {};
            if (!response || response->request_id() != token ||
                response->planner_id() != options_.planner_id)
                return {{}, false, "native Vista reply correlation mismatch"};
            return {std::move(response), false, {}};
        } catch (const std::exception& error) {
            // Keep any still-unresolved future; never create a second pending slot.
            return {{}, false, error.what()};
        }
    }

    /** @brief Accept only explicitly certified evidence in the configured coordinate frame. */
    std::optional<RouteMapEvidence> Evidence() const {
        auto reply = Send(true, {}, {}, {});
        if (!reply.response || reply.response->status() != Query::SUCCESS ||
            !reply.response->has_graph_evidence() || reply.response->plan().poses_size()) return {};
        auto evidence = v::DecodeEvidence(reply.response->graph_evidence());
        if (evidence && evidence->frame_id == options_.frame_id) return evidence;
        return {};
    }

    /** @brief Convert a checked native response; faults remain distinct from a checked no-route. */
    v::VistaRoute Route(const Pose2d& start, const Pose2d& goal,
                        const std::function<bool()>& canceled) const {
        v::VistaRoute route;
        if (!autonomy::navigation::IsFinite(start) || !autonomy::navigation::IsFinite(goal))
            throw std::invalid_argument("nonfinite native Vista route endpoints");
        auto reply = Send(false, start, goal, canceled);
        if (reply.canceled) { route.canceled = true; return route; }
        if (!reply.response) throw std::runtime_error(reply.error);
        const auto& response = *reply.response;
        if (response.status() != Query::SUCCESS && response.status() != Query::NO_ROUTE)
            throw std::runtime_error("native Vista query failed with status " + std::to_string(response.status()));
        if (!response.has_graph_evidence()) throw std::runtime_error("native Vista reply lacks graph evidence");
        route.evidence = v::DecodeEvidence(response.graph_evidence());
        if (!route.evidence || route.evidence->frame_id != options_.frame_id)
            throw std::runtime_error("native Vista reply has invalid source provenance");
        route.map_epoch = route.evidence->map_epoch;
        route.geometry_revision = route.evidence->geometry_revision;
        route.path.frame_id = route.evidence->frame_id;
        if (response.status() == Query::NO_ROUTE) {
            if (response.plan().poses_size()) throw std::runtime_error("no-route reply contains an executable path");
            return route;
        }
        const auto& path = response.plan();
        if (path.header().frame_id() != options_.frame_id || path.poses_size() < 2 ||
            static_cast<std::size_t>(path.poses_size()) > options_.maximum_poses ||
            !std::isfinite(response.length()) || response.length() < 0)
            throw std::runtime_error("native Vista reply has invalid path geometry or length");
        for (const auto& native : path.poses()) {
            const auto pose = PlanarPose(native, options_.frame_id);
            if (!pose) throw std::runtime_error("native Vista reply contains a malformed pose");
            if (!route.path.poses.empty()) route.length += autonomy::navigation::Distance(route.path.poses.back(), *pose);
            route.path.poses.push_back(*pose);
        }
        if (!std::isfinite(route.length) || !SamePose(route.path.poses.front(), start) ||
            !SamePose(route.path.poses.back(), goal) ||
            std::abs(route.length - response.length()) > 1e-6 * std::max(1.0, route.length))
            throw std::runtime_error("native Vista route endpoints or metric length disagree with the request");
        if (canceled && canceled()) { route = {}; route.canceled = true; return route; }
        route.used_grid_fallback = response.used_grid_fallback();
        route.success = true; route.execution_verified = false;
        return route;
    }

private:
    std::shared_ptr<autolink::Node> node_;  ///< Outlives the native client.
    NativeVistaPathOptions options_;  ///< Immutable transport/frame/capacity policy.
    std::string prefix_;  ///< Independent opaque caller token namespace.
    std::shared_ptr<Client> client_;  ///< One actual native request/response endpoint.
    mutable std::mutex mutex_;  ///< Serializes native pending-request ownership.
    mutable std::uint64_t sequence_ = 0;  ///< Monotone caller correlation only.
    mutable Client::SharedFuture pending_;  ///< At most one unresolved native request.
};

NativeVistaPathProvider::NativeVistaPathProvider(std::shared_ptr<autolink::Node> node,
                                               NativeVistaPathOptions options)
    : impl_(std::make_unique<Impl>(std::move(node), std::move(options))) {}
NativeVistaPathProvider::~NativeVistaPathProvider() = default;
std::optional<RouteMapEvidence> NativeVistaPathProvider::Evidence() const { return impl_->Evidence(); }
v::VistaRoute NativeVistaPathProvider::Route(const Pose2d& start, const Pose2d& goal) const {
    return impl_->Route(start, goal, {});
}
v::VistaRoute NativeVistaPathProvider::Route(const Pose2d& start, const Pose2d& goal,
                                           const std::function<bool()>& canceled) const {
    return impl_->Route(start, goal, canceled);
}

}  // namespace autonomy::task::exploration
