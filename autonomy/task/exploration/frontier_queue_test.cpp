/* Copyright 2026 The Openbot Authors. */
#include <iostream>
#include <stdexcept>

#include "autonomy/task/exploration/frontier/frontier_queue.hpp"

namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;
namespace {

void Require(bool condition, const char* message) {
    if (!condition)
        throw std::runtime_error(message);
}

e::CoverageUnit Unit(e::CoverageKind kind, int x) {
    e::CoverageUnit unit;
    unit.id = {x, 0, 0, 0, kind};
    unit.center = {static_cast<double>(x), 0, 0};
    unit.required_quality = 0.5;
    return unit;
}

void OnlyFrontiersAreScheduled() {
    e::CoverageState coverage;
    coverage.Reset(1);
    const auto frontier = Unit(e::CoverageKind::kFrontier, 1);
    coverage.Upsert(frontier);
    coverage.Upsert(Unit(e::CoverageKind::kSurface, 2));
    coverage.Upsert(Unit(e::CoverageKind::kDirection, 3));
    e::FrontierQueue queue;
    queue.Synchronize(coverage, 4);
    Require(queue.records().size() == 1 && queue.records().count(frontier.id),
            "surface and direction requirements never enter FrontierQueue");
    Require(queue.OldestDue(4) == frontier.id,
            "only a frontier can become the mandatory oldest item");
    coverage.Confirm(frontier.id, p::SensorKind::kRgbd, 1, 5);
    queue.Synchronize(coverage, 5);
    Require(queue.PendingCount() == 0 && !queue.OldestDue(5),
            "measured CoverageState completion confirms the frontier");
}

void FailureAgeAndGeometryReopenAreStable() {
    e::FrontierQueue queue;
    const auto frontier = Unit(e::CoverageKind::kFrontier, 1).id;
    queue.Upsert(frontier, 2);
    queue.Block(frontier, 3, 10, 7, "route unavailable");
    queue.Block(frontier, 4, 10, 7, "route still unavailable");
    Require(queue.records().at(frontier).first_pending == 2,
            "repeated blocks preserve first-pending fairness age");
    queue.ReopenForMapChange(7);
    Require(queue.records().at(frontier).disposition ==
                e::FrontierDisposition::kTemporarilyBlocked,
            "same geometry revision cannot reopen a failure");
    queue.ReopenForMapChange(8);
    Require(queue.records().at(frontier).disposition ==
                    e::FrontierDisposition::kPending &&
                queue.OldestDue(4) == frontier,
            "newer geometry reopens the existing frontier record");
    const auto surface = Unit(e::CoverageKind::kSurface, 2).id;
    queue.Upsert(surface, 1);
    queue.Block(surface, 1, 1, 1, "not a frontier");
    Require(!queue.records().count(surface),
            "direct non-frontier scheduling calls are ignored");
}

}  // namespace

int main() {
    try {
        OnlyFrontiersAreScheduled();
        FailureAgeAndGeometryReopenAreStable();
        std::cout << "PASS frontier_queue\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "FAIL frontier_queue: " << error.what() << '\n';
        return 1;
    }
}
