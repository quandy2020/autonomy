/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/frontier/frontier_queue.hpp"

#include <algorithm>
#include <stdexcept>

namespace autonomy::task::exploration {

void FrontierQueue::Upsert(const CoverageId& id, double now) {
    if (id.kind != CoverageKind::kFrontier)
        return;
    if (!std::isfinite(now))
        throw std::invalid_argument("invalid frontier time");
    const auto found = records_.find(id);
    if (found == records_.end() ||
        found->second.disposition == FrontierDisposition::kConfirmed) {
        records_[id] = {id, FrontierDisposition::kPending, now, now, 0, 0, {}};
    }
}

void FrontierQueue::Confirm(const CoverageId& id) {
    const auto found = records_.find(id);
    if (found != records_.end())
        found->second.disposition = FrontierDisposition::kConfirmed;
}

void FrontierQueue::Block(const CoverageId& id, double now, double delay,
                          std::uint64_t revision, const std::string& reason) {
    if (id.kind != CoverageKind::kFrontier)
        return;
    if (!std::isfinite(delay) || delay < 0 || reason.empty())
        throw std::invalid_argument("blocked frontier needs retry and reason");
    Upsert(id, now);
    auto& record = records_.at(id);
    record.disposition = FrontierDisposition::kTemporarilyBlocked;
    record.next_retry = now + delay;
    record.failure_revision = revision;
    record.reason = reason;
    ++record.failed_predictions;
}

void FrontierQueue::MarkUnreachable(const CoverageId& id,
                                    std::uint64_t revision,
                                    const std::string& proof) {
    if (id.kind != CoverageKind::kFrontier)
        return;
    if (proof.empty() || !records_.count(id))
        throw std::invalid_argument("unreachable needs recorded search proof");
    auto& record = records_.at(id);
    record.disposition = FrontierDisposition::kUnreachable;
    record.failure_revision = revision;
    record.reason = proof;
}

void FrontierQueue::ReopenForMapChange(std::uint64_t revision) {
    for (auto& entry : records_) {
        auto& record = entry.second;
        if (record.failure_revision && revision > record.failure_revision &&
            record.disposition != FrontierDisposition::kConfirmed) {
            record.disposition = FrontierDisposition::kPending;
            record.next_retry = 0;
            record.reason.clear();
        }
    }
}

void FrontierQueue::Synchronize(const CoverageState& coverage, double now) {
    for (auto iterator = records_.begin(); iterator != records_.end();) {
        if (iterator->first.kind != CoverageKind::kFrontier ||
            !coverage.units().count(iterator->first))
            iterator = records_.erase(iterator);
        else
            ++iterator;
    }
    for (const auto& entry : coverage.units()) {
        if (entry.first.kind != CoverageKind::kFrontier)
            continue;
        Upsert(entry.first, now);
        if (!entry.second.pending)
            Confirm(entry.first);
    }
}

std::vector<CoverageId> FrontierQueue::Due(double now) const {
    std::vector<CoverageId> result;
    for (const auto& entry : records_) {
        const auto& record = entry.second;
        if ((record.disposition == FrontierDisposition::kPending ||
             record.disposition == FrontierDisposition::kTemporarilyBlocked) &&
            record.next_retry <= now)
            result.push_back(entry.first);
    }
    return result;
}

std::optional<CoverageId> FrontierQueue::OldestDue(double now) const {
    std::optional<CoverageId> result;
    double oldest = std::numeric_limits<double>::infinity();
    for (const auto& id : Due(now)) {
        if (records_.at(id).first_pending < oldest) {
            oldest = records_.at(id).first_pending;
            result = id;
        }
    }
    return result;
}

std::size_t FrontierQueue::PendingCount() const {
    return std::count_if(
        records_.begin(), records_.end(), [](const auto& entry) {
            return entry.second.disposition != FrontierDisposition::kConfirmed;
        });
}

}  // namespace autonomy::task::exploration
