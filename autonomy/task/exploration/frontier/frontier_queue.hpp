/* Copyright 2026 The Openbot Authors. */
/** @file frontier_queue.hpp
 * @brief Fair retry scheduling for measured frontier requirements only.
 */
#pragma once

#include <map>
#include <optional>

#include "autonomy/task/exploration/coverage/coverage_state.hpp"

namespace autonomy::task::exploration {

/** @brief Current scheduling disposition of one frontier requirement. */
enum class FrontierDisposition {
    kPending,
    kTemporarilyBlocked,
    kUnreachable,
    kConfirmed,
};

/** @brief Stable retry metadata for one frontier CoverageId. */
struct FrontierRecord {
    CoverageId id;
    FrontierDisposition disposition = FrontierDisposition::kPending;
    double first_pending = 0;
    double next_retry = 0;
    std::uint64_t failure_revision = 0;
    unsigned failed_predictions = 0;
    std::string reason;
};

/** @brief Maintains frontier fairness without duplicating coverage evidence. */
class FrontierQueue
{
public:
    /** @brief Admit or retain one frontier; non-frontier IDs are ignored. */
    void Upsert(const CoverageId& id, double now);
    /** @brief Mirror measured completion for an admitted frontier. */
    void Confirm(const CoverageId& id);
    /** @brief Defer one frontier while preserving its first-pending age. */
    void Block(const CoverageId& id, double now, double retry_after,
               std::uint64_t revision, const std::string& reason);
    /** @brief Record a route-owner failure at one geometry revision. */
    void MarkUnreachable(const CoverageId& id, std::uint64_t revision,
                         const std::string& proof);
    /** @brief Reopen failures only when newer geometry may change reachability.
     */
    void ReopenForMapChange(std::uint64_t revision);
    /** @brief Mirror frontier requirements and measured dispositions. */
    void Synchronize(const CoverageState& coverage, double now);
    /** @brief Return due frontier IDs in deterministic identity order. */
    std::vector<CoverageId> Due(double now) const;
    /** @brief Return the oldest due frontier, if any. */
    std::optional<CoverageId> OldestDue(double now) const;
    /** @brief Count frontier records not confirmed by measured evidence. */
    std::size_t PendingCount() const;
    /** @brief Expose immutable scheduling diagnostics. */
    const std::map<CoverageId, FrontierRecord>& records() const {
        return records_;
    }
    /** @brief Clear scheduling metadata on session or epoch reset. */
    void Reset() {
        records_.clear();
    }

private:
    std::map<CoverageId, FrontierRecord> records_;
};

}  // namespace autonomy::task::exploration
