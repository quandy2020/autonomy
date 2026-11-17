/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <map>
#include <optional>
#include "autonomy/task/exploration/coverage/coverage_state.hpp"
namespace autonomy::task::exploration {
enum class FrontierDisposition { kPending, kTemporarilyBlocked, kUnreachable, kConfirmed };
struct FrontierRecord {
    CoverageId id;
    FrontierDisposition disposition=FrontierDisposition::kPending;
    double first_pending=0,next_retry=0;
    std::uint64_t failure_revision=0;
    unsigned failed_predictions=0;
    std::string reason;
};
class FrontierQueue {
public:
    void Upsert(const CoverageId& id,double now);
    void Confirm(const CoverageId& id);
    void Block(const CoverageId& id,double now,double retry_after,std::uint64_t revision,const std::string& reason);
    void MarkUnreachable(const CoverageId& id,std::uint64_t revision,const std::string& proof);
    void ReopenForMapChange(std::uint64_t revision);
    void Synchronize(const CoverageState& coverage, double now);
    std::vector<CoverageId> Due(double now) const;
    std::optional<CoverageId> OldestDue(double now) const;
    std::size_t PendingCount() const;
    const std::map<CoverageId,FrontierRecord>& records() const{return records_;}
    void Reset(){records_.clear();}
private:
    std::map<CoverageId,FrontierRecord> records_;
};
}  // namespace autonomy::task::exploration
