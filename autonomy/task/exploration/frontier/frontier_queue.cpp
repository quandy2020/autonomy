/* Copyright 2026 The Openbot Authors. */
#include "autonomy/task/exploration/frontier/frontier_queue.hpp"
#include <algorithm>
#include <stdexcept>
namespace autonomy::task::exploration {
void FrontierQueue::Upsert(const CoverageId& id,double now) {
    if(!std::isfinite(now))throw std::invalid_argument("invalid frontier time");
    if(!records_.count(id) || records_.at(id).disposition==FrontierDisposition::kConfirmed)
        records_[id]={id,FrontierDisposition::kPending,now,now,0,0,{}};
}
void FrontierQueue::Confirm(const CoverageId& id) {auto it=records_.find(id);if(it!=records_.end())it->second.disposition=FrontierDisposition::kConfirmed;}
void FrontierQueue::Block(const CoverageId& id,double now,double delay,std::uint64_t revision,const std::string& reason) {
    if(!std::isfinite(delay) || delay<0 || reason.empty())throw std::invalid_argument("blocked frontier needs retry and reason");
    Upsert(id,now);auto& r=records_.at(id);r.disposition=FrontierDisposition::kTemporarilyBlocked;
    r.next_retry=now+delay;r.failure_revision=revision;r.reason=reason;++r.failed_predictions;
}
void FrontierQueue::MarkUnreachable(const CoverageId& id,std::uint64_t revision,const std::string& proof) {
    if(proof.empty() || !records_.count(id))throw std::invalid_argument("unreachable needs recorded search proof");
    auto& r=records_.at(id);r.disposition=FrontierDisposition::kUnreachable;r.failure_revision=revision;r.reason=proof;
}
void FrontierQueue::ReopenForMapChange(std::uint64_t revision) {
    for(auto& p:records_) {
        auto& r=p.second;
        if(r.failure_revision && revision>r.failure_revision && r.disposition!=FrontierDisposition::kConfirmed) {
            r.disposition=FrontierDisposition::kPending;r.next_retry=0;r.reason.clear();
        }
    }
}
void FrontierQueue::Synchronize(const CoverageState& coverage, double now) {
    for (auto it = records_.begin(); it != records_.end();) {
        if (!coverage.units().count(it->first)) it = records_.erase(it);
        else ++it;
    }
    for (const auto& entry : coverage.units()) {
        Upsert(entry.first, now);
        if (!entry.second.pending) Confirm(entry.first);
    }
}
std::vector<CoverageId> FrontierQueue::Due(double now) const {
    std::vector<CoverageId> result;
    for(const auto& p:records_)if((p.second.disposition==FrontierDisposition::kPending ||
        p.second.disposition==FrontierDisposition::kTemporarilyBlocked) && p.second.next_retry<=now)result.push_back(p.first);
    return result;
}
std::optional<CoverageId> FrontierQueue::OldestDue(double now) const {
    std::optional<CoverageId> result;double oldest=std::numeric_limits<double>::infinity();
    for(const auto& id:Due(now))if(records_.at(id).first_pending<oldest) {oldest=records_.at(id).first_pending;result=id;}
    return result;
}
std::size_t FrontierQueue::PendingCount() const {
    return std::count_if(records_.begin(),records_.end(),[](const auto& p){return p.second.disposition!=FrontierDisposition::kConfirmed;});
}
}  // namespace autonomy::task::exploration
