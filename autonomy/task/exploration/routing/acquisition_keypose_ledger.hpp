/* Copyright 2026 The Openbot Authors. */
/**
 * @file acquisition_keypose_ledger.hpp
 * @brief Bounded edge-free ledger of exact acquisition-localized base poses.
 */
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <optional>
#include <string>

#include "autonomy/task/exploration/routing/acquisition_keypose.hpp"

namespace autonomy::task::exploration {

/**
 * @brief Retains provenance-complete acquisition poses without route edges.
 * @details The owner serializes calls. Rejected or spacing-filtered evidence
 * never becomes a keypose. Source cursors still advance after a spacing filter
 * so stale packets cannot be admitted later. Vista remains the sole route-graph
 * owner.
 */
class AcquisitionKeyposeLedger
{
public:
    /** @brief Construct a ledger with finite positive spacing and bounded
     * capacity. */
    explicit AcquisitionKeyposeLedger(
        AcquisitionKeyposeLedgerOptions options = {});

    /** @brief Clear retained evidence and begin one explicit map epoch. */
    void Reset(std::uint64_t epoch);

    /** @brief Validate and conditionally retain one exact acquisition pose. */
    AcquisitionKeyposeRecordResult Record(AcquisitionKeyposeEvidence evidence);

    /** @return Explicit epoch accepted by this ledger, or zero while disabled.
     */
    std::uint64_t epoch() const {
        return epoch_;
    }

    /** @return Mutation generation for immutable snapshot publication. */
    std::uint64_t generation() const {
        return generation_;
    }

    /** @return Oldest-to-newest retained keyposes. */
    const std::deque<AcquisitionKeypose>& entries() const {
        return entries_;
    }

private:
    struct SourceCursor {
        bool initialized = false;
        std::uint64_t sequence = 0;
        std::uint64_t calibration_revision = 0;
        double stamp = 0;
    };

    static std::optional<std::size_t> SourceIndex(
        perception::SensorKind sensor);

    AcquisitionKeyposeLedgerOptions options_;
    std::uint64_t epoch_ = 0;
    std::uint64_t generation_ = 0;
    std::uint64_t next_id_ = 0;
    std::uint64_t geometry_revision_ = 0;
    std::string frame_id_;
    std::array<SourceCursor, 2> sources_{};
    std::deque<AcquisitionKeypose> entries_;
};

}  // namespace autonomy::task::exploration
