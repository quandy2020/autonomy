/* Copyright 2026 The Openbot Authors. */

#include "autonomy/task/exploration/routing/acquisition_keypose_ledger.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace autonomy::task::exploration {

AcquisitionKeyposeLedger::AcquisitionKeyposeLedger(
    AcquisitionKeyposeLedgerOptions options)
    : options_(options) {
    if (!std::isfinite(options_.minimum_spacing) ||
        options_.minimum_spacing <= 0.0) {
        throw std::invalid_argument(
            "keypose spacing must be finite and positive");
    }
    if (options_.maximum_entries == 0 || options_.maximum_entries > 1000000) {
        throw std::invalid_argument("keypose retention bound is invalid");
    }
}

void AcquisitionKeyposeLedger::Reset(std::uint64_t epoch) {
    epoch_ = epoch;
    ++generation_;
    next_id_ = 0;
    geometry_revision_ = 0;
    frame_id_.clear();
    sources_ = {};
    entries_.clear();
}

std::optional<std::size_t> AcquisitionKeyposeLedger::SourceIndex(
    perception::SensorKind sensor) {
    switch (sensor) {
        case perception::SensorKind::kLidar:
            return 0;
        case perception::SensorKind::kRgbd:
            return 1;
    }
    return std::nullopt;
}

AcquisitionKeyposeRecordResult AcquisitionKeyposeLedger::Record(
    AcquisitionKeyposeEvidence evidence) {
    const auto source = SourceIndex(evidence.source_sensor);
    if (!source || !autonomy::navigation::IsFinite(evidence.base_pose) ||
        !std::isfinite(evidence.acquisition_stamp) ||
        evidence.acquisition_stamp <= 0.0 || evidence.source_sequence == 0 ||
        evidence.map_epoch == 0 || evidence.geometry_revision == 0 ||
        evidence.calibration_revision == 0 || evidence.frame_id.empty()) {
        return AcquisitionKeyposeRecordResult::kInvalidEvidence;
    }
    if (epoch_ == 0 || evidence.map_epoch != epoch_) {
        return AcquisitionKeyposeRecordResult::kWrongEpoch;
    }
    if ((!frame_id_.empty() && evidence.frame_id != frame_id_) ||
        evidence.geometry_revision < geometry_revision_) {
        return AcquisitionKeyposeRecordResult::kStaleProvenance;
    }

    auto& cursor = sources_[*source];
    if (cursor.initialized &&
        (evidence.source_sequence <= cursor.sequence ||
         evidence.acquisition_stamp < cursor.stamp ||
         evidence.calibration_revision < cursor.calibration_revision)) {
        return AcquisitionKeyposeRecordResult::kStaleProvenance;
    }

    cursor.initialized = true;
    cursor.sequence = evidence.source_sequence;
    cursor.stamp = evidence.acquisition_stamp;
    cursor.calibration_revision = evidence.calibration_revision;
    geometry_revision_ = evidence.geometry_revision;
    if (frame_id_.empty()) {
        frame_id_ = evidence.frame_id;
    }

    for (const auto& retained : entries_) {
        const double dx = retained.base_pose.x - evidence.base_pose.x;
        const double dy = retained.base_pose.y - evidence.base_pose.y;
        if (std::hypot(dx, dy) <= options_.minimum_spacing) {
            return AcquisitionKeyposeRecordResult::kSpacingFiltered;
        }
    }

    AcquisitionKeypose keypose;
    static_cast<AcquisitionKeyposeEvidence&>(keypose) = std::move(evidence);
    keypose.id = ++next_id_;
    entries_.push_back(std::move(keypose));
    while (entries_.size() > options_.maximum_entries) {
        entries_.pop_front();
    }
    ++generation_;
    return AcquisitionKeyposeRecordResult::kRecorded;
}

}  // namespace autonomy::task::exploration
