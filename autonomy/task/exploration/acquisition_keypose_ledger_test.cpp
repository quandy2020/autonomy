/* Copyright 2026 The Openbot Authors. */
/** @file acquisition_keypose_ledger_test.cpp
 * @brief Regressions for edge-free, provenance-complete acquisition keyposes.
 */
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>

#include "autonomy/task/exploration/routing/acquisition_keypose_ledger.hpp"

namespace {
namespace e = autonomy::task::exploration;
namespace p = autonomy::perception;

void Require(bool condition, const char* message) {
    if (!condition) {
        throw std::runtime_error(message);
    }
}

e::AcquisitionKeyposeEvidence Evidence(
    double x, std::uint64_t sequence,
    p::SensorKind sensor = p::SensorKind::kLidar, double stamp = 1.0,
    std::uint64_t epoch = 7, std::uint64_t revision = 3,
    std::uint64_t calibration = 5, std::string frame = "map") {
    e::AcquisitionKeyposeEvidence evidence;
    evidence.base_pose = {x, x + 0.25, 0.1 * x};
    evidence.source_sensor = sensor;
    evidence.source_sequence = sequence;
    evidence.map_epoch = epoch;
    evidence.geometry_revision = revision;
    evidence.calibration_revision = calibration;
    evidence.acquisition_stamp = stamp;
    evidence.frame_id = std::move(frame);
    return evidence;
}

void ExactEvidenceAndNearestSpacing() {
    e::AcquisitionKeyposeLedger ledger({1.0, 8});
    ledger.Reset(7);
    Require(ledger.Record(Evidence(0, 1)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "first exact acquisition was rejected");
    Require(ledger.Record(Evidence(3, 2, p::SensorKind::kLidar, 2)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "distant acquisition was rejected");
    Require(ledger.Record(Evidence(0.5, 3, p::SensorKind::kLidar, 3)) ==
                e::AcquisitionKeyposeRecordResult::kSpacingFiltered,
            "spacing was not checked against every retained keypose");
    Require(ledger.entries().size() == 2, "spacing filter retained a keypose");
    const auto& first = ledger.entries().front();
    Require(first.id == 1 && first.base_pose.x == 0 &&
                first.base_pose.y == 0.25 && first.base_pose.yaw == 0 &&
                first.source_sequence == 1 && first.map_epoch == 7 &&
                first.geometry_revision == 3 &&
                first.calibration_revision == 5 &&
                first.acquisition_stamp == 1 && first.frame_id == "map",
            "retained keypose did not preserve exact evidence");
}

void FilteredEvidenceAdvancesCursor() {
    e::AcquisitionKeyposeLedger ledger({1.0, 8});
    ledger.Reset(7);
    Require(ledger.Record(Evidence(0, 1)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "cursor fixture was rejected");
    Require(ledger.Record(Evidence(0.25, 2, p::SensorKind::kLidar, 2)) ==
                e::AcquisitionKeyposeRecordResult::kSpacingFiltered,
            "near packet was not spacing-filtered");
    Require(ledger.Record(Evidence(3, 2, p::SensorKind::kLidar, 2)) ==
                e::AcquisitionKeyposeRecordResult::kStaleProvenance,
            "filtered packet did not advance source cursor");
    Require(ledger.Record(Evidence(3, 3, p::SensorKind::kLidar, 3)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "newer source evidence did not recover after filtering");
}

void BoundedRetentionAndEpochReset() {
    e::AcquisitionKeyposeLedger ledger({1.0, 2});
    ledger.Reset(7);
    Require(ledger.Record(Evidence(0, 1)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "retention fixture one failed");
    Require(ledger.Record(Evidence(2, 2, p::SensorKind::kLidar, 2)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "retention fixture two failed");
    Require(ledger.Record(Evidence(4, 3, p::SensorKind::kLidar, 3)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "retention fixture three failed");
    Require(ledger.entries().size() == 2 && ledger.entries().front().id == 2 &&
                ledger.entries().back().id == 3,
            "bounded FIFO retention is incorrect");
    const auto generation = ledger.generation();
    ledger.Reset(8);
    Require(ledger.generation() == generation + 1 && ledger.entries().empty(),
            "explicit epoch reset did not invalidate retained evidence");
    Require(ledger.Record(Evidence(8, 1, p::SensorKind::kLidar, 4, 7)) ==
                e::AcquisitionKeyposeRecordResult::kWrongEpoch,
            "old-epoch evidence survived reset");
    Require(ledger.Record(Evidence(8, 1, p::SensorKind::kLidar, 4, 8)) ==
                    e::AcquisitionKeyposeRecordResult::kRecorded &&
                ledger.entries().front().id == 1,
            "new epoch did not permit restarted source and ledger ordinals");
}

void IndependentSourcesAndMonotonicProvenance() {
    e::AcquisitionKeyposeLedger ledger({1.0, 8});
    ledger.Reset(7);
    Require(ledger.Record(Evidence(0, 10, p::SensorKind::kLidar, 10)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "lidar source fixture failed");
    Require(ledger.Record(Evidence(2, 1, p::SensorKind::kRgbd, 2)) ==
                e::AcquisitionKeyposeRecordResult::kRecorded,
            "independent RGB-D clock was treated as globally stale");
    Require(ledger.Record(Evidence(4, 11, p::SensorKind::kLidar, 9)) ==
                e::AcquisitionKeyposeRecordResult::kStaleProvenance,
            "source timestamp rollback was admitted");
    Require(
        ledger.Record(Evidence(4, 11, p::SensorKind::kLidar, 11, 7, 3, 4)) ==
            e::AcquisitionKeyposeRecordResult::kStaleProvenance,
        "calibration rollback was admitted");
    Require(ledger.Record(Evidence(4, 11, p::SensorKind::kLidar, 11, 7, 2)) ==
                e::AcquisitionKeyposeRecordResult::kStaleProvenance,
            "geometry revision rollback was admitted");
    Require(ledger.Record(
                Evidence(4, 11, p::SensorKind::kLidar, 11, 7, 4, 6, "other")) ==
                e::AcquisitionKeyposeRecordResult::kStaleProvenance,
            "frame mismatch was admitted");
}

void InvalidConfigurationAndEvidence() {
    bool threw = false;
    try {
        e::AcquisitionKeyposeLedger invalid({0, 8});
    } catch (const std::invalid_argument&) {
        threw = true;
    }
    Require(threw, "zero spacing configuration was accepted");
    e::AcquisitionKeyposeLedger ledger;
    ledger.Reset(7);
    auto invalid = Evidence(0, 1);
    invalid.source_sensor = static_cast<p::SensorKind>(255);
    Require(ledger.Record(std::move(invalid)) ==
                e::AcquisitionKeyposeRecordResult::kInvalidEvidence,
            "unknown source kind was accepted");
}

}  // namespace

int main() {
    try {
        ExactEvidenceAndNearestSpacing();
        FilteredEvidenceAdvancesCursor();
        BoundedRetentionAndEpochReset();
        IndependentSourcesAndMonotonicProvenance();
        InvalidConfigurationAndEvidence();
        std::cout << "acquisition keypose ledger tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
