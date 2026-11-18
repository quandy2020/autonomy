/* Copyright 2026 The Openbot Authors. */
/** @file observation_buffer.hpp
 * @brief Bounded observations, finite-prefix processing and source health.
 */
#pragma once
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {
/** @brief Immutable checkpoint of unresolved loss and its post-loss admission boundary. */
struct ObservationLossSnapshot {
    std::uint64_t map_epoch = 0;  ///< Alignment whose observations were lost.
    std::uint64_t generation = 0;  ///< Changes on new loss or Reset, never on ordinary admission.
    SensorMask pending_sources = 0;  ///< Sources requiring an owner-verified recovery.
    ObservationCut first_loss{};  ///< Earliest unresolved sequence for each source.
    ObservationCut last_loss{};  ///< Last unresolved sequence, including gap interval endpoints.
    ObservationCut admitted_cut{};  ///< Atomic admission boundary when the owner captured this checkpoint.
    ObservationCut historical_gaps{};  ///< Earliest audited source gap, retained after recovery.
    ObservationCut historical_discards{};  ///< Earliest audited source discard, retained after recovery.
};
/** @brief Thread-safe admission queue with explicit source-gap and loss evidence. */
class ObservationBuffer {
public:
    /** @brief Creates a queue with a nonzero admission capacity. */
    explicit ObservationBuffer(std::size_t capacity = 64);
    /** @brief Admit a valid increasing ordinal with nondecreasing acquisition time.
     * @details New ordinals rejected for backward time, validity or capacity are
     * immediately recorded as lost. Replayed ordinals do not manufacture loss.
     */
    bool Push(SensorObservation observation, double now);
    /** @brief Examines at most maximum queued inputs; expired ones are recorded as lost. */
    std::vector<SensorObservation> Drain(double now, std::uint64_t map_epoch,
        std::size_t maximum = std::numeric_limits<std::size_t>::max());
    /** @brief Clears all source evidence when the alignment/session baseline changes. */
    void Reset(std::uint64_t map_epoch);
    /** @brief Admission/reset counter for compatibility diagnostics. */
    std::uint64_t watermark() const;
    /** @brief Whether the current queue is empty; not a map-fusion receipt. */
    bool Empty() const;
    /** @brief Source endpoints of a finite admitted input prefix. */
    ObservationCut accepted_cut() const;
    /** @brief Last examined source sequences, including rejected expired inputs. */
    ObservationCut processed_cut() const;
    /** @brief Latest discarded sequences for diagnostics, not valid coverage. */
    ObservationCut discarded_cut() const;
    /** @brief Earliest missing sequence after each source's session baseline. */
    ObservationCut gap_cut() const;
    /** @brief Checks every admitted item through cut has left the queue.
     * Later queued suffixes do not invalidate this proof. This confirms local
     * examination only; it does not assert usable coverage or mapper fusion.
     */
    bool ProcessedThrough(const ObservationCut& cut) const;
    /** @brief Checks for a sequence gap or discarded input inside a finite prefix. */
    bool HasLossThrough(const ObservationCut& cut) const;
    /** @brief Tests unresolved loss; acknowledged historical loss remains in HasLossThrough. */
    bool HasUnrecoveredLossThrough(const ObservationCut& cut) const;
    /** @brief Atomically captures unresolved loss and the current admission boundary. */
    ObservationLossSnapshot LossSnapshot() const;
    /** @brief Commits an owner's verified replacement baseline without erasing history.
     * The owner must first verify scope, measured coverage and actual mapper/graph
     * receipts. This queue checks epoch, loss generation, a processed replacement
     * suffix and absence of concurrent new loss; it does not verify map fusion.
     * @return False on stale checkpoints or insufficient replacement processing.
     */
    bool AcknowledgeRecovery(const ObservationLossSnapshot& checkpoint,
                             const ObservationCut& replacement_cut);
    /** @brief Records a drained input rejected by frame, pose or quality validation. */
    void Reject(SensorKind sensor, std::uint64_t sequence);
private:
    /** @brief Records an unresolved loss interval while holding mutex_. */
    void RecordLossLocked(SensorKind sensor, std::uint64_t first, std::uint64_t last);
    /** @brief Checks finite FIFO examination under the already-held mutex_. */
    bool ProcessedThroughLocked(const ObservationCut& cut) const;
    /** @brief Most recent accepted sequence and time for one source. */
    struct Last {
        std::uint64_t sequence = 0;  ///< Monotonic accepted endpoint.
        double stamp = 0;  ///< Last accepted acquisition time.
        bool set = false;  ///< A source baseline has been established.
    };
    mutable std::mutex mutex_;  ///< Protects admission, draining and prefix evidence.
    std::size_t capacity_;  ///< Maximum pending observations.
    std::uint64_t epoch_ = 0;  ///< Alignment epoch accepted by this queue.
    std::uint64_t watermark_ = 0;  ///< Diagnostic admission/reset counter.
    ObservationCut accepted_cut_{};  ///< Current source admission endpoints.
    ObservationCut processed_cut_{};  ///< Current examined endpoints.
    ObservationCut discarded_cut_{};  ///< Latest rejected source observations.
    ObservationCut first_discarded_cut_{};  ///< Earliest loss for each source.
    ObservationCut first_gap_cut_{};  ///< Earliest skipped sequence after source baseline.
    ObservationCut pending_first_loss_{};  ///< Earliest unresolved loss, independent of audit history.
    ObservationCut pending_last_loss_{};  ///< Latest unresolved interval endpoint per source.
    std::uint64_t loss_generation_ = 0;  ///< Monotonic checkpoint generation across Reset.
    std::map<SensorKind, Last> last_;  ///< Per-source monotonic admission checks.
    std::deque<SensorObservation> observations_;  ///< Bounded FIFO admitted inputs.
};

/** @brief Finite freshness, quality and hysteresis thresholds for configured sources. */
struct SensorHealthOptions {
    double maximum_age = 0.5;  ///< Maximum acquisition age in owner-clock seconds; strictly positive.
    double minimum_valid_fraction = 0.1;  ///< Minimum valid-measurement fraction in (0, 1].
    double minimum_completeness = 0.8;  ///< Minimum delivered scan fraction in (0, 1].
    double dropout_hold = 0.1;  ///< Nonnegative bad-condition hold in seconds; never extends evidence validity.
    double recovery_hold = 0.5;  ///< Nonnegative sustained-good hold in seconds before source activation.
};
/** @brief Current usable sources and the generation of hysteresis-state changes. */
struct SensorReadiness {
    SensorMask active_sensors = 0;  ///< Sources with fresh evidence and current localization support.
    bool ready = false;  ///< Nonempty active mask exactly matches a validated controller input mode.
    std::uint64_t sensor_epoch = 0;  ///< Changes when a source enters or leaves its hysteresis-active state.
};
/** @brief Gates execution sensing by freshness, localization and validated control modes.
 * A dropout hold stabilizes configured state only. Evaluate excludes stale
 * evidence immediately, even before its hysteresis-active state changes.
 */
class SensorHealthMonitor {
public:
    /** @brief Creates a monitor for the startup sensor mode.
     * @param mode Configured source set; runtime availability does not change this policy.
     * @param options Finite thresholds with valid fraction and timing ranges.
     * @throws std::invalid_argument If the mode or any threshold is invalid.
     */
    explicit SensorHealthMonitor(SensorMode mode, SensorHealthOptions options = {});
    /** @brief Retains only a newer configured-source observation in the same or a later epoch.
     * @param observation Measured input with source identity, acquisition time and validity.
     */
    void Observe(const SensorObservation& observation);
    /** @brief Evaluates usable evidence without allowing hysteresis to extend its lifetime.
     * @param now Finite monotonic owner time in seconds; reversed time returns not ready.
     * @param localization_support Sources currently supported by valid localization.
     * @param controller_modes Exact source masks of independently validated execution backends.
     * @return Available sources and readiness, with the current capability generation.
     */
    SensorReadiness Evaluate(double now, SensorMask localization_support,
                             const std::vector<SensorMask>& controller_modes);
private:
    /** @brief Source-local evidence and sustained-good/bad timing. */
    struct State {
        std::optional<SensorObservation> latest;  ///< Latest monotonic evidence for this source.
        bool active = false;  ///< Hysteresis state; does not itself authorize execution.
        std::optional<double> good_since;  ///< Start of the current uninterrupted good interval.
        std::optional<double> bad_since;  ///< Start of the current uninterrupted bad interval.
    };
    SensorMode mode_;  ///< Fixed configured mode; a dropout does not narrow task requirements.
    SensorHealthOptions options_;  ///< Validated finite quality and timing thresholds.
    std::map<SensorKind, State> states_;  ///< Independently maintained source evidence.
    std::uint64_t epoch_ = 0;  ///< Generation of actual hysteresis-active transitions.
    double last_now_ = -std::numeric_limits<double>::infinity();  ///< Last accepted owner time.
};
}  // namespace autonomy::perception
