/* Copyright 2026 The Openbot Authors. */
/** @file observation_buffer.cpp
 * @brief FIFO prefix evidence without conflating examination and valid fusion.
 */
#include "autonomy/perception/data/observation_buffer.hpp"
#include <algorithm>
#include <stdexcept>

namespace autonomy::perception {
namespace {
/** @brief Keeps the earliest source loss so later losses cannot hide an old gap. */
void RecordFirstLoss(ObservationCut& cut, SensorKind sensor, std::uint64_t sequence) {
    const auto index = sensor == SensorKind::kLidar ? 0U : 1U;
    if (!(cut.present & Mask(sensor)) || sequence < cut.sequence[index])
        cut.sequence[index] = sequence;
    cut.present |= Mask(sensor);
}
}  // namespace
ObservationBuffer::ObservationBuffer(std::size_t capacity) : capacity_(capacity) {
    if (!capacity) throw std::invalid_argument("observation buffer capacity is zero");
}
bool ObservationBuffer::Push(SensorObservation o, double now) {
    if ((o.sensor != SensorKind::kLidar && o.sensor != SensorKind::kRgbd) ||
        !o.sequence) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (o.map_epoch != epoch_) return false;
    auto& last = last_[o.sensor];
    // Replayed ordinals do not lose new input. A new ordinal whose acquisition
    // time moves backward is a discarded input, even if no later frame arrives.
    if (last.set && o.sequence <= last.sequence) return false;
    if ((last.set && o.stamp < last.stamp) || !o.Valid(now) ||
        observations_.size() >= capacity_) {
        discarded_cut_.Include(o.sensor, o.sequence);
        RecordFirstLoss(first_discarded_cut_, o.sensor, o.sequence);
        RecordLossLocked(o.sensor, o.sequence, o.sequence);
        return false;
    }
    if (last.set && o.sequence > last.sequence + 1) {
        RecordFirstLoss(first_gap_cut_, o.sensor, last.sequence + 1);
        RecordLossLocked(o.sensor, last.sequence + 1, o.sequence - 1);
    }
    last = {o.sequence, o.stamp, true};
    accepted_cut_.Include(o.sensor, o.sequence);
    observations_.push_back(std::move(o)); ++watermark_;
    return true;
}
std::vector<SensorObservation> ObservationBuffer::Drain(double now, std::uint64_t epoch,
                                                       std::size_t maximum) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<SensorObservation> result;
    for (std::size_t examined = 0; examined < maximum && !observations_.empty(); ++examined) {
        auto o = std::move(observations_.front()); observations_.pop_front();
        processed_cut_.Include(o.sensor, o.sequence);
        if (o.map_epoch == epoch && o.Valid(now)) result.push_back(std::move(o));
        else {
            discarded_cut_.Include(o.sensor, o.sequence);
            RecordFirstLoss(first_discarded_cut_, o.sensor, o.sequence);
            RecordLossLocked(o.sensor, o.sequence, o.sequence);
        }
    }
    return result;
}
void ObservationBuffer::Reset(std::uint64_t epoch) {
    std::lock_guard<std::mutex> lock(mutex_);
    epoch_ = epoch; ++watermark_; last_.clear(); observations_.clear(); accepted_cut_ = {};
    processed_cut_ = {}; discarded_cut_ = {};
    first_discarded_cut_ = {}; first_gap_cut_ = {};
    pending_first_loss_ = {}; pending_last_loss_ = {}; ++loss_generation_;
}
std::uint64_t ObservationBuffer::watermark() const {
    std::lock_guard<std::mutex> lock(mutex_); return watermark_;
}
bool ObservationBuffer::Empty() const {
    std::lock_guard<std::mutex> lock(mutex_); return observations_.empty();
}
ObservationCut ObservationBuffer::accepted_cut() const {
    std::lock_guard<std::mutex> lock(mutex_); return accepted_cut_;
}
ObservationCut ObservationBuffer::processed_cut() const {
    std::lock_guard<std::mutex> lock(mutex_); return processed_cut_;
}
ObservationCut ObservationBuffer::discarded_cut() const {
    std::lock_guard<std::mutex> lock(mutex_); return discarded_cut_;
}
ObservationCut ObservationBuffer::gap_cut() const {
    std::lock_guard<std::mutex> lock(mutex_); return first_gap_cut_;
}
bool ObservationBuffer::ProcessedThrough(const ObservationCut& cut) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ProcessedThroughLocked(cut);
}
bool ObservationBuffer::ProcessedThroughLocked(const ObservationCut& cut) const {
    if (!accepted_cut_.Covers(cut) || !processed_cut_.Covers(cut)) return false;
    return std::none_of(observations_.begin(), observations_.end(), [&](const auto& item) {
        const auto index = item.sensor == SensorKind::kLidar ? 0U : 1U;
        return (cut.present & Mask(item.sensor)) && item.sequence <= cut.sequence[index];
    });
}
bool ObservationBuffer::HasLossThrough(const ObservationCut& cut) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (unsigned index = 0; index < 2; ++index) {
        const auto source = SensorMask{static_cast<SensorMask>(1U << index)};
        if (!(cut.present & source)) continue;
        for (const auto* loss : {&first_gap_cut_, &first_discarded_cut_})
            if ((loss->present & source) && loss->sequence[index] <= cut.sequence[index]) return true;
    }
    return false;
}
void ObservationBuffer::Reject(SensorKind sensor, std::uint64_t sequence) {
    if ((sensor != SensorKind::kLidar && sensor != SensorKind::kRgbd) || !sequence) return;
    std::lock_guard<std::mutex> lock(mutex_);
    discarded_cut_.Include(sensor, sequence);
    RecordFirstLoss(first_discarded_cut_, sensor, sequence);
    RecordLossLocked(sensor, sequence, sequence);
}
void ObservationBuffer::RecordLossLocked(SensorKind sensor, std::uint64_t first,
                                         std::uint64_t last) {
    const auto index = sensor == SensorKind::kLidar ? 0U : 1U;
    const auto source = Mask(sensor);
    // A generation identifies one loss episode. Extending an interval that is
    // still open is the same episode: with continuous rejected input this used
    // to bump the generation on every frame, which turned one real gap into
    // hundreds of owner invalidations. Only a newly opened source, an earlier
    // interval start, or a separately disconnected interval starts a new one.
    const bool open = (pending_first_loss_.present & source) != 0;
    const bool began = !open;
    const bool earlier = open && first < pending_first_loss_.sequence[index];
    const bool separate = open && first > pending_last_loss_.sequence[index] + 1;
    RecordFirstLoss(pending_first_loss_, sensor, first);
    pending_last_loss_.Include(sensor, last);
    if (began || earlier || separate) ++loss_generation_;
}
bool ObservationBuffer::HasUnrecoveredLossThrough(const ObservationCut& cut) const {
    std::lock_guard<std::mutex> lock(mutex_);
    for (unsigned i = 0; i < 2; ++i)
        if ((cut.present & pending_first_loss_.present & (1U << i)) &&
            pending_first_loss_.sequence[i] <= cut.sequence[i]) return true;
    return false;
}
ObservationLossSnapshot ObservationBuffer::LossSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return {epoch_, loss_generation_, pending_first_loss_.present,
            pending_first_loss_, pending_last_loss_, accepted_cut_, first_gap_cut_, first_discarded_cut_};
}
bool ObservationBuffer::AcknowledgeRecovery(const ObservationLossSnapshot& checkpoint,
                                           const ObservationCut& replacement) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!checkpoint.pending_sources || checkpoint.map_epoch != epoch_ ||
        checkpoint.generation != loss_generation_ ||
        checkpoint.pending_sources != pending_first_loss_.present ||
        !(checkpoint.first_loss == pending_first_loss_) ||
        !(checkpoint.last_loss == pending_last_loss_) || !ProcessedThroughLocked(replacement)) return false;
    for (unsigned i = 0; i < 2; ++i) {
        if (!(checkpoint.pending_sources & (1U << i))) continue;
        if (!(replacement.present & (1U << i)) || replacement.sequence[i] <=
            std::max(checkpoint.admitted_cut.sequence[i], checkpoint.last_loss.sequence[i])) return false;
    }
    pending_first_loss_ = {}; pending_last_loss_ = {};
    return true;
}
SensorHealthMonitor::SensorHealthMonitor(SensorMode mode, SensorHealthOptions options)
    : mode_(mode), options_(options) {
    for (double value : {options.maximum_age, options.minimum_valid_fraction,
                         options.minimum_completeness, options.dropout_hold,
                         options.recovery_hold})
        if (!std::isfinite(value))
            throw std::invalid_argument("sensor health thresholds must be finite");
    if (Mask(mode) == 0 || Mask(mode) > 3 || options.maximum_age <= 0 ||
        options.minimum_valid_fraction <= 0 || options.minimum_valid_fraction > 1 ||
        options.minimum_completeness <= 0 || options.minimum_completeness > 1 ||
        options.dropout_hold < 0 || options.recovery_hold < 0)
        throw std::invalid_argument("invalid sensor health configuration");
}
void SensorHealthMonitor::Observe(const SensorObservation& o) {
    if (!(Mask(mode_) & Mask(o.sensor))) return;
    auto& s = states_[o.sensor];
    if (!s.latest || o.map_epoch > s.latest->map_epoch ||
        (o.map_epoch == s.latest->map_epoch && o.sequence > s.latest->sequence &&
         o.stamp >= s.latest->stamp)) s.latest = o;
}
SensorReadiness SensorHealthMonitor::Evaluate(double now, SensorMask localization,
                                              const std::vector<SensorMask>& models) {
    if (!std::isfinite(now) || now < last_now_) return {0, false, epoch_};
    last_now_ = now;
    SensorMask active = 0;
    for (SensorKind kind : {SensorKind::kLidar, SensorKind::kRgbd}) {
        if (!(Mask(mode_)&Mask(kind))) continue;
        auto& s = states_[kind];
        const bool good = s.latest && s.latest->Valid(now) &&
            now-s.latest->stamp <= options_.maximum_age &&
            s.latest->valid_fraction >= options_.minimum_valid_fraction &&
            s.latest->scan_completeness >= options_.minimum_completeness;
        if (good) {
            s.bad_since.reset();
            if (!s.good_since) s.good_since = now;
            if (!s.active && now-*s.good_since >= options_.recovery_hold) {
                s.active = true; ++epoch_;
            }
        } else {
            s.good_since.reset();
            if (!s.bad_since) s.bad_since = now;
            if (s.active && now-*s.bad_since >= options_.dropout_hold) {
                s.active = false; ++epoch_;
            }
        }
        // Hysteresis never authorizes stale geometry during its dropout window.
        if (s.active && good && (localization&Mask(kind))) active |= Mask(kind);
    }
    const bool supported = std::find(models.begin(), models.end(), active) != models.end();
    return {active, active != 0 && supported, epoch_};
}
}  // namespace autonomy::perception
