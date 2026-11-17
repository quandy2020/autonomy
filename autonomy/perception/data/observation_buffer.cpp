/* Copyright 2026 The Openbot Authors. */
#include "autonomy/perception/data/observation_buffer.hpp"
#include <algorithm>
#include <stdexcept>

namespace autonomy::perception {
ObservationBuffer::ObservationBuffer(std::size_t capacity) : capacity_(capacity) {
    if (!capacity) throw std::invalid_argument("observation buffer capacity is zero");
}
bool ObservationBuffer::Push(SensorObservation o, double now) {
    if (!o.Valid(now)) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    if (o.map_epoch != epoch_ || observations_.size() >= capacity_) return false;
    auto& last = last_[o.sensor];
    if (last.set && (o.sequence <= last.sequence || o.stamp < last.stamp)) return false;
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
        else discarded_cut_.Include(o.sensor, o.sequence);
    }
    return result;
}
void ObservationBuffer::Reset(std::uint64_t epoch) {
    std::lock_guard<std::mutex> lock(mutex_);
    epoch_ = epoch; ++watermark_; last_.clear(); observations_.clear(); accepted_cut_ = {};
    processed_cut_ = {}; discarded_cut_ = {};
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
SensorHealthMonitor::SensorHealthMonitor(SensorMode mode, SensorHealthOptions options)
    : mode_(mode), options_(options) {
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
