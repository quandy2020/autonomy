/* Copyright 2026 The Openbot Authors. */
#pragma once
#include <deque>
#include <map>
#include <mutex>
#include <optional>
#include "autonomy/perception/data/sensor_observation.hpp"

namespace autonomy::perception {
class ObservationBuffer {
public:
    explicit ObservationBuffer(std::size_t capacity = 64);
    bool Push(SensorObservation observation, double now);
    std::vector<SensorObservation> Drain(double now, std::uint64_t map_epoch,
        std::size_t maximum = std::numeric_limits<std::size_t>::max());
    void Reset(std::uint64_t map_epoch);
    std::uint64_t watermark() const;
    bool Empty() const;
    ObservationCut accepted_cut() const;
    ObservationCut processed_cut() const;
    ObservationCut discarded_cut() const;
private:
    struct Last { std::uint64_t sequence = 0; double stamp = 0; bool set = false; };
    mutable std::mutex mutex_;
    std::size_t capacity_;
    std::uint64_t epoch_ = 0, watermark_ = 0;
    ObservationCut accepted_cut_{};
    ObservationCut processed_cut_{}, discarded_cut_{};
    std::map<SensorKind, Last> last_;
    std::deque<SensorObservation> observations_;
};

struct SensorHealthOptions {
    double maximum_age = 0.5, minimum_valid_fraction = 0.1;
    double minimum_completeness = 0.8, dropout_hold = 0.1, recovery_hold = 0.5;
};
struct SensorReadiness {
    SensorMask active_sensors = 0;
    bool ready = false;
    std::uint64_t sensor_epoch = 0;
};
class SensorHealthMonitor {
public:
    explicit SensorHealthMonitor(SensorMode mode, SensorHealthOptions options = {});
    void Observe(const SensorObservation& observation);
    SensorReadiness Evaluate(double now, SensorMask localization_support,
                             const std::vector<SensorMask>& controller_modes);
private:
    struct State { std::optional<SensorObservation> latest; bool active = false;
                   std::optional<double> good_since, bad_since; };
    SensorMode mode_;
    SensorHealthOptions options_;
    std::map<SensorKind, State> states_;
    std::uint64_t epoch_ = 0;
    double last_now_ = -std::numeric_limits<double>::infinity();
};
}  // namespace autonomy::perception
