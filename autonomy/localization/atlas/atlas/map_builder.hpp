/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 */

#pragma once

#include <memory>
#include <string>
#include <vector>

#include "atlas/sensor/sensor_types.hpp"

namespace autonomy::localization::atlas {

/**
 * @brief High-level owner and façade for an ORB-SLAM3 tracking session.
 *
 * MapBuilder validates sensor ordering, owns the SLAM worker threads, and
 * exposes poses in the world-from-camera convention. Unless stated otherwise,
 * members are safe to call only from the thread driving the sensor stream.
 */
class MapBuilder
{
public:
    /** @brief Creates a stopped session from the supplied configuration. */
    explicit MapBuilder(SystemConfig config);
    ~MapBuilder();

    MapBuilder(const MapBuilder&) = delete;
    MapBuilder& operator=(const MapBuilder&) = delete;

    /** @brief Starts the SLAM runtime. @return true when ready for input. */
    bool Start();
    /** @brief Stops workers; repeated calls are harmless. */
    void Stop();
    /** @return Whether the runtime has been started and not stopped. */
    bool started() const;

    /** @brief Queues one IMU sample. @return false for invalid ordering/data.
     */
    bool AddImu(const ImuSample& sample);
    /** @brief Tracks a monocular frame. */
    bool TrackMonocular(const ImageFrame& image,
                        TrackingResult* result = nullptr);
    /** @brief Tracks a timestamp-aligned stereo pair. */
    bool TrackStereo(const ImageFrame& left, const ImageFrame& right,
                     TrackingResult* result = nullptr);
    /** @brief Tracks a timestamp-aligned color and depth pair. */
    bool TrackRgbd(const ImageFrame& color, const ImageFrame& depth,
                   TrackingResult* result = nullptr);

    /** @brief Copies the most recent tracking result into @p result. */
    bool LatestResult(TrackingResult* result) const;
    /** @return Current non-null tracked map points in world coordinates. */
    std::vector<Eigen::Vector3f> TrackedMapPoints() const;

    /** @brief Stops map growth while retaining localization. */
    void ActivateLocalizationMode();
    /** @brief Re-enables local mapping. */
    void DeactivateLocalizationMode();
    /** @brief Clears tracker state and starts a fresh active map. */
    void Reset();

    /** @brief Stops the runtime and writes a TUM-format camera trajectory. */
    bool ShutdownAndSaveTumTrajectory(const std::string& path);
    /** @brief Stops the runtime and writes a EuRoC-format camera trajectory. */
    bool ShutdownAndSaveEurocTrajectory(const std::string& path);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace autonomy::localization::atlas
