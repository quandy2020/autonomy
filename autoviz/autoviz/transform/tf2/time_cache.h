/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @file time_cache.h
 * @brief Per-frame transform history: @ref TimeCache and @ref StaticCache.
 *
 * @ref BufferCore stores one cache per frame edge. Dynamic frames use a
 * time-sorted list with interpolation; static frames keep a single sample.
 *
 * @see TransformStorage
 * @see BufferCore
 */

#ifndef TF2_TIME_CACHE_H
#define TF2_TIME_CACHE_H

#include <list>
#include <sstream>

#include "autoviz/transform/tf2/time.h"
#include "autoviz/transform/tf2/transform_storage.h"

// #include <ros/message_forward.h>
// #include <ros/time.h>

#include <memory>

// namespace geometry_msgs {
// ROS_DECLARE_MESSAGE(TransformStamped);
// }
//

namespace autoviz {
namespace transform {
namespace tf2 {

/**
 * @brief Pair of latest sample time and parent compact frame id.
 */
typedef std::pair<Time, CompactFrameID> P_TimeAndFrameID;

/**
 * @class TimeCacheInterface
 * @brief Abstract per-edge transform history used by @ref BufferCore.
 */
class TimeCacheInterface
{
public:
    /**
     * @brief Looks up (possibly interpolates) a transform at @p time.
     *
     * @param time Query time (nanoseconds); @c 0 may mean latest.
     * @param[out] data_out Filled storage on success.
     * @param[out] error_str Optional human-readable failure reason.
     * @return @c false if data unavailable (caller should throw lookup error).
     */
    virtual bool getData(Time time, TransformStorage& data_out,
                         std::string* error_str =
                             0) = 0;  // returns false if data unavailable
                                      // (should be thrown as lookup exception

    /**
     * @brief Inserts a new sample into the cache.
     * @param new_data Transform sample to store.
     * @return @c true if inserted.
     */
    virtual bool insertData(const TransformStorage& new_data) = 0;

    /** @brief Removes all stored samples. */
    virtual void clearList() = 0;

    /**
     * @brief Returns the parent compact frame id at @p time.
     *
     * @param time Query time.
     * @param[out] error_str Optional failure reason.
     * @return Parent @ref CompactFrameID, or @c 0 if unavailable.
     */
    virtual CompactFrameID getParent(Time time, std::string* error_str) = 0;

    /**
     * @brief Latest stored time and its parent frame id.
     * @return Pair (@c time, @c parent); parent is @c 0 if empty.
     */
    virtual P_TimeAndFrameID getLatestTimeAndParent() = 0;

    /** @brief Number of stored samples (debugging). */
    virtual unsigned int getListLength() = 0;

    /** @brief Newest sample timestamp (debugging). */
    virtual Time getLatestTimestamp() = 0;

    /** @brief Oldest sample timestamp (debugging). */
    virtual Time getOldestTimestamp() = 0;
};

/** Shared pointer to a @ref TimeCacheInterface. */
typedef std::shared_ptr<TimeCacheInterface> TimeCacheInterfacePtr;

/**
 * @class TimeCache
 * @brief Time-sorted transform list with interpolation for dynamic frames.
 *
 * Maintains a linked list of @ref TransformStorage samples, prunes by
 * @c max_storage_time_, and interpolates between bracketing samples.
 */
class TimeCache : public TimeCacheInterface
{
public:
    /** Nano-seconds below which interpolation is skipped. */
    static const int MIN_INTERPOLATION_DISTANCE =
        5;  //!< Number of nano-seconds to not interpolate below.
    /** Hard cap on list length to bound memory. */
    static const unsigned int MAX_LENGTH_LINKED_LIST =
        1000000;  //!< Maximum length of linked list, to make sure not to
                  //!< be able to use unlimited memory.
    /** Default max history length (1e9 ns = 1 s in this fork's constant). */
    static const int64_t DEFAULT_MAX_STORAGE_TIME =
        1ULL * 1000000000LL;  //!< default value of 10 seconds storage

    /**
     * @brief Constructs a cache retaining at most @p max_storage_time of history.
     * @param max_storage_time Max age of retained samples (nanoseconds).
     */
    TimeCache(Duration max_storage_time = DEFAULT_MAX_STORAGE_TIME);

    /// Virtual methods

    /**
     * @brief Interpolated lookup at @p time.
     * @param time Query time.
     * @param[out] data_out Result storage.
     * @param[out] error_str Optional error text.
     * @return @c true on success.
     */
    virtual bool getData(Time time, TransformStorage& data_out,
                         std::string* error_str = 0);
    /**
     * @brief Inserts @p new_data and prunes old samples.
     * @param new_data Sample to insert.
     * @return @c true if inserted.
     */
    virtual bool insertData(const TransformStorage& new_data);
    /** @brief Clears all samples. */
    virtual void clearList();
    /**
     * @brief Parent frame at @p time.
     * @param time Query time.
     * @param[out] error_str Optional error text.
     * @return Compact parent id.
     */
    virtual CompactFrameID getParent(Time time, std::string* error_str);
    /** @brief Latest time and parent pair. */
    virtual P_TimeAndFrameID getLatestTimeAndParent();

    /// Debugging information methods
    /** @brief Sample count. */
    virtual unsigned int getListLength();
    /** @brief Newest stamp. */
    virtual Time getLatestTimestamp();
    /** @brief Oldest stamp. */
    virtual Time getOldestTimestamp();

private:
    typedef std::list<TransformStorage> L_TransformStorage;
    /** Time-sorted storage (newest typically at front per tf2 convention). */
    L_TransformStorage storage_;

    /** Max age of retained samples. */
    Duration max_storage_time_;

    /**
     * @brief Finds bracketing samples for interpolation (storage already locked).
     *
     * @param[out] one Earlier / first bracketing sample.
     * @param[out] two Later / second bracketing sample.
     * @param target_time Query time.
     * @param[out] error_str Optional error text.
     * @return Status byte used by getData (tf2 internal codes).
     */
    inline uint8_t findClosest(TransformStorage*& one, TransformStorage*& two,
                               Time target_time, std::string* error_str);

    /**
     * @brief Linearly interpolates pose between @p one and @p two at @p time.
     *
     * @param one First bracketing sample.
     * @param two Second bracketing sample.
     * @param time Query time between the two stamps.
     * @param[out] output Interpolated storage.
     */
    inline void interpolate(const TransformStorage& one,
                            const TransformStorage& two, Time time,
                            TransformStorage& output);

    /** Drops samples older than @c max_storage_time_. */
    void pruneList();
};

/**
 * @class StaticCache
 * @brief Single-sample cache for static transforms (@c /tf_static).
 */
class StaticCache : public TimeCacheInterface
{
public:
    /// Virtual methods

    /**
     * @brief Returns the static sample regardless of @p time (when present).
     *
     * @param time Ignored for static data (API symmetry with @ref TimeCache).
     * @param[out] data_out Copied static storage.
     * @param[out] error_str Optional error text.
     * @return @c false if no static transform has been set.
     */
    virtual bool getData(
        Time time, TransformStorage& data_out,
        std::string* error_str = 0);  // returns false if data unavailable
                                      // (should be thrown as lookup exception
    /**
     * @brief Replaces the single static sample.
     * @param new_data New static transform.
     * @return @c true on success.
     */
    virtual bool insertData(const TransformStorage& new_data);
    /** @brief Clears the static sample. */
    virtual void clearList();
    /**
     * @brief Parent of the static transform.
     * @param time Unused (API symmetry).
     * @param[out] error_str Optional error text.
     * @return Compact parent id.
     */
    virtual CompactFrameID getParent(Time time, std::string* error_str);
    /** @brief Static stamp and parent pair. */
    virtual P_TimeAndFrameID getLatestTimeAndParent();

    /// Debugging information methods
    /** @brief @c 0 or @c 1 depending on whether a sample exists. */
    virtual unsigned int getListLength();
    /** @brief Stamp of the static sample. */
    virtual Time getLatestTimestamp();
    /** @brief Same as latest for a single sample. */
    virtual Time getOldestTimestamp();

private:
    /** Sole static transform sample. */
    TransformStorage storage_;
};

}  // namespace tf2
}  // namespace transform
}  // namespace autoviz

#endif  // TF2_TIME_CACHE_H
