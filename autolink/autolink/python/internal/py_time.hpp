/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <unistd.h>
#include <memory>

#include "autolink/autolink.hpp"
#include "autolink/common/init.hpp"
#include "autolink/time/rate.hpp"
#include "autolink/time/time.hpp"

namespace autolink {

class PyTime
{
public:
    PyTime() = default;
    explicit PyTime(uint64_t nanoseconds) {
        time_ = Time(nanoseconds);
    }

    static PyTime now() {
        PyTime t;
        t.time_ = Time::Now();
        return t;
    }

    static PyTime mono_time() {
        PyTime t;
        t.time_ = Time::MonoTime();
        return t;
    }

    static void sleep_until(uint64_t nanoseconds) {
        Time::SleepUntil(Time(nanoseconds));
    }

    double to_sec() const {
        return time_.ToSecond();
    }

    uint64_t to_nsec() const {
        return time_.ToNanosecond();
    }

private:
    Time time_;
};

class PyDuration
{
public:
    explicit PyDuration(int64_t nanoseconds) {
        duration_ = std::make_shared<Duration>(nanoseconds);
    }

    void sleep() const {
        return duration_->Sleep();
    }

private:
    std::shared_ptr<Duration> duration_ = nullptr;
};

class PyRate
{
public:
    explicit PyRate(uint64_t nanoseconds) {
        rate_ = std::make_shared<Rate>(nanoseconds);
    }

    void sleep() const {
        return rate_->Sleep();
    }
    void reset() const {
        return rate_->Reset();
    }
    uint64_t get_cycle_time() const {
        return rate_->CycleTime().ToNanosecond();
    }
    uint64_t get_expected_cycle_time() const {
        return rate_->ExpectedCycleTime().ToNanosecond();
    }

private:
    std::shared_ptr<Rate> rate_ = nullptr;
};

}  // namespace autolink