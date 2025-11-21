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

#include <cstdint>
#include <memory>
#include <string>

namespace autolink {
namespace event {

enum class SchedPerf : int {
    UNKNOWN = 0,
};

enum class TransPerf : int {
    UNKNOWN = 0,
    TRANSMIT_BEGIN = 1,
    DISPATCH = 2,
    NOTIFY = 3,
};

class EventBase
{
public:
    virtual ~EventBase() = default;
    virtual std::string SerializeToString() const {
        return serialized_;
    }

    void set_eid(int eid) {
        eid_ = eid;
    }
    void set_stamp(uint64_t stamp) {
        stamp_ = stamp;
    }

protected:
    int eid_ = 0;
    uint64_t stamp_ = 0;
    std::string serialized_;
};

class SchedEvent : public EventBase
{
public:
    void set_cr_state(int s) {
        cr_state_ = s;
    }
    void set_cr_id(uint64_t id) {
        cr_id_ = id;
    }
    void set_proc_id(int id) {
        proc_id_ = id;
    }
    std::string SerializeToString() const override {
        return "SCHED," + std::to_string(eid_) + "," + std::to_string(stamp_) +
               "," + std::to_string(cr_id_) + "," + std::to_string(proc_id_) +
               "," + std::to_string(cr_state_);
    }

private:
    int cr_state_ = -1;
    uint64_t cr_id_ = 0;
    int proc_id_ = 0;
};

class TransportEvent : public EventBase
{
public:
    void set_channel_id(uint64_t id) {
        channel_id_ = id;
    }
    void set_msg_seq(uint64_t seq) {
        msg_seq_ = seq;
    }
    void set_adder(const std::string& a) {
        adder_ = a;
    }
    std::string SerializeToString() const override {
        return "TRANS," + std::to_string(eid_) + "," + std::to_string(stamp_) +
               "," + std::to_string(channel_id_) + "," +
               std::to_string(msg_seq_) + "," + adder_;
    }

private:
    uint64_t channel_id_ = 0;
    uint64_t msg_seq_ = 0;
    std::string adder_;
};

}  // namespace event
}  // namespace autolink
