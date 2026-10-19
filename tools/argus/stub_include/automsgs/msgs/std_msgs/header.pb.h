#pragma once
#include <string>

#include <automsgs/msgs/builtin_interfaces/time.pb.h>

namespace automsgs {
namespace msgs {
namespace std_msgs {

struct Header {
    builtin_interfaces::Time stamp_;
    std::string frame_id_;
    bool has_stamp_ = false;

    bool has_stamp() const {
        return has_stamp_;
    }
    const builtin_interfaces::Time& stamp() const {
        return stamp_;
    }
    builtin_interfaces::Time* mutable_stamp() {
        has_stamp_ = true;
        return &stamp_;
    }
    const std::string& frame_id() const {
        return frame_id_;
    }
    void set_frame_id(const std::string& v) {
        frame_id_ = v;
    }
};

}  // namespace std_msgs
}  // namespace msgs
}  // namespace automsgs
