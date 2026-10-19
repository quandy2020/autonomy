#pragma once
#include <cstdint>

namespace automsgs {
namespace msgs {
namespace builtin_interfaces {

struct Time {
    int32_t sec_ = 0;
    uint32_t nanosec_ = 0;
    int32_t sec() const {
        return sec_;
    }
    uint32_t nanosec() const {
        return nanosec_;
    }
    void set_sec(int32_t v) {
        sec_ = v;
    }
    void set_nanosec(uint32_t v) {
        nanosec_ = v;
    }
};

}  // namespace builtin_interfaces
}  // namespace msgs
}  // namespace automsgs
