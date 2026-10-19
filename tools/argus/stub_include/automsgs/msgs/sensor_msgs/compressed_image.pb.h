#pragma once
#include <cstdint>
#include <string>
#include <utility>

#include <automsgs/msgs/std_msgs/header.pb.h>

namespace automsgs {
namespace msgs {
namespace sensor_msgs {

struct CompressedImage {
    std_msgs::Header header_;
    bool has_header_ = false;
    std::string format_;
    std::string data_;

    bool has_header() const {
        return has_header_;
    }
    const std_msgs::Header& header() const {
        return header_;
    }
    std_msgs::Header* mutable_header() {
        has_header_ = true;
        return &header_;
    }
    const std::string& format() const {
        return format_;
    }
    const std::string& data() const {
        return data_;
    }
    void set_format(const std::string& v) {
        format_ = v;
    }
    void set_data(const void* p, size_t n) {
        data_.assign(static_cast<const char*>(p), n);
    }
    void set_data(std::string&& v) {
        data_ = std::move(v);
    }
};

}  // namespace sensor_msgs
}  // namespace msgs
}  // namespace automsgs
