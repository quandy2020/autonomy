#pragma once
#include <cstdint>
#include <string>
#include <utility>

#include <automsgs/msgs/std_msgs/header.pb.h>

namespace automsgs {
namespace msgs {
namespace sensor_msgs {

struct Image {
    std_msgs::Header header_;
    bool has_header_ = false;
    uint32_t height_ = 0;
    uint32_t width_ = 0;
    std::string encoding_;
    bool is_bigendian_ = false;
    uint32_t step_ = 0;
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
    uint32_t height() const {
        return height_;
    }
    uint32_t width() const {
        return width_;
    }
    const std::string& encoding() const {
        return encoding_;
    }
    uint32_t step() const {
        return step_;
    }
    const std::string& data() const {
        return data_;
    }
    void set_height(uint32_t v) {
        height_ = v;
    }
    void set_width(uint32_t v) {
        width_ = v;
    }
    void set_encoding(const std::string& v) {
        encoding_ = v;
    }
    void set_is_bigendian(bool v) {
        is_bigendian_ = v;
    }
    void set_step(uint32_t v) {
        step_ = v;
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
