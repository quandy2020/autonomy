#pragma once
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include <automsgs/msgs/std_msgs/header.pb.h>

namespace automsgs {
namespace msgs {
namespace sensor_msgs {

struct PointField {
    enum DataType {
        INT8 = 1,
        UINT8 = 2,
        INT16 = 3,
        UINT16 = 4,
        INT32 = 5,
        UINT32 = 6,
        FLOAT32 = 7,
        FLOAT64 = 8,
    };
    std::string name_;
    uint32_t offset_ = 0;
    int datatype_ = FLOAT32;
    uint32_t count_ = 1;

    const std::string& name() const {
        return name_;
    }
    uint32_t offset() const {
        return offset_;
    }
    int datatype() const {
        return datatype_;
    }
    uint32_t count() const {
        return count_;
    }
    void set_name(const std::string& n) {
        name_ = n;
    }
    void set_offset(uint32_t o) {
        offset_ = o;
    }
    void set_datatype(int d) {
        datatype_ = d;
    }
    void set_count(uint32_t c) {
        count_ = c;
    }
};

struct PointCloud2 {
    std_msgs::Header header_;
    bool has_header_ = false;
    std::vector<PointField> fields_;
    std::string data_;
    uint32_t width_ = 0;
    uint32_t height_ = 0;
    uint32_t point_step_ = 0;
    uint32_t row_step_ = 0;
    bool is_dense_ = true;

    void Clear() {
        *this = PointCloud2{};
    }
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
    const std::vector<PointField>& fields() const {
        return fields_;
    }
    PointField* add_fields() {
        fields_.emplace_back();
        return &fields_.back();
    }
    const std::string& data() const {
        return data_;
    }
    uint32_t width() const {
        return width_;
    }
    uint32_t height() const {
        return height_;
    }
    uint32_t point_step() const {
        return point_step_;
    }
    uint32_t row_step() const {
        return row_step_;
    }
    bool is_dense() const {
        return is_dense_;
    }
    void set_width(uint32_t v) {
        width_ = v;
    }
    void set_height(uint32_t v) {
        height_ = v;
    }
    void set_point_step(uint32_t v) {
        point_step_ = v;
    }
    void set_row_step(uint32_t v) {
        row_step_ = v;
    }
    void set_is_dense(bool v) {
        is_dense_ = v;
    }
    void set_data(std::string&& v) {
        data_ = std::move(v);
    }
    void set_data(const void* p, size_t n) {
        data_.assign(static_cast<const char*>(p), n);
    }
};

}  // namespace sensor_msgs
}  // namespace msgs
}  // namespace automsgs
