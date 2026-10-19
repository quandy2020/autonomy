#pragma once

namespace automsgs {
namespace msgs {
namespace geometry_msgs {

struct Vector3 {
    double x_ = 0;
    double y_ = 0;
    double z_ = 0;
    double x() const {
        return x_;
    }
    double y() const {
        return y_;
    }
    double z() const {
        return z_;
    }
    void set_x(double v) {
        x_ = v;
    }
    void set_y(double v) {
        y_ = v;
    }
    void set_z(double v) {
        z_ = v;
    }
};

}  // namespace geometry_msgs
}  // namespace msgs
}  // namespace automsgs
