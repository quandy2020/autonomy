/*
 * Copyright 2026 The Openbot Authors
 *
 * Licensed under the Apache License, Version 2.0.
 */
#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "autonomy/common/math/so3.hpp"

namespace autonomy {
namespace common {
namespace math {

template <typename FloatType>
class Sim3
{
public:
    using Vector = Eigen::Matrix<FloatType, 3, 1>;
    using Matrix3 = Eigen::Matrix<FloatType, 3, 3>;
    using Quaternion = Eigen::Quaternion<FloatType>;

    Sim3()
        : rotation_(SO3<FloatType>::Identity()),
          translation_(Vector::Zero()),
          scale_(FloatType(1)) {}

    Sim3(const SO3<FloatType>& rotation, const Vector& translation,
         FloatType scale)
        : rotation_(rotation), translation_(translation), scale_(scale) {}

    Sim3(const Quaternion& rotation, const Vector& translation)
        : Sim3(rotation, translation, FloatType(1)) {}

    Sim3(const Quaternion& rotation, const Vector& translation, FloatType scale)
        : Sim3(SO3<FloatType>(rotation), translation, scale) {}

    Sim3(const Matrix3& rotation, const Vector& translation, FloatType scale)
        : Sim3(SO3<FloatType>(rotation), translation, scale) {}

    const SO3<FloatType>& rotation() const {
        return rotation_;
    }
    SO3<FloatType>& rotation() {
        return rotation_;
    }
    Matrix3 rotationMatrix() const {
        return rotation_.Matrix();
    }

    const Vector& translation() const {
        return translation_;
    }
    Vector& translation() {
        return translation_;
    }

    FloatType scale() const {
        return scale_;
    }

    Sim3 inverse() const {
        const SO3<FloatType> inverse_rotation = rotation_.Inverse();
        const FloatType inverse_scale = FloatType(1) / scale_;
        return Sim3(inverse_rotation,
                    -(inverse_scale * (inverse_rotation * translation_)),
                    inverse_scale);
    }

    template <typename OtherType>
    Sim3<OtherType> cast() const {
        return Sim3<OtherType>(rotation_.template Cast<OtherType>(),
                               translation_.template cast<OtherType>(),
                               static_cast<OtherType>(scale_));
    }

private:
    SO3<FloatType> rotation_;
    Vector translation_;
    FloatType scale_;
};

template <typename FloatType>
Sim3<FloatType> operator*(const Sim3<FloatType>& left,
                          const Sim3<FloatType>& right) {
    return Sim3<FloatType>(
        left.rotation() * right.rotation(),
        left.scale() * (left.rotation() * right.translation()) +
            left.translation(),
        left.scale() * right.scale());
}

template <typename FloatType>
typename Sim3<FloatType>::Vector operator*(
    const Sim3<FloatType>& transform,
    const typename Sim3<FloatType>::Vector& point) {
    return transform.scale() * (transform.rotation() * point) +
           transform.translation();
}

using Sim3d = Sim3<double>;
using Sim3f = Sim3<float>;

}  // namespace math
}  // namespace common
}  // namespace autonomy
