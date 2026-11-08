/* Copyright 2026 The Navigation Authors (navigation) */
#include <cmath>
#include <iostream>

#include "navigation/common/pose2d.hpp"

namespace
{

bool Close(double lhs, double rhs, double epsilon = 1e-9)
{
    return std::abs(lhs - rhs) <= epsilon;
}

}  // namespace

int main()
{
    using navigation::common::Compose;
    using navigation::common::Distance;
    using navigation::common::Inverse;
    using navigation::common::Pose2D;

    const Pose2D parent = navigation::common::MakePose2D(1.0, 2.0, 1.5707963267948966);
    const Pose2D child = navigation::common::MakePose2D(1.0, 0.0, 0.0);
    const Pose2D composed = Compose(parent, child);
    if (!Close(composed.x(), 1.0) || !Close(composed.y(), 3.0) ||
        !Close(composed.theta(), parent.theta()))
    {
        std::cerr << "Compose failed\n";
        return 1;
    }

    const Pose2D identity = Compose(Inverse(parent), parent);
    if (!Close(identity.x(), 0.0) || !Close(identity.y(), 0.0) ||
        !Close(identity.theta(), 0.0))
    {
        std::cerr << "Inverse failed\n";
        return 1;
    }

    const Pose2D other = navigation::common::MakePose2D(4.0, 6.0, 0.0);
    if (!Close(Distance(parent, other), 5.0))
    {
        std::cerr << "Distance failed\n";
        return 1;
    }
    return 0;
}
