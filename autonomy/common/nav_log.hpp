/*
 * Compact navigation diagnostics. Prefix every lifecycle line with "nav "
 * so operators can grep one token:
 *   grep ' nav '  or  grep 'nav \(goal\|plan\|smooth\|follow\|recover\)'
 *
 * Fields stay key=value with 3-decimal poses: xy=(x,y).
 */

#pragma once

#include <iomanip>
#include <ostream>
#include <string>

namespace autonomy {
namespace common {

struct NavXY {
    double x{0.0};
    double y{0.0};
};

inline std::ostream& operator<<(std::ostream& os, NavXY p) {
    const auto flags = os.flags();
    const auto prec = os.precision();
    os << std::fixed << std::setprecision(3) << '(' << p.x << ',' << p.y << ')';
    os.flags(flags);
    os.precision(prec);
    return os;
}

inline std::string NavBasename(const std::string& path) {
    const auto pos = path.find_last_of("/\\");
    return pos == std::string::npos ? path : path.substr(pos + 1);
}

}  // namespace common
}  // namespace autonomy
