#include "atlas/common/time.hpp"

#include <cmath>
#include <cstdint>
#include <ctime>
#include <iomanip>
#include <limits>
#include <sstream>
#include <stdexcept>

namespace autonomy::localization::atlas {

TimePoint TimePointFromSeconds(double seconds) {
    if (!std::isfinite(seconds)) {
        throw std::invalid_argument("timestamp must be finite");
    }
    const long double nanoseconds =
        std::round(static_cast<long double>(seconds) * 1.0e9L);
    if (nanoseconds < static_cast<long double>(
                          std::numeric_limits<std::int64_t>::min()) ||
        nanoseconds > static_cast<long double>(
                          std::numeric_limits<std::int64_t>::max())) {
        throw std::out_of_range("timestamp is outside nanosecond range");
    }
    return TimePoint(Duration(static_cast<std::int64_t>(nanoseconds)));
}

double TimePointToSeconds(TimePoint time) {
    return static_cast<double>(time.time_since_epoch().count()) / 1.0e9;
}

std::string FormatTime(TimePoint time) {
    const auto seconds =
        std::chrono::time_point_cast<std::chrono::seconds>(time);
    const auto subseconds =
        std::chrono::duration_cast<Duration>(time - seconds).count();
    const std::time_t raw = std::chrono::system_clock::to_time_t(seconds);
    std::tm utc{};
#if defined(_WIN32)
    gmtime_s(&utc, &raw);
#else
    gmtime_r(&raw, &utc);
#endif
    std::ostringstream output;
    output << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S") << '.' << std::setw(9)
           << std::setfill('0') << subseconds << 'Z';
    return output.str();
}

}  // namespace autonomy::localization::atlas
