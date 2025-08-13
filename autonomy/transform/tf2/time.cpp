#include "autonomy/transform/tf2/time.h"

namespace autonomy {
namespace transform {
namespace tf2 {
  double time_to_sec(Time t) { return static_cast<double>(t) / 1e9; }
}  // namespace tf2
}  // namespace transform
}  // namespace autonomy