#include "autonomy/autoviz/core/converter/type_name_converter.hpp"

namespace autoviz {
namespace converter {

std::string ToFoxgloveTypeName(const std::string& automsgs_full_name) {
  // Very small heuristic for now: map
  //   automsgs.msgs.<pkg>.<Msg>
  // to
  //   <pkg>/<Msg>
  //
  // This can be extended or overridden by a lookup table later.
  constexpr char kPrefix[] = "automsgs.msgs.";
  if (automsgs_full_name.rfind(kPrefix, 0) != 0) {
    return automsgs_full_name;
  }

  const std::string_view sv(automsgs_full_name);
  const std::string_view without_prefix = sv.substr(sizeof(kPrefix) - 1);
  const auto last_dot = without_prefix.rfind('.');
  if (last_dot == std::string_view::npos) {
    return std::string(without_prefix);
  }

  const std::string_view pkg = without_prefix.substr(0, last_dot);
  const std::string_view name = without_prefix.substr(last_dot + 1);
  return std::string(pkg) + "/" + std::string(name);
}

}  // namespace converter
}  // namespace autoviz

