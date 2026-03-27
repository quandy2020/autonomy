#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry.hpp"
#include "autonomy/autoviz/core/convert/registry/automsgs_foxglove_registry_data.hpp"

#include <algorithm>
#include <vector>

namespace autoviz {
namespace converter {
namespace {

const std::vector<AutomsgsFoxgloveEntry>& RegistryVector() {
  static const std::vector<AutomsgsFoxgloveEntry> kVec = [] {
    std::vector<AutomsgsFoxgloveEntry> v;
    v.reserve(foxglove_registry_data::kRows.size());
    for (const auto& row : foxglove_registry_data::kRows) {
      v.push_back(AutomsgsFoxgloveEntry{
          std::string(row.proto_full_name),
          std::string(row.foxglove_canonical_name),
          row.strategy,
          std::string(row.studio_hint),
      });
    }
    return v;
  }();
  return kVec;
}

}  // namespace

const AutomsgsFoxgloveEntry* FindFoxgloveRule(const std::string& proto_full_name) {
  const auto& vec = RegistryVector();
  auto it = std::lower_bound(vec.begin(), vec.end(), proto_full_name,
                             [](const AutomsgsFoxgloveEntry& e, const std::string& key) {
                               return e.proto_full_name < key;
                             });
  if (it != vec.end() && it->proto_full_name == proto_full_name) {
    return &*it;
  }
  return nullptr;
}

}  // namespace converter
}  // namespace autoviz
