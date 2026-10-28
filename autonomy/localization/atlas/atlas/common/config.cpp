#include "atlas/common/config.hpp"

#include <utility>

namespace autonomy::localization::atlas {

void Config::Set(std::string key, ConfigValue value) {
    if (key.empty())
        throw std::invalid_argument("configuration key is empty");
    values_.insert_or_assign(std::move(key), std::move(value));
}

bool Config::Has(std::string_view key) const {
    return Find(key) != nullptr;
}

const ConfigValue* Config::Find(std::string_view key) const {
    const auto iterator = values_.find(std::string(key));
    return iterator == values_.end() ? nullptr : &iterator->second;
}

}  // namespace autonomy::localization::atlas
