#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <unordered_map>
#include <utility>
#include <variant>

namespace autonomy::localization::atlas {

/** @brief Scalar value accepted by the lightweight configuration map. */
using ConfigValue = std::variant<bool, std::int64_t, double, std::string>;

/** @brief Type-safe string-keyed configuration container. */
class Config
{
public:
    /** @brief Inserts or replaces @p key. */
    void Set(std::string key, ConfigValue value);
    /** @return Whether @p key is present. */
    bool Has(std::string_view key) const;
    /** @return Number of stored keys. */
    std::size_t size() const {
        return values_.size();
    }

    /**
     * @brief Reads a value using an exact type, except integer-to-double
     * widening.
     * @throws std::out_of_range if the key is absent.
     * @throws std::invalid_argument if the stored type is incompatible.
     */
    template <typename T>
    T Get(std::string_view key) const {
        const ConfigValue* value = Find(key);
        if (value == nullptr) {
            throw std::out_of_range("missing configuration key: " +
                                    std::string(key));
        }
        if (const auto* exact = std::get_if<T>(value))
            return *exact;
        if constexpr (std::is_same_v<T, double>) {
            if (const auto* integer = std::get_if<std::int64_t>(value)) {
                return static_cast<double>(*integer);
            }
        }
        throw std::invalid_argument("configuration key has the wrong type: " +
                                    std::string(key));
    }

    /** @brief Reads @p key or returns @p fallback when it is absent. */
    template <typename T>
    T GetOr(std::string_view key, T fallback) const {
        return Has(key) ? Get<T>(key) : std::move(fallback);
    }

private:
    const ConfigValue* Find(std::string_view key) const;

    std::unordered_map<std::string, ConfigValue> values_;
};

}  // namespace autonomy::localization::atlas
