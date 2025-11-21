/**
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>
#include <cstring>
#include <string>

namespace autolink {
namespace transport {

constexpr uint8_t ID_SIZE = 8;

class Identity
{
public:
    /**
     * @brief Constructor
     * @param need_generate True if the identity should be generated, false
     * otherwise
     */
    explicit Identity(bool need_generate = true);

    /**
     * @brief Copy constructor
     * @param another The identity to copy
     */
    Identity(const Identity& another);

    /**
     * @brief Destructor
     */
    virtual ~Identity();

    /**
     * @brief Assign the identity to another identity
     * @param another The identity to assign
     * @return Identity& The assigned identity
     */
    Identity& operator=(const Identity& another);

    /**
     * @brief Check if the identity is equal to another identity
     * @param another The identity to check
     * @return bool True if the identity is equal to another identity, false
     * otherwise
     */
    bool operator==(const Identity& another) const;

    /**
     * @brief Check if the identity is not equal to another identity
     * @param another The identity to check
     * @return bool True if the identity is not equal to another identity, false
     * otherwise
     */
    bool operator!=(const Identity& another) const;

    /**
     * @brief Convert the identity to a string
     * @return std::string The string representation of the identity
     */
    std::string ToString() const;

    /**
     * @brief Get the length of the identity
     * @return size_t The length of the identity
     */
    size_t Length() const;

    /**
     * @brief Get the hash value of the identity
     * @return uint64_t The hash value of the identity
     */
    uint64_t HashValue() const;

    /**
     * @brief Get the data of the identity
     * @return const char* The data of the identity
     */
    const char* data() const {
        return data_;
    }

    /**
     * @brief Set the data of the identity
     * @param data The data to set
     */
    void set_data(const char* data) {
        if (data == nullptr) {
            return;
        }
        std::memcpy(data_, data, sizeof(data_));
        Update();
    }

private:
    void Update();
    char data_[ID_SIZE];
    uint64_t hash_value_;
};
}  // namespace transport
}  // namespace autolink