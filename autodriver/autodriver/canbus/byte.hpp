/*
 * Copyright 2026 Autodriver contributors
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

/**
 * @file
 * @brief Bit helpers for CAN protocol packing.
 */

#ifndef AUTODRIVER_CANBUS_BYTE_HPP_
#define AUTODRIVER_CANBUS_BYTE_HPP_

#include <cstdint>
#include <string>

namespace autodriver {
namespace canbus {

/**
 * @class autodriver::canbus::Byte
 * @brief Mutable view of one payload byte with bit-field helpers.
 *
 * Used when packing/unpacking Conti radar or other multi-signal CAN frames.
 * Bit positions use LSB = 0 within the byte.
 */
class Byte {
public:
    /**
     * @brief Bind to an existing byte in a CAN payload buffer.
     * @param value Pointer to the byte to mutate; may be null (ops no-op).
     */
    explicit Byte(std::uint8_t* value) : value_(value) {}

    /**
     * @brief Overwrite the entire byte.
     * @param v New 8-bit value.
     */
    void set_value(std::uint8_t v) {
        if (value_) {
            *value_ = v;
        }
    }

    /**
     * @brief Read the entire byte.
     * @return Current value, or 0 if unbound.
     */
    std::uint8_t get_byte() const { return value_ ? *value_ : 0; }

    /**
     * @brief Set a single bit to 1.
     * @param pos Bit index in [0, 7].
     */
    void set_bit(int pos) {
        if (value_ && pos >= 0 && pos < 8) {
            *value_ = static_cast<std::uint8_t>(*value_ | (1u << pos));
        }
    }

    /**
     * @brief Clear a single bit to 0.
     * @param pos Bit index in [0, 7].
     */
    void clear_bit(int pos) {
        if (value_ && pos >= 0 && pos < 8) {
            *value_ = static_cast<std::uint8_t>(*value_ & ~(1u << pos));
        }
    }

    /**
     * @brief Test whether a bit is set.
     * @param pos Bit index in [0, 7].
     * @return True when the bit is 1.
     */
    bool is_bit_1(int pos) const {
        return value_ && pos >= 0 && pos < 8 &&
               ((*value_ >> pos) & 0x1u) != 0;
    }

    /**
     * @brief Write @p len bits starting at @p start from the low bits of @p value.
     * @param value Source bits (only the low @p len bits are used).
     * @param start First bit index (LSB = 0).
     * @param len Number of bits to write (must fit in the byte).
     */
    void set_value(std::uint8_t value, int start, int len) {
        if (!value_ || start < 0 || len <= 0 || start + len > 8) {
            return;
        }
        const std::uint8_t mask =
            static_cast<std::uint8_t>(((1u << len) - 1u) << start);
        *value_ = static_cast<std::uint8_t>((*value_ & ~mask) |
                                           ((value << start) & mask));
    }

    /**
     * @brief Extract @p len bits starting at @p start.
     * @param start First bit index (LSB = 0).
     * @param len Number of bits to read.
     * @return Extracted field in the low bits of the return value.
     */
    std::uint8_t get_byte(int start, int len) const {
        if (!value_ || start < 0 || len <= 0 || start + len > 8) {
            return 0;
        }
        return static_cast<std::uint8_t>((*value_ >> start) & ((1u << len) - 1u));
    }

    /**
     * @brief Format one byte as two uppercase hex digits.
     * @param value Byte to format.
     * @return Two-character hex string (e.g. "1F").
     */
    static std::string byte_to_hex(std::uint8_t value) {
        static const char* kHex = "0123456789ABCDEF";
        std::string out(2, '0');
        out[0] = kHex[(value >> 4) & 0xF];
        out[1] = kHex[value & 0xF];
        return out;
    }

private:
    // Bound payload byte; null disables mutations.
    std::uint8_t* value_ = nullptr;
};

}  // namespace canbus
}  // namespace autodriver

#endif  // AUTODRIVER_CANBUS_BYTE_HPP_
