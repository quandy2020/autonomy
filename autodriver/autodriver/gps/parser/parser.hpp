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
 * @brief GNSS parser factory.
 */

#ifndef AUTODRIVER_GPS_PARSER_PARSER_HPP_
#define AUTODRIVER_GPS_PARSER_PARSER_HPP_

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include "autodriver/gps/nmea_0183.hpp"

namespace autodriver {
namespace gps {

/**
 * @brief One parsed GNSS fix (same fields as NMEA GGA/RMC helpers).
 */
using ParsedFix = protocol::NmeaGgaFix;

/**
 * @class autodriver::gps::GnssParser
 * @brief Vendor-agnostic streaming parser: feed bytes, optionally emit a fix.
 *
 * Drivers keep a parser instance and call Consume() on each Stream read.
 */
class GnssParser {
public:
    virtual ~GnssParser() = default;

    /**
     * @brief Append @p size bytes and try to decode a complete message.
     * @param data Pointer to received bytes; may be null only if size == 0.
     * @param size Number of bytes.
     * @return A fix when a full sentence/message was completed; else nullopt.
     */
    virtual std::optional<ParsedFix> Consume(const std::uint8_t* data,
                                             std::size_t size) = 0;
};

// Factory that constructs a concrete GnssParser.
using GnssParserFactory = std::function<std::unique_ptr<GnssParser>()>;

/**
 * @class autodriver::gps::GnssParserRegistry
 * @brief Name → GnssParserFactory map ("nmea", "nmea0183", future vendors).
 */
class GnssParserRegistry {
public:
    /**
     * @brief Process-wide singleton; registers built-in NMEA on first use.
     */
    static GnssParserRegistry& Instance();

    /**
     * @brief Register or replace a factory under @p name.
     */
    void Register(const std::string& name, GnssParserFactory factory);

    /**
     * @brief Construct a parser by name.
     * @return nullptr when @p name is unknown.
     */
    std::unique_ptr<GnssParser> Create(const std::string& name) const;

    /**
     * @brief Whether @p name is registered.
     */
    bool Has(const std::string& name) const;

private:
    // Guards factories_.
    mutable std::mutex mutex_;
    // parser name → factory.
    std::unordered_map<std::string, GnssParserFactory> factories_;
};

/**
 * @class autodriver::gps::Nmea0183Parser
 * @brief Line-buffered NMEA 0183 parser wrapping ParseGgaSentence/ParseRmcSentence.
 */
class Nmea0183Parser : public GnssParser {
public:
    /**
     * @brief Append bytes (optional) then return the next complete fix.
     * Pass size==0 to drain remaining complete lines after a read.
     */
    std::optional<ParsedFix> Consume(const std::uint8_t* data,
                                     std::size_t size) override {
        if (data != nullptr && size > 0) {
            buffer_.append(reinterpret_cast<const char*>(data), size);
        }
        while (true) {
            const auto pos = buffer_.find('\n');
            if (pos == std::string::npos) {
                return std::nullopt;
            }
            std::string line = buffer_.substr(0, pos);
            buffer_.erase(0, pos + 1);
            if (!line.empty() && line.back() == '\r') {
                line.pop_back();
            }
            if (line.find("GGA") != std::string::npos) {
                if (auto fix = protocol::ParseGgaSentence(line)) {
                    return fix;
                }
            } else if (line.find("RMC") != std::string::npos) {
                if (auto fix = protocol::ParseRmcSentence(line)) {
                    return fix;
                }
            }
        }
    }

private:
    // Incomplete trailing line across Consume() calls.
    std::string buffer_;
};

}  // namespace gps
}  // namespace autodriver

#endif  // AUTODRIVER_GPS_PARSER_PARSER_HPP_
