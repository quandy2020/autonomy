/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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
 * @file parser.hpp
 * @brief GNSS parser factory.
 */

#ifndef AUTODRIVER_GPS_PARSER_PARSER_HPP_
#define AUTODRIVER_GPS_PARSER_PARSER_HPP_

#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "autodriver/gps/nmea_0183.hpp"
#include "autodriver/common/named_factory.hpp"
#include "autolink/common/macros.hpp"

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
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(GnssParser)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(GnssParser)

    /**
     * @brief Default constructor (DISALLOW_COPY suppresses the implicit one).
     */
    GnssParser() = default;

    /**
     * @brief Virtual destructor for polymorphic parser deletion.
     */
    virtual ~GnssParser() = default;

    /**
     * @brief Append @p size bytes and try to decode a complete message.
     * @param[in] data Pointer to received bytes; may be null only if size == 0.
     * @param[in] size Number of bytes.
     * @return A fix when a full sentence/message was completed; else nullopt.
     */
    virtual std::optional<ParsedFix> Consume(const std::uint8_t* data,
                                             std::size_t size) = 0;
};

/**
 * @brief Creator for NamedProductFactory0: returns owning GnssParser*.
 * @return New parser, or nullptr on failure.
 */
using GnssParserFactory = NamedProductFactory0<GnssParser>::Creator;

/**
 * @class autodriver::gps::GnssParserRegistry
 * @brief Name → GnssParserFactory map ("nmea", "nmea0183", future vendors).
 *
 * Internally uses NamedProductFactory0 (autolink::common::Factory).
 */
class GnssParserRegistry {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(GnssParserRegistry)

    /**
     * @brief Disable copy construction and copy assignment.
     */
    DISALLOW_COPY_AND_ASSIGN(GnssParserRegistry)
    /**
     * @brief Default constructor (DISALLOW_COPY suppresses the implicit one).
     */
    GnssParserRegistry() = default;

    /**
     * @brief Process-wide singleton; registers built-in NMEA on first use.
     * @return Reference to the process-local registry instance.
     */
    static GnssParserRegistry& Instance();

    /**
     * @brief Register or replace a factory under @p name.
     * @param[in] name Parser id (e.g. "nmea").
     * @param[in] factory Creator returning new GnssParser*.
     */
    void RegisterParser(const std::string& name, GnssParserFactory factory);

    /**
     * @brief Construct a parser by name.
     * @param[in] name Parser id.
     * @return Owning unique_ptr, or nullptr when @p name is unknown.
     */
    std::unique_ptr<GnssParser> CreateParser(const std::string& name) const;

    /**
     * @brief Whether @p name is registered.
     * @param[in] name Parser id.
     * @return true when a creator is available.
     */
    bool HasParser(const std::string& name) const;

private:
  NamedProductFactory0<GnssParser> factory_;
};

/**
 * @class autodriver::gps::Nmea0183Parser
 * @brief Line-buffered NMEA 0183 parser wrapping ParseGgaSentence/ParseRmcSentence.
 */
class Nmea0183Parser : public GnssParser {
public:
  /**
   * @brief SharedPtr / ConstSharedPtr aliases and Class::make_shared().
   */
  AUTOLINK_SHARED_PTR_DEFINITIONS(Nmea0183Parser)

  /**
   * @brief Disable copy construction and copy assignment.
   */
  DISALLOW_COPY_AND_ASSIGN(Nmea0183Parser)

  /**
   * @brief Default constructor (DISALLOW_COPY suppresses the implicit one).
   */
  Nmea0183Parser() = default;

  /**
   * @brief Append bytes (optional) then return the next complete fix.
   * Pass size==0 to drain remaining complete lines after a read.
   * @param[in] data Pointer to received bytes; may be null only if size == 0.
   * @param[in] size Number of bytes to append.
   * @return A fix when a full GGA/RMC line was completed; else nullopt.
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
