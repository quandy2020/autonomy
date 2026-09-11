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
 * @file parser.cpp
 * @brief GNSS parser factory (implementation).
 */

#include "autodriver/gps/parser/parser.hpp"

#include <mutex>

namespace autodriver {
namespace gps {

GnssParserRegistry& GnssParserRegistry::Instance() {
  static GnssParserRegistry instance;
  static std::once_flag once;
  std::call_once(once, [&]() {
    instance.RegisterParser("nmea", []() -> GnssParser* {
      return new Nmea0183Parser();
    });
    instance.RegisterParser("nmea0183", []() -> GnssParser* {
      return new Nmea0183Parser();
    });
  });
  return instance;
}

void GnssParserRegistry::RegisterParser(const std::string& name,
                                        GnssParserFactory factory) {
  factory_.Register(name, std::move(factory));
}

std::unique_ptr<GnssParser> GnssParserRegistry::CreateParser(
    const std::string& name) const {
  return factory_.CreateUnique(name);
}

bool GnssParserRegistry::HasParser(const std::string& name) const {
  return factory_.Contains(name);
}

}  // namespace gps
}  // namespace autodriver
