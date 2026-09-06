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

#include "autodriver/gps/parser/parser.hpp"

#include <mutex>

namespace autodriver {
namespace gps {

GnssParserRegistry& GnssParserRegistry::Instance() {
    static GnssParserRegistry instance;
    static std::once_flag once;
    std::call_once(once, [&]() {
        instance.factories_["nmea"] = []() {
            return std::make_unique<Nmea0183Parser>();
        };
        instance.factories_["nmea0183"] = []() {
            return std::make_unique<Nmea0183Parser>();
        };
    });
    return instance;
}

void GnssParserRegistry::Register(const std::string& name,
                                  GnssParserFactory factory) {
    std::lock_guard<std::mutex> lock(mutex_);
    factories_[name] = std::move(factory);
}

std::unique_ptr<GnssParser> GnssParserRegistry::Create(
    const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = factories_.find(name);
    if (it == factories_.end()) {
        return nullptr;
    }
    return it->second();
}

bool GnssParserRegistry::Has(const std::string& name) const {
    std::lock_guard<std::mutex> lock(mutex_);
    return factories_.count(name) > 0;
}

}  // namespace gps
}  // namespace autodriver
