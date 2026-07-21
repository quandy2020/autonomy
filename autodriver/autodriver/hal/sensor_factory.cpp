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
 * @brief Implements SensorFactory.
 */

#include "autodriver/hal/sensor_factory.hpp"

namespace autodriver {

SensorFactory & SensorFactory::Instance()
{
  static SensorFactory instance;
  return instance;
}

bool SensorFactory::Register(const std::string & name, Creator creator)
{
  std::lock_guard<std::mutex> lock(mutex_);
  return creators_.emplace(name, std::move(creator)).second;
}

std::shared_ptr<SensorDriver> SensorFactory::Create(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  const auto it = creators_.find(name);
  if (it == creators_.end()) {
    return nullptr;
  }
  return it->second();
}

bool SensorFactory::Has(const std::string & name) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return creators_.count(name) > 0;
}

}  // namespace autodriver
