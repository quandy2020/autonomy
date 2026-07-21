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
 * @brief Implements SensorManager.
 */

#include "autodriver/app/sensor_manager.hpp"

#include "autodriver/drivers/driver_registry.hpp"

namespace autodriver {

SensorManager::SensorManager()
: SensorManager(DefaultSimulationHubConfig())
{
}

SensorManager::SensorManager(HubConfig config)
: config_(std::move(config)),
  hub_(config_.hub_options)
{
}

bool SensorManager::Initialize()
{
  if (initialized_) {
    return init_errors_.empty();
  }

  drivers::RegisterDriverFactories(config_.register_builtin_mocks);

  for (const DriverConfig & entry : config_.drivers) {
    if (!CreateDriver(entry)) {
      init_errors_.push_back(entry.factory_name);
    }
  }

  initialized_ = true;
  return init_errors_.empty();
}

bool SensorManager::Start()
{
  if (!initialized_ && !Initialize()) {
    return false;
  }
  return hub_.Start();
}

void SensorManager::Stop()
{
  hub_.Stop();
}

bool SensorManager::IsRunning() const
{
  return hub_.IsRunning();
}

SensorHub & SensorManager::hub()
{
  return hub_;
}

const SensorHub & SensorManager::hub() const
{
  return hub_;
}

const std::vector<std::string> & SensorManager::init_errors() const
{
  return init_errors_;
}

void SensorManager::SetAlignedCallback(SensorHub::AlignedCallback callback)
{
  hub_.SetAlignedCallback(std::move(callback));
}

void SensorManager::SetRawSampleCallback(SensorHub::RawSampleCallback callback)
{
  hub_.SetRawSampleCallback(std::move(callback));
}

bool SensorManager::CreateDriver(const DriverConfig & entry)
{
  std::shared_ptr<SensorDriver> driver =
    drivers::CreateDriver(entry, config_.register_builtin_mocks);
  if (!driver) {
    return false;
  }

  hub_.RegisterDriver(std::move(driver));
  return true;
}

}  // namespace autodriver
