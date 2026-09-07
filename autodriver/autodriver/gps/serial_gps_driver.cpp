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

#include "autodriver/gps/serial_gps_driver.hpp"

#include <utility>

#include "autolink/time/time.hpp"
#include "autodriver/types/sensor_sample.hpp"

namespace autodriver {
namespace hardware {

SerialGpsDriver::SerialGpsDriver(SensorId id, DriverParams params)
: id_(std::move(id)),
  params_(std::move(params))
{
}

SerialGpsDriver::~SerialGpsDriver()
{
  Stop();
}

bool SerialGpsDriver::Start()
{
  if (running_.exchange(true)) {
    return true;
  }

  parser_ = gps::GnssParserRegistry::Instance().Create("nmea");
  if (!parser_) {
    running_ = false;
    return false;
  }

  const std::string device = GetString(params_, "device", "/dev/ttyUSB0");
  const int baud = ParseInt(params_, "baud", 115200);
  stream_ = common::CreateSerialStream(device, baud);
  if (!stream_ || !stream_->Connect()) {
    stream_.reset();
    parser_.reset();
    running_ = false;
    return false;
  }

  worker_ = std::thread([this]() { ReadLoop(); });
  return true;
}

void SerialGpsDriver::Stop()
{
  if (!running_.exchange(false)) {
    return;
  }
  if (stream_) {
    stream_->Disconnect();
  }
  if (worker_.joinable()) {
    worker_.join();
  }
  stream_.reset();
  parser_.reset();
}

bool SerialGpsDriver::IsRunning() const
{
  return running_.load();
}

void SerialGpsDriver::SetSampleCallback(SampleCallback callback)
{
  callback_ = std::move(callback);
}

void SerialGpsDriver::ReadLoop()
{
  std::uint8_t chunk[256];
  while (running_.load()) {
    if (!stream_ || !parser_) {
      break;
    }
    if (stream_->status() == common::Stream::Status::kError) {
      if (!common::ReconnectStream(stream_.get(), 3, 200)) {
        break;
      }
    }
    const std::size_t n = stream_->Read(chunk, sizeof(chunk), 100);
    if (n == 0) {
      continue;
    }

    auto fix = parser_->Consume(chunk, n);
    while (fix) {
      if (callback_) {
        callback_(std::make_unique<GpsSample>(
            id_, autolink::Time::Now(),
            GpsMsg(fix->latitude_deg, fix->longitude_deg, fix->altitude_m,
                   fix->status)));
      }
      fix = parser_->Consume(nullptr, 0);
    }
  }
}

std::shared_ptr<SensorDriver> CreateSerialGpsDriver(
  const SensorId & id,
  const DriverParams & params)
{
  return std::make_shared<SerialGpsDriver>(id, params);
}

}  // namespace hardware
}  // namespace autodriver
