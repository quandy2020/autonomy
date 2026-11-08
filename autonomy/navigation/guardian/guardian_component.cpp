/*
 * Copyright 2026 The Navigation Authors (navigation)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "navigation/guardian/guardian_component.hpp"

#include <chrono>
#include <vector>

#include "navigation/common/configuration_loader.hpp"


namespace navigation
{
namespace guardian
{

GuardianComponent::GuardianComponent() = default;

GuardianComponent::~GuardianComponent()
{
    Clear();
}

bool GuardianComponent::Init()
{
    config::GuardianConfig configuration;
    if (!common::LoadTextProto(ConfigFilePath(), &configuration).Ok() ||
        !supervisor_.Configure(configuration))
    {
        return false;
    }
    navigation_reader_ = node_->CreateReader<lifecycle::Heartbeat>("/navigation/heartbeat");
    safety_reader_ = node_->CreateReader<lifecycle::Heartbeat>("/safety/heartbeat");
    behavior_reader_ = node_->CreateReader<lifecycle::Heartbeat>("/behavior/heartbeat");
    degradation_writer_ =
        node_->CreateWriter<lifecycle::Degradation>("/guardian/degradation");
    heartbeat_writer_ = node_->CreateWriter<lifecycle::Heartbeat>("/guardian/heartbeat");
    return navigation_reader_ != nullptr && safety_reader_ != nullptr &&
           behavior_reader_ != nullptr && degradation_writer_ != nullptr &&
           heartbeat_writer_ != nullptr;
}

bool GuardianComponent::Proc()
{
    const std::int64_t now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
        .count();
    std::vector<lifecycle::Heartbeat> heartbeats;
    if (const auto heartbeat = navigation_reader_->GetLatestObserved(); heartbeat != nullptr)
    {
        heartbeats.push_back(*heartbeat);
    }
    if (const auto heartbeat = safety_reader_->GetLatestObserved(); heartbeat != nullptr)
    {
        heartbeats.push_back(*heartbeat);
    }
    if (const auto heartbeat = behavior_reader_->GetLatestObserved(); heartbeat != nullptr)
    {
        heartbeats.push_back(*heartbeat);
    }
    const auto next = supervisor_.Evaluate(heartbeats, now_ns);
    if (next != mode_)
    {
        lifecycle::Degradation degradation;
        degradation.set_from(mode_);
        degradation.set_to(next);
        degradation.set_reason("supervisor");
        degradation.set_stamp_ns(now_ns);
        degradation.set_sequence(++sequence_);
        (void)degradation_writer_->Write(degradation);
        mode_ = next;
    }
    lifecycle::Heartbeat heartbeat;
    heartbeat.set_module("guardian");
    heartbeat.set_sequence(++sequence_);
    heartbeat.set_stamp_ns(now_ns);
    heartbeat.set_state(lifecycle::MODULE_STATE_READY);
    (void)heartbeat_writer_->Write(heartbeat);
    return true;
}

void GuardianComponent::Clear()
{
    navigation_reader_.reset();
    safety_reader_.reset();
    behavior_reader_.reset();
    degradation_writer_.reset();
    heartbeat_writer_.reset();
    mode_ = lifecycle::DEGRADATION_MODE_UNSPECIFIED;
}

}  // namespace guardian
}  // namespace navigation

AUTOLINK_REGISTER_COMPONENT(navigation::guardian::GuardianComponent)
