/*
 * Copyright 2026 Autodriver contributors duyongquan (quandy2020@126.com)
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

#include "examples/legged_robot/reference_manager/switched_model_reference_manager.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <iostream>

#include <boost/filesystem.hpp>

#include <bolza/core/misc/load_data.hpp>

namespace bolza {
namespace legged_robot {
namespace {

ModeSequenceTemplate TrotGait() {
  // Diagonal pairs, 0.35 s each. Stance feet keep zero velocity on the ground.
  return ModeSequenceTemplate({0.0, 0.35, 0.70}, {ModeNumber::LF_RH, ModeNumber::RF_LH});
}

ModeSequenceTemplate StanceGait() {
  return ModeSequenceTemplate({0.0, 0.5}, {ModeNumber::STANCE});
}

std::string lowerCopy(std::string text) {
  for (char& letter : text) {
    letter = static_cast<char>(std::tolower(static_cast<unsigned char>(letter)));
  }
  const auto begin = std::find_if_not(text.begin(), text.end(), [](unsigned char letter) { return std::isspace(letter); });
  const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char letter) { return std::isspace(letter); }).base();
  if (begin >= end) {
    return {};
  }
  return std::string(begin, end);
}

vector_t basePoseAt(const TargetTrajectories& command, scalar_t time) {
  vector_t pose = vector_t::Zero(6);
  if (command.empty() || command.stateTrajectory.front().size() < 12) {
    pose(2) = 0.575;
    return pose;
  }
  const scalar_t t0 = command.timeTrajectory.front();
  const scalar_t t1 = command.timeTrajectory.back();
  const scalar_t query = std::min(std::max(time, t0), t1);
  const vector_t state = command.GetDesiredState(query);
  if (state.size() >= 12) {
    pose = state.segment<6>(6);
  }
  return pose;
}

Eigen::Vector2d footXy(const vector_t& base, const vector3_t& offset) {
  const double yaw = base(3);
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  return {base(0) + c * offset.x() - s * offset.y(), base(1) + s * offset.x() + c * offset.y()};
}

std::pair<int, int> swingEventIndex(std::size_t index, const std::vector<bool>& contact) {
  if (index >= contact.size() || contact[index]) {
    return {0, 0};
  }
  int start = -1;
  for (int phase = static_cast<int>(index) - 1; phase >= 0; --phase) {
    if (contact[static_cast<std::size_t>(phase)]) {
      start = phase;
      break;
    }
  }
  int final_index = static_cast<int>(contact.size()) - 1;
  for (std::size_t phase = index + 1; phase < contact.size(); ++phase) {
    if (contact[phase]) {
      final_index = static_cast<int>(phase) - 1;
      break;
    }
  }
  return {start, final_index};
}

TargetTrajectories adaptBase(const TargetTrajectories& command, const Terrain& terrain) {
  if (command.empty() || command.stateTrajectory.front().size() < 12) {
    return command;
  }
  const double com_height = terrain.comHeight();
  const double pitch_limit = std::max(0.05, terrain.maxSlope());
  auto place = [&](vector_t state) {
    const double x = state(6);
    const double y = state(7);
    const vector3_t normal = terrain.normal(x, y);
    state(8) = terrain.smoothHeight(x, y) + com_height;
    const double pitch = std::atan2(-normal.x(), std::max(0.2, normal.z()));
    const double roll = std::atan2(normal.y(), std::max(0.2, normal.z()));
    state(10) = std::max(-pitch_limit, std::min(pitch_limit, pitch));
    state(11) = std::max(-pitch_limit, std::min(pitch_limit, roll));
    return state;
  };
  const scalar_t t0 = command.timeTrajectory.front();
  const scalar_t t1 = command.timeTrajectory.back();
  const vector_t input0 = command.inputTrajectory.empty() ? vector_t() : command.inputTrajectory.front();
  if (!(t1 > t0 + 1e-4)) {
    TargetTrajectories out = command;
    for (auto& state : out.stateTrajectory) {
      if (state.size() >= 12) {
        state = place(std::move(state));
      }
    }
    return out;
  }
  const int count = std::min(100, std::max(2, static_cast<int>(std::ceil((t1 - t0) / 0.1)) + 1));
  scalar_array_t times;
  vector_array_t states;
  vector_array_t inputs;
  times.reserve(static_cast<std::size_t>(count));
  states.reserve(static_cast<std::size_t>(count));
  for (int i = 0; i < count; ++i) {
    const scalar_t time = t0 + (t1 - t0) * static_cast<scalar_t>(i) / static_cast<scalar_t>(count - 1);
    times.push_back(time);
    states.push_back(place(command.GetDesiredState(time)));
    if (input0.size() > 0) {
      inputs.push_back(input0);
    }
  }
  return TargetTrajectories(std::move(times), std::move(states), std::move(inputs));
}

}  // namespace

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
SwitchedModelReferenceManager::SwitchedModelReferenceManager(std::shared_ptr<GaitSchedule> gaitSchedulePtr,
                                                             std::shared_ptr<SwingTrajectoryPlanner> swingTrajectoryPtr)
    : ReferenceManager(TargetTrajectories(), ModeSchedule()),
      gaitSchedulePtr_(std::move(gaitSchedulePtr)),
      swingTrajectoryPtr_(std::move(swingTrajectoryPtr)) {
  gaits_.emplace("stance", StanceGait());
  gaits_.emplace("trot", TrotGait());
  foot_offsets_[0] = vector3_t(0.30, 0.20, 0.0);
  foot_offsets_[1] = vector3_t(0.30, -0.20, 0.0);
  foot_offsets_[2] = vector3_t(-0.30, 0.20, 0.0);
  foot_offsets_[3] = vector3_t(-0.30, -0.20, 0.0);
  surface_normals_.fill(vector3_t::UnitZ());
}

/******************************************************************************************************/
/******************************************************************************************************/
/******************************************************************************************************/
void SwitchedModelReferenceManager::SetModeSchedule(const ModeSchedule& modeSchedule) {
  ReferenceManager::SetModeSchedule(modeSchedule);
  gaitSchedulePtr_->SetModeSchedule(modeSchedule);
}

void SwitchedModelReferenceManager::RememberCommand(const TargetTrajectories& targetTrajectories) {
  std::lock_guard<std::mutex> lock(mutex_);
  command_ = targetTrajectories;
  // A new Nav Goal returns gait selection to distance-based switching.
  hold_gait_ = false;
  force_stance_ = false;
}

void SwitchedModelReferenceManager::SetTargetTrajectories(const TargetTrajectories& targetTrajectories) {
  RememberCommand(targetTrajectories);
  ReferenceManager::SetTargetTrajectories(targetTrajectories);
}

void SwitchedModelReferenceManager::SetTargetTrajectories(TargetTrajectories&& targetTrajectories) {
  RememberCommand(targetTrajectories);
  ReferenceManager::SetTargetTrajectories(std::move(targetTrajectories));
}

contact_flag_t SwitchedModelReferenceManager::GetContactFlags(scalar_t time) const {
  return ModeNumber2StanceLeg(this->GetModeSchedule().ModeAtTime(time));
}

void SwitchedModelReferenceManager::LoadGaits(const std::string& gait_file) {
  std::vector<std::string> names;
  try {
    loadData::LoadStdVector(gait_file, "list", names, false);
  } catch (const std::exception& ex) {
    std::cerr << "[SwitchedModelReferenceManager] " << ex.what() << std::endl;
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (const auto& name : names) {
    try {
      gaits_.insert_or_assign(lowerCopy(name), LoadModeSequenceTemplate(gait_file, name, false));
    } catch (const std::exception& ex) {
      std::cerr << "[SwitchedModelReferenceManager] skip gait " << name << ": " << ex.what() << std::endl;
    }
  }
  if (gaits_.find("trot") == gaits_.end()) {
    gaits_.emplace("trot", TrotGait());
  }
  if (gaits_.find("stance") == gaits_.end()) {
    gaits_.emplace("stance", StanceGait());
  }
  std::cerr << "[SwitchedModelReferenceManager] gaits:";
  for (const auto& gait : gaits_) {
    std::cerr << " " << gait.first;
  }
  std::cerr << std::endl;
}

void SwitchedModelReferenceManager::SetGait(const std::string& name) {
  const std::string key = lowerCopy(name);
  std::lock_guard<std::mutex> lock(mutex_);
  if (key.empty()) {
    return;
  }
  if (key == "stance") {
    hold_gait_ = false;
    force_stance_ = true;
    gait_dirty_ = true;
    std::cerr << "[SwitchedModelReferenceManager] gait stance" << std::endl;
    return;
  }
  if (gaits_.find(key) == gaits_.end()) {
    std::cerr << "[SwitchedModelReferenceManager] unknown gait '" << key << "'. known:";
    for (const auto& gait : gaits_) {
      std::cerr << " " << gait.first;
    }
    std::cerr << std::endl;
    return;
  }
  locomotion_ = key;
  hold_gait_ = true;
  force_stance_ = false;
  gait_dirty_ = true;
  std::cerr << "[SwitchedModelReferenceManager] gait " << key << std::endl;
}

std::string SwitchedModelReferenceManager::GaitList() const {
  std::lock_guard<std::mutex> lock(mutex_);
  std::string list;
  for (const auto& gait : gaits_) {
    if (!list.empty()) {
      list.push_back(' ');
    }
    list += gait.first;
  }
  return list;
}

void SwitchedModelReferenceManager::LoadTerrain(const std::string& terrain_file) {
  Terrain loaded = Terrain::load(terrain_file);
  std::lock_guard<std::mutex> lock(mutex_);
  terrain_file_ = terrain_file;
  terrain_ = std::move(loaded);
  std::cerr << "[SwitchedModelReferenceManager] terrain " << terrain_.description() << std::endl;
}

void SwitchedModelReferenceManager::SetTerrainCommand(const std::string& command) {
  const std::string key = lowerCopy(command);
  Terrain loaded = Terrain::flat();
  if (key == "stairs") {
    std::string file;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      file = terrain_file_;
    }
    loaded = file.empty() ? Terrain::flat() : Terrain::load(file);
  } else if (key != "flat" && !key.empty() && boost::filesystem::exists(command)) {
    loaded = Terrain::loadHeightFile(command);
  } else if (key != "flat") {
    std::cerr << "[SwitchedModelReferenceManager] terrain command is flat, stairs, or a height-file path" << std::endl;
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  terrain_ = std::move(loaded);
  std::cerr << "[SwitchedModelReferenceManager] terrain " << terrain_.description() << std::endl;
}

Terrain SwitchedModelReferenceManager::TerrainCopy() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return terrain_;
}

void SwitchedModelReferenceManager::SetFootOffsets(const feet_array_t<vector3_t>& offsets) {
  std::lock_guard<std::mutex> lock(mutex_);
  foot_offsets_ = offsets;
}

vector3_t SwitchedModelReferenceManager::SurfaceNormal(size_t foot) const {
  if (foot >= surface_normals_.size()) {
    return vector3_t::UnitZ();
  }
  return surface_normals_[foot];
}

void SwitchedModelReferenceManager::ModifyReferences(scalar_t initTime, scalar_t finalTime, const vector_t& initState,
                                                     TargetTrajectories& targetTrajectories, ModeSchedule& modeSchedule) {
  const auto timeHorizon = finalTime - initTime;
  TargetTrajectories command;
  Terrain terrain;
  feet_array_t<vector3_t> offsets;
  ModeSequenceTemplate walk = TrotGait();
  bool switch_gait = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (command_.empty()) {
      command_ = targetTrajectories;
    }
    command = command_;
    terrain = terrain_;
    offsets = foot_offsets_;
    const auto walk_it = gaits_.find(locomotion_);
    if (walk_it != gaits_.end()) {
      walk = walk_it->second;
    }
    if (initState.size() >= 12 && !command.empty()) {
      const vector_t goal = command.GetDesiredState(finalTime);
      if (goal.size() >= 12) {
        const double dx = goal(6) - initState(6);
        const double dy = goal(7) - initState(7);
        const double dyaw = std::atan2(std::sin(goal(9) - initState(9)), std::cos(goal(9) - initState(9)));
        const double distance = std::sqrt(dx * dx + dy * dy);
        const bool moving = walking_ ? (distance > 0.05 || std::abs(dyaw) > 0.08)
                                     : (distance > 0.12 || std::abs(dyaw) > 0.2);
        const bool want_walk = !force_stance_ && (hold_gait_ || moving);
        if (want_walk != walking_ || gait_dirty_) {
          walking_ = want_walk;
          switch_gait = true;
          gait_dirty_ = false;
          std::cerr << "[SwitchedModelReferenceManager] gait " << (walking_ ? locomotion_ : "stance") << std::endl;
        }
      }
    }
  }

  modeSchedule = gaitSchedulePtr_->GetModeSchedule(initTime - timeHorizon, finalTime + timeHorizon);

  feet_array_t<scalar_array_t> lift_off;
  feet_array_t<scalar_array_t> touch_down;
  const std::size_t phases = modeSchedule.modeSequence.size();
  const auto& events = modeSchedule.eventTimes;
  for (std::size_t foot = 0; foot < 4; ++foot) {
    std::vector<bool> contact(phases, true);
    for (std::size_t phase = 0; phase < phases; ++phase) {
      contact[phase] = ModeNumber2StanceLeg(modeSchedule.modeSequence[phase])[foot];
    }
    lift_off[foot].assign(phases, 0.0);
    touch_down[foot].assign(phases, 0.0);
    for (std::size_t phase = 0; phase < phases; ++phase) {
      if (contact[phase]) {
        scalar_t time = initTime;
        if (phase < events.size()) {
          time = events[phase];
        }
        const Eigen::Vector2d xy = footXy(basePoseAt(command, time), offsets[foot]);
        const double height = terrain.height(xy.x(), xy.y());
        lift_off[foot][phase] = height;
        touch_down[foot][phase] = height;
        continue;
      }
      const auto index = swingEventIndex(phase, contact);
      const int start = index.first;
      const int final_index = index.second;
      scalar_t lift_time = initTime;
      scalar_t land_time = finalTime;
      if (start >= 0 && static_cast<std::size_t>(start) < events.size()) {
        lift_time = events[static_cast<std::size_t>(start)];
      }
      if (final_index >= 0 && static_cast<std::size_t>(final_index) < events.size()) {
        land_time = events[static_cast<std::size_t>(final_index)];
      }
      const Eigen::Vector2d lift_xy = footXy(basePoseAt(command, lift_time), offsets[foot]);
      const Eigen::Vector2d land_xy = footXy(basePoseAt(command, land_time), offsets[foot]);
      lift_off[foot][phase] = terrain.height(lift_xy.x(), lift_xy.y());
      touch_down[foot][phase] = terrain.height(land_xy.x(), land_xy.y());
    }
  }
  swingTrajectoryPtr_->Update(modeSchedule, lift_off, touch_down);

  const vector_t base_now = basePoseAt(command, initTime);
  for (std::size_t foot = 0; foot < 4; ++foot) {
    const Eigen::Vector2d xy = footXy(base_now, offsets[foot]);
    surface_normals_[foot] = terrain.normal(xy.x(), xy.y());
  }

  // GaitReceiver inserts the template at finalTime, after this horizon has been
  // built. Replacing the modes inside [initTime, finalTime] makes trajectory
  // spreading walk off the mode array (bad_alloc / process abort).
  if (switch_gait) {
    gaitSchedulePtr_->InsertModeSequenceTemplate(walking_ ? walk : StanceGait(), finalTime, timeHorizon);
  }

  targetTrajectories = adaptBase(command, terrain);
}

}  // namespace legged_robot
}  // namespace bolza
