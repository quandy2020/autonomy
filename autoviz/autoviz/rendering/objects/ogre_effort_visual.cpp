/******************************************************************************
 * Copyright 2023, Open Source Robotics Foundation, Inc.
 * Adapted for Autoviz (BSD-3-Clause).
 *****************************************************************************/

#include "autoviz/rendering/objects/ogre_effort_visual.hpp"

#ifdef AUTOVIZ_USE_OGRE

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "autoviz/rendering/objects/ogre_arrow.hpp"
#include "autoviz/rendering/objects/ogre_billboard_line.hpp"

namespace autoviz {
namespace rendering {

OgreEffortVisual::OgreEffortVisual(Ogre::SceneManager* scene_manager,
                                   Ogre::SceneNode* parent_node, float width,
                                   float scale)
    : scene_manager_(scene_manager), parent_node_(parent_node), width_(width),
      scale_(scale) {
  if (scene_manager_ == nullptr) {
    throw std::runtime_error("OgreEffortVisual: Scene Manager is null.");
  }
}

void OgreEffortVisual::getRainbowColor(float value, Ogre::ColourValue& color) {
  value = std::min(value, 1.0f);
  value = std::max(value, 0.0f);
  float h = value * 5.0f + 1.0f;
  int i = static_cast<int>(floor(h));
  float f = h - static_cast<float>(i);
  if (!(i & 1)) {
    f = 1 - f;
  }
  float n = 1 - f;
  if (i <= 1) {
    color[0] = n, color[1] = 0, color[2] = 1;
  } else if (i == 2) {
    color[0] = 0, color[1] = n, color[2] = 1;
  } else if (i == 3) {
    color[0] = 0, color[1] = 1, color[2] = n;
  } else if (i == 4) {
    color[0] = n, color[1] = 1, color[2] = 0;
  } else {
    color[0] = 1, color[1] = n, color[2] = 0;
  }
}

void OgreEffortVisual::setEffort(const std::string& joint_name, double effort,
                                 double max_effort) {
  const bool enabled = effort_enabled_.insert({joint_name, true}).first->second;
  if (enabled) {
    if (effort_circle_.count(joint_name) == 0) {
      effort_circle_[joint_name] =
          std::make_unique<OgreBillboardLine>(scene_manager_, parent_node_);
    }
    if (effort_arrow_.count(joint_name) == 0) {
      effort_arrow_[joint_name] = std::make_unique<OgreArrow>(scene_manager_, parent_node_);
    }
    if (position_.count(joint_name) == 0) {
      position_[joint_name] = Ogre::Vector3::ZERO;
    }
    if (orientation_.count(joint_name) == 0) {
      orientation_[joint_name] = Ogre::Quaternion::IDENTITY;
    }
  } else {
    effort_circle_.erase(joint_name);
    effort_arrow_.erase(joint_name);
    return;
  }

  float effort_value;
  if (max_effort != 0.0) {
    effort_value = static_cast<float>(std::fmin(fabs(effort) / max_effort, 1.0f) + 0.05f);
  } else {
    effort_value = static_cast<float>(fabs(effort) + 0.05f);
  }

  effort_arrow_[joint_name]->set(0, width_ * 2.0f, width_ * 2.0f * 1.0f, width_ * 2.0f * 2.0f);
  if (effort > 0) {
    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(-1, 0, 0));
  } else {
    effort_arrow_[joint_name]->setDirection(orientation_[joint_name] * Ogre::Vector3(1, 0, 0));
  }
  effort_arrow_[joint_name]->setPosition(
      orientation_[joint_name] * Ogre::Vector3(0, 0.05f + effort_value * scale_ * 0.5f, 0) +
      position_[joint_name]);
  effort_circle_[joint_name]->clear();
  effort_circle_[joint_name]->setLineWidth(width_);
  for (int i = 0; i < 30; ++i) {
    Ogre::Vector3 point(
        static_cast<float>((0.05f + effort_value * scale_ * 0.5f) * sin(i * 2.0f * M_PI / 32.0f)),
        static_cast<float>((0.05f + effort_value * scale_ * 0.5f) * cos(i * 2.0f * M_PI / 32.0f)),
        0.f);
    if (effort < 0) {
      point.x = -point.x;
    }
    effort_circle_[joint_name]->addPoint(orientation_[joint_name] * point + position_[joint_name]);
  }
  Ogre::ColourValue color;
  getRainbowColor(effort_value, color);
  effort_arrow_[joint_name]->setColor(color.r, color.g, color.b, color.a);
  effort_circle_[joint_name]->setColor(color.r, color.g, color.b, color.a);
}

void OgreEffortVisual::setFrameEnabled(const std::string& joint_name, bool enabled) {
  effort_enabled_[joint_name] = enabled;
}

void OgreEffortVisual::setFramePosition(const std::string& joint_name,
                                        const Ogre::Vector3& position) {
  position_[joint_name] = position;
}

void OgreEffortVisual::setFrameOrientation(const std::string& joint_name,
                                           const Ogre::Quaternion& orientation) {
  orientation_[joint_name] = orientation;
}

void OgreEffortVisual::setWidth(float width) { width_ = width; }
void OgreEffortVisual::setScale(float scale) { scale_ = scale; }

}  // namespace rendering
}  // namespace autoviz

#endif
