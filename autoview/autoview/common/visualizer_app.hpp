/******************************************************************************
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#pragma once

#include <QObject>

class QTimer;

namespace Ogre
{
class Light;
class Root;
}

namespace autoview {
namespace common {

namespace properties
{

class ColorProperty;
class IntProperty;
class Property;
class PropertyTreeModel;
class StatusList;
class TfFrameProperty;

}  // namespace properties

class Display;
class Tool;
class OgreRenderQueueClearer;

class VisualizationManagerPrivate;

class VisualizerApp : public QObject
{
  Q_OBJECT
public:
  explicit VisualizerApp(QObject* parent = nullptr) : QObject(parent) {}
  ~VisualizerApp() override = default;
};

}  // namespace common
}  // namespace autoview