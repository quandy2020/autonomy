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

#include <QSplitter>

#include "autoview/common/config.hpp"

class QTextBrowser;

namespace autoview {
namespace common {
namespace properties {

class Property;
class PropertyTreeWidget;

/** A PropertyTreeWidget with built-in help info display. */
class PropertyTreeWithHelp : public QSplitter
{
  Q_OBJECT

public:
  explicit PropertyTreeWithHelp(QWidget * parent = 0);

  PropertyTreeWidget * getTree() {return property_tree_;}

  /** @brief Write state to the given Config. */
  void save(Config config) const;

  /** @brief Read state from the given Config. */
  void load(const Config & config);

private Q_SLOTS:
  void showHelpForProperty(const Property * property);

private:
  PropertyTreeWidget * property_tree_;
  QTextBrowser * help_;
};

}  // namespace properties
}  // namespace common
}  // namespace autoview