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

#ifndef AVIZ_COMMON__PANEL_HPP_
#define AVIZ_COMMON__PANEL_HPP_

#include <QWidget>  // NOLINT: cpplint is unable to handle the include order here

#include "autonomy/tools/aviz/common/config.hpp"

namespace aviz {
namespace common {

class DisplayContext;

/// Base class for panels
/**
 * Panels are dockable widgets that provide additional UI elements.
 * Examples include property panels, tool panels, etc.
 */
class Panel : public QWidget {
  Q_OBJECT

 public:
  explicit Panel(QWidget* parent = nullptr);
  ~Panel() override;

  /// Initialize the panel with display context.
  void initialize(DisplayContext* context);

  /// Load panel configuration from Config.
  virtual void load(const Config& config);

  /// Save panel configuration to Config.
  virtual void save(Config config) const;

  /// Get the panel name.
  QString getName() const;

  /// Set the panel name.
  void setName(const QString& name);

  /// Get the class identifier.
  QString getClassId() const;

  /// Set the class identifier.
  void setClassId(const QString& class_id);

 protected:
  /// Override to do subclass-specific initialization.
  /**
   * Called by Panel::initialize() after context_ is set.
   */
  virtual void onInitialize() {}

  DisplayContext* context_;

 private:
  QString name_;
  QString class_id_;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__PANEL_HPP_
