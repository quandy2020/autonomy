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

#ifndef AVIZ_COMMON__PROPERTIES__PROPERTY_HPP_
#define AVIZ_COMMON__PROPERTIES__PROPERTY_HPP_

#include <QIcon>     // NOLINT: cpplint is unable to handle the include order here
#include <QObject>   // NOLINT: cpplint is unable to handle the include order here
#include <QString>   // NOLINT: cpplint is unable to handle the include order here
#include <QVariant>  // NOLINT: cpplint is unable to handle the include order here
#include <string>

#include "autonomy/tools/aviz/common/config.hpp"

class QPainter;
class QStyleOptionViewItem;

namespace aviz {
namespace common {
namespace properties {

class PropertyTreeModel;

/// A single element of a property tree, with a name, value, description, and possibly children.
/**
 * A Property in a property tree is a piece of data editable or at
 * least displayable in a PropertyTreeWidget.
 * A Property object owns the data item in question.
 * When client code needs to be informed about changes, it can connect to the
 * Property's aboutToChange() and changed() signals.
 * A slot receiving the changed() signal should then ask the Property itself
 * for the new data.
 */
class Property : public QObject {
  Q_OBJECT

 public:
  /// Constructor.
  /**
   * All parameters to the constructor are optional and can all be set
   * after construction as well.
   *
   * If a parent is given, this constructor calls `parent->addChild(this)`
   * to add itself to the parent's list of children.
   *
   * If `changed_slot` is given and either `parent` or `receiver` is
   * also, then the changed() signal is connected via QObject::connect() to the
   * slot described by `changed_slot` on the parent or the receiver.
   * If both parent and receiver are specified, receiver is the one which gets
   * connected.
   * If receiver is not specified, parent is used instead.
   */
  explicit Property(const QString& name = QString(), const QVariant default_value = QVariant(),
                    const QString& description = QString(), Property* parent = nullptr,
                    const char* changed_slot = nullptr, QObject* receiver = nullptr);

  /// Destructor which also removes this property from its parent's list of children.
  virtual ~Property();

  /// Remove and delete some or all child Properties.
  virtual void removeChildren(int start_index = 0, int count = -1);

  /// Set the new value for this property.
  /**
   * Returns true if the new value is different from the old value, false if
   * the same.
   *
   * If the new value is different from the old value, this emits
   * aboutToChange() before changing the value and emits changed() after.
   */
  virtual bool setValue(const QVariant& new_value);

  /// Return the value of this Property as a QVariant.
  virtual QVariant getValue() const;

  /// Set the name.
  virtual void setName(const QString& name);

  /// Return the name of this Property as a QString.
  virtual QString getName() const;

  /// Return the name of this Property as a std::string.
  std::string getNameStd() const;

  /// Set the description.
  virtual void setDescription(const QString& description);

  /// Return the description.
  virtual QString getDescription() const;

  /// Set the icon to be displayed next to the property.
  virtual void setIcon(const QIcon& icon);

  /// Return the icon.
  virtual QIcon getIcon() const;

  /// Return the first child Property with the given name if found, else the FailureProperty.
  virtual Property* subProp(const QString& sub_name);

  /// Return the number of child objects (Property or otherwise).
  virtual int numChildren() const;

  /// Return the child Property with the given index, or nullptr.
  Property* childAt(int index) const;

  /// Return the child Property at the given index, without checking the bounds.
  virtual Property* childAtUnchecked(int index) const;

  /// Return true if the list of children includes possible_child, false if not.
  bool contains(Property* possible_child) const;

  /// Return the parent Property.
  Property* getParent() const;

  /// Set parent property, without telling the parent.
  void setParent(Property* new_parent);

  /// Return data appropriate for the given column (0 or 1) and role for this Property.
  virtual QVariant getViewData(int column, int role) const;

  /// Return item flags appropriate for the given column (0 or 1) for this Property.
  virtual Qt::ItemFlags getViewFlags(int column) const;

  /// Hook to provide custom painting of the value data (right-hand column) in a subclass.
  virtual bool paint(QPainter* painter, const QStyleOptionViewItem& option) const;

  /// Create an editor widget to edit the value of this property.
  virtual QWidget* createEditor(QWidget* parent, const QStyleOptionViewItem& option);

  /// Returns true if this Property is an ancestor of possible_child.
  bool isAncestorOf(Property* possible_child) const;

  /// Remove a given child object and return a pointer to it.
  Property* takeChild(Property* child);

  /// Take a child out of the child list, but don't destroy it.
  virtual Property* takeChildAt(int index);

  /// Add a child property.
  virtual void addChild(Property* child, int index = -1);

  /// Set the model managing this Property and all its child properties, recursively.
  void setModel(PropertyTreeModel* model);

  /// Return the model managing this Property and its children.
  PropertyTreeModel* getModel() const;

  /// Return the row number of this property within its parent, or -1 if it has no parent.
  int rowNumberInParent() const;

  /// Move the child at from_index to to_index.
  virtual void moveChild(int from_index, int to_index);

  /// Load the value of this property and/or its children from the given Config reference.
  virtual void load(const Config& config);

  /// Write the value of this property and/or its children into the given Config reference.
  virtual void save(Config config) const;

  /// Returns true if the property is not read-only AND has data worth saving.
  bool shouldBeSaved() const;

  /// If save is true and getReadOnly() is false, shouldBeSaved will return true, otherwise false.
  void setShouldBeSaved(bool save);

  /// If true, the children of this property should set their ItemIsEnabled flag to false.
  virtual bool getDisableChildren();

  /// Hide this Property in any PropertyTreeWidgets.
  void hide();

  /// Show this Property in any PropertyTreeWidgets.
  void show();

  /// Hide or show this property in any PropertyTreeWidget viewing its parent.
  virtual void setHidden(bool hidden);

  /// Return the hidden/shown state.
  virtual bool getHidden() const;

  /// Prevent or allow users to edit this property from a PropertyTreeWidget.
  virtual void setReadOnly(bool read_only);

  /// Return the read-only-ness of this property.
  virtual bool getReadOnly();

  /// Return whether this property is expanded or collapsed.
  virtual bool isExpanded();

  /// Collapse (hide the children of) this Property.
  virtual void collapse();

  /// Expand (show the children of) this Property.
  virtual void expand();

 Q_SIGNALS:
  /// Emitted by setValue() just before the value has changed.
  void aboutToChange();

  /// Emitted by setValue() just after the value has changed.
  void changed();

  /// Emitted after insertions and deletions of child Properties.
  void childListChanged(aviz::common::properties::Property* this_property);

 protected:
  /// Load the value of this property specifically, not including children.
  void loadValue(const Config& config);

  /// This is the central property value.
  QVariant value_;

  /// Pointer to the PropertyTreeModel managing this property tree.
  PropertyTreeModel* model_;

  /// True if row_number_within_parent_ of all children is valid, false if not.
  bool child_indexes_valid_;

  QIcon icon_;

 private:
  /// Set row_number_within_parent_ correctly for every child.
  void reindexChildren();

  Property* parent_;
  QList<Property*> children_;
  QString description_;
  bool hidden_;

  /// The property returned by subProp() when the requested name is not found.
  static Property* failprop_;

  int row_number_within_parent_;
  bool is_read_only_;
  bool is_expanded_;
  bool save_;
};

}  // namespace properties
}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__PROPERTIES__PROPERTY_HPP_
