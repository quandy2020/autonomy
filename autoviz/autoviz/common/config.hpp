// Copyright (c) 2012, Willow Garage, Inc.
// Copyright (c) 2017, Open Source Robotics Foundation, Inc.
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/**
 * @file config.hpp
 * @brief Format-independent hierarchical configuration tree (RViz Config port).
 *
 * Stores configuration as a tree of Map / List / Value / Empty nodes with
 * reference-counted internal @c Node objects. Used with
 * @ref YamlConfigReader and @ref YamlConfigWriter for @c .autoviz / @c .rviz
 * files, and with @ref SessionConfigToConfig / @ref SessionConfigFromConfig.
 *
 * @see YamlConfigReader
 * @see YamlConfigWriter
 * @see config_session.hpp
 */

#ifndef AUTOVIZ_COMMON__CONFIG_HPP_
#define AUTOVIZ_COMMON__CONFIG_HPP_

#include <algorithm>
#include <cstdio>
#include <memory>
#include <string>

#include <QMap>  // NOLINT: cpplint is unable to handle the include order here
#include <QString>  // NOLINT: cpplint is unable to handle the include order here
#include <QVariant>  // NOLINT: cpplint is unable to handle the include order here

namespace autoviz {
namespace common
{

/**
 * @class Config
 * @brief Flexible hierarchical configuration data store.
 *
 * The purpose of the Config class is to provide a flexible place to store
 * configuration data during saving and loading which is independent of the
 * particular storage format (like YAML or XML or INI). The data is stored in
 * a tree structure, supporting both named and numerically-indexed children.
 * Leaves store @c QVariant values, with convenience functions for int, float,
 * @c QString, and bool.
 *
 * Config instances are references to an internal @c Node class which actually
 * stores the data and the tree structure. Nodes are reference-counted and
 * deletion is handled automatically. This makes it safe to hold a reference
 * to a portion of a Config tree to use later, because the internal Nodes
 * beneath the saved reference will not be destroyed when the root of the tree
 * goes out of scope.
 *
 * ## Typical reading
 *
 * @code
 * YamlConfigReader reader;
 * Config config;
 * reader.readFile(config, "my_file.yaml");
 * if (!reader.error()) {
 *   int height = 0, width = 0;
 *   // …
 * }
 * @endcode
 *
 * ## Typical writing
 *
 * @code
 * Config config;
 * config.mapSetValue("Height", height());
 * YamlConfigWriter writer;
 * writer.writeFile(config, "my_file.yaml");
 * @endcode
 *
 * @note @ref setType() can change a node's type; mutating helpers
 *       (@ref mapSetValue, @ref mapMakeChild, @ref setValue, @ref listAppendNew)
 *       call it internally as needed. Changing type destroys incompatible data
 *       (except child nodes still referenced elsewhere).
 *
 * @see YamlConfigReader
 * @see YamlConfigWriter
 */
class  Config
{
private:
  class Node;
  typedef std::shared_ptr<Node> NodePtr;

public:
  /**
   * @brief Default constructor; creates an empty (Empty-type) config object.
   */
  Config();

  /**
   * @brief Copy constructor; copies only the reference to the data, not the data itself.
   * @param source Source config (shares the same internal Node).
   */
  Config(const Config & source);

  /**
   * @brief Converting constructor; makes a Value-type Config with the given value.
   * @param value Initial leaf value.
   */
  explicit Config(QVariant value);

  /**
   * @brief Assignment; shares the source's Node reference.
   * @param source Source config.
   * @return @c *this.
   */
  Config &
  operator=(const Config & source);

  /**
   * @brief Makes this object a deep copy of @p source.
   * @param source Config tree to duplicate.
   */
  void
  copy(const Config & source);

  /**
   * @enum Type
   * @brief Possible types a Config Node can have.
   *
   * @c Invalid means the Config object does not point to a Node at all.
   * Invalid Config objects are returned by data access functions when the
   * data does not exist (e.g. @ref listChildAt(7) on a list of length 3).
   */
  enum Type {
    Map,      /**< Named children (string keys). */
    List,     /**< Numerically indexed children. */
    Value,    /**< Leaf holding a @c QVariant. */
    Empty,    /**< Valid but empty node. */
    Invalid   /**< No Node referenced. */
  };

  /**
   * @brief Returns the Type of the referenced Node, or Invalid if none.
   * @return Current @ref Type.
   */
  Type
  getType() const;

  /**
   * @brief Sets the type of this Config Node.
   *
   * If @p new_type is Invalid, this de-references the node and makes the
   * Config object invalid. If the new type differs from the old type, existing
   * data in the Node is deleted and the type changes. Same-type calls are
   * no-ops. If currently invalid and @p new_type is not Invalid, a new Node
   * is created and referenced.
   *
   * @param new_type Desired type.
   */
  void
  setType(Type new_type);

  /**
   * @brief Returns whether the internal Node reference is valid.
   *
   * Same as (@ref getType() != Invalid).
   *
   * @return @c true if a Node is referenced.
   */
  bool
  isValid() const;

  /**
   * @brief Sets a named child to the given value (forces Map type).
   *
   * Since @c QVariant has constructors for int, float, bool, @c QString, and
   * other supported types, you can call this directly with your data in most
   * cases. Equivalent to @c mapMakeChild(key).setValue(value).
   *
   * @param key Child key.
   * @param value Value to store.
   */
  void
  mapSetValue(const QString & key, QVariant value);

  /**
   * @brief Creates a child node under @p key and returns it (forces Map type).
   *
   * @param key Child key.
   * @return Config referencing the new child.
   */
  Config
  mapMakeChild(const QString & key);

  /**
   * @brief Returns a reference to the child if this Node is a Map containing @p key.
   *
   * @param key Child key.
   * @return Child Config, or Invalid if missing / wrong type.
   */
  Config
  mapGetChild(const QString & key) const;

  /**
   * @brief Looks up a named value child.
   *
   * @param key Child key.
   * @param[out] value_out Receives the value on success (non-null).
   * @return @c true if a Value child with @p key exists.
   */
  bool
  mapGetValue(const QString & key, QVariant * value_out) const;

  /**
   * @brief Looks up a named integer (int or string-ified int).
   *
   * @param key Child key.
   * @param[out] value_out Receives the integer on success.
   * @return @c true on success.
   */
  bool
  mapGetInt(const QString & key, int * value_out) const;

  /**
   * @brief Looks up a named float (float, double, or string-ified).
   *
   * @param key Child key.
   * @param[out] value_out Receives the float on success.
   * @return @c true on success.
   */
  bool
  mapGetFloat(const QString & key, float * value_out) const;

  /**
   * @brief Looks up a named boolean (bool or string-ified bool).
   *
   * @param key Child key.
   * @param[out] value_out Receives the bool on success.
   * @return @c true on success.
   */
  bool
  mapGetBool(const QString & key, bool * value_out) const;

  /**
   * @brief Looks up a named string value.
   *
   * @param key Child key.
   * @param[out] value_out Receives the string on success.
   * @return @c true on success.
   */
  bool
  mapGetString(const QString & key, QString * value_out) const;

  /**
   * @brief Ensures this is a valid Value-type node and sets its value.
   * @param value New leaf value.
   */
  void
  setValue(const QVariant & value);

  /**
   * @brief Returns the leaf value if this is a valid Value-type node.
   * @return Value, or an invalid @c QVariant otherwise.
   */
  QVariant
  getValue() const;

  /**
   * @brief Returns the list length, or 0 if not a List.
   * @return Number of list children.
   */
  int
  listLength() const;

  /**
   * @brief Returns the i'th child if this Node has type List.
   *
   * @param i Zero-based index.
   * @return Child Config, or Invalid if out of range / wrong type.
   */
  Config
  listChildAt(int i) const;

  /**
   * @brief Appends a new empty Node to the list and returns a reference.
   *
   * Forces List type when needed. Returns Invalid if the operation fails.
   *
   * @return Config referencing the new list child.
   */
  Config
  listAppendNew();

  /**
   * @class MapIterator
   * @brief Iterator for looping over all entries in a Map-type Config Node.
   *
   * Typical usage:
   * @code
   * for (Config::MapIterator iter = config.mapIterator(); iter.isValid();
   *      iter.advance()) {
   *   QString key = iter.currentKey();
   *   Config child = iter.currentChild();
   * }
   * @endcode
   *
   * Maps are stored in alphabetical order of their keys; MapIterator uses
   * the same order.
   */
  class  MapIterator
  {
    // *INDENT-OFF*
  public:
    // *INDENT-ON*
    /**
     * @brief Advances the iterator to the next entry.
     */
    void
    advance();

    /**
     * @brief Returns whether the iterator currently points to a valid entry.
     *
     * This is how you tell if your loop over entries is at the end.
     *
     * @return @c true if current entry is valid.
     */
    bool
    isValid();

    /**
     * @brief Resets the iterator to the start of the map.
     */
    void
    start();

    /**
     * @brief Returns the name of the current map entry.
     * @return Key string.
     */
    QString
    currentKey();

    /**
     * @brief Returns a Config reference to the current map entry.
     * @return Child config.
     */
    Config
    currentChild();

    // *INDENT-OFF*
  private:
    // *INDENT-ON*
    /**
     * @brief Private constructor; only @ref Config may create MapIterators.
     */
    MapIterator();

    Config::NodePtr node_;
    QMap<QString, Config::NodePtr>::const_iterator iterator_;
    bool iterator_valid_;
    friend class Config;
  };

  /**
   * @brief Returns a new iterator for looping over key/value pairs.
   *
   * The returned MapIterator is initialized to point at the start of the map.
   * If this Config is Invalid or its Node is not a Map, returns an iterator
   * for which @ref MapIterator::isValid always returns @c false.
   *
   * @return Map iterator positioned at the first entry (if any).
   */
  MapIterator
  mapIterator() const;

private:
  /**
   * @brief Internal constructor wrapping an existing Node.
   * @param node Shared node pointer.
   */
  explicit Config(NodePtr node);

  /**
   * @brief Returns a Config with no Node (Invalid).
   * @return Invalid config sentinel.
   */
  static
  Config
  invalidConfig();

  /**
   * @brief If the node pointer is nullptr, sets it to a new empty node.
   */
  void
  makeValid();

  NodePtr node_; /**< Shared reference to the internal tree node. */

  friend class MapIterator;
};

}  // namespace common
}  // namespace autoviz

#endif  // AUTOVIZ_COMMON__CONFIG_HPP_
