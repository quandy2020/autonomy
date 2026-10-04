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
 * @file yaml_config_reader.hpp
 * @brief Reads YAML into a hierarchical @ref Config tree (RViz YamlConfigReader).
 *
 * Supports file, string, and stream sources. After each read, check
 * @ref error() / @ref errorMessage() before using the filled @ref Config.
 *
 * @see Config
 * @see YamlConfigWriter
 * @see SessionConfigFromConfig
 */

#ifndef AUTOVIZ_COMMON__YAML_CONFIG_READER_HPP_
#define AUTOVIZ_COMMON__YAML_CONFIG_READER_HPP_

#include <istream>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "autoviz/common/config.hpp"

#include "yaml-cpp/yaml.h"

namespace autoviz {
namespace common
{

/**
 * @class YamlConfigReader
 * @brief Parses YAML documents into @ref Config trees.
 *
 * Object begins in a no-error state. Each read call may update @ref error()
 * and @ref errorMessage(). The @p filename arguments are used only for
 * diagnostics in error messages (logical source name).
 */
class  YamlConfigReader
{
public:
  /**
   * @brief Constructs a reader in a no-error state.
   */
  YamlConfigReader();

  /**
   * @brief Reads config data from a file into @p config.
   *
   * Potentially changes the return values of @ref error() and
   * @ref errorMessage().
   *
   * @param[out] config Destination config tree (overwritten on success).
   * @param filename Filesystem path to open.
   */
  void readFile(Config & config, const QString & filename);

  /**
   * @brief Reads config data from a YAML string into @p config.
   *
   * @param[out] config Destination config tree.
   * @param data YAML document text.
   * @param filename Logical name for error messages (default
   *        @c "data string").
   */
  void readString(Config & config, const QString & data, const QString & filename = "data string");

  /**
   * @brief Reads config data from a @c std::istream into @p config.
   *
   * @param[out] config Destination config tree.
   * @param in Input stream positioned at the YAML document.
   * @param filename Logical name for error messages (default
   *        @c "data stream").
   */
  void readStream(Config & config, std::istream & in, const QString & filename = "data stream");

  /**
   * @brief Returns whether the latest read call had an error.
   * @return @c true if the last @ref readFile / @ref readString /
   *         @ref readStream failed.
   */
  bool error();

  /**
   * @brief Returns an error message if the latest read had an error.
   * @return Error text, or the empty string if none.
   */
  QString errorMessage();

private:
  /**
   * @brief Recursively converts a yaml-cpp node into a @ref Config subtree.
   *
   * @param[out] config Destination config node.
   * @param yaml_node Source YAML node.
   */
  void readYamlNode(Config & config, const YAML::Node & yaml_node);

  QString message_; /**< Last error message (empty if ok). */
  bool error_;      /**< Last operation error flag. */
};

}  // namespace common
}  // namespace autoviz

#endif  // AUTOVIZ_COMMON__YAML_CONFIG_READER_HPP_
