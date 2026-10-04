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
 * @file yaml_config_writer.hpp
 * @brief Writes a hierarchical @ref Config tree as YAML (RViz YamlConfigWriter).
 *
 * Supports file, string, and stream destinations. After each write, check
 * @ref error() / @ref errorMessage().
 *
 * @see Config
 * @see YamlConfigReader
 * @see SessionConfigToConfig
 */

#ifndef AUTOVIZ_COMMON__YAML_CONFIG_WRITER_HPP_
#define AUTOVIZ_COMMON__YAML_CONFIG_WRITER_HPP_

#include <ostream>

#include <QString>  // NOLINT: cpplint is unable to handle the include order here

#include "autoviz/common/config.hpp"

namespace YAML
{
class Emitter;
}

namespace autoviz {
namespace common {

/**
 * @class YamlConfigWriter
 * @brief Serializes @ref Config trees to YAML.
 *
 * Writer starts in a non-error state. Each write call may update
 * @ref error() and @ref errorMessage(). The optional @p filename arguments
 * are used only for diagnostics in error messages.
 */
class  YamlConfigWriter
{
public:
  /**
   * @brief Constructs a writer in a non-error state.
   */
  YamlConfigWriter();

  /**
   * @brief Writes config data to a file.
   *
   * Potentially changes the return values of @ref error() and
   * @ref errorMessage().
   *
   * @param config Source config tree.
   * @param filename Filesystem path to create/overwrite.
   */
  void writeFile(const Config & config, const QString & filename);

  /**
   * @brief Writes config data to a string and returns it.
   *
   * @param config Source config tree.
   * @param filename Logical name for error messages (default
   *        @c "data string").
   * @return YAML text (empty or partial on error; check @ref error()).
   */
  QString writeString(const Config & config, const QString & filename = "data string");

  /**
   * @brief Writes config data to a @c std::ostream.
   *
   * @param config Source config tree.
   * @param out Output stream.
   * @param filename Logical name for error messages (default
   *        @c "data stream").
   */
  void writeStream(
    const Config & config,
    std::ostream & out,
    const QString & filename = "data stream");

  /**
   * @brief Returns whether the latest write operation had an error.
   * @return @c true if the last write failed.
   */
  bool error();

  /**
   * @brief Returns an error message if the latest write had an error.
   * @return Error text, or the empty string if none.
   */
  QString errorMessage();

private:
  /**
   * @brief Recursively emits a @ref Config subtree into a YAML emitter.
   *
   * @param config Source config node.
   * @param emitter Destination yaml-cpp emitter.
   */
  void writeConfigNode(const Config & config, YAML::Emitter & emitter);

  QString message_; /**< Last error message (empty if ok). */
  bool error_;      /**< Last operation error flag. */
};

}  // namespace common
}  // namespace autoviz

#endif  // AUTOVIZ_COMMON__YAML_CONFIG_WRITER_HPP_
