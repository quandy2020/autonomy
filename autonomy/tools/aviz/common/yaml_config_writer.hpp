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

#ifndef AVIZ_COMMON__YAML_CONFIG_WRITER_HPP_
#define AVIZ_COMMON__YAML_CONFIG_WRITER_HPP_

#include <ostream>

#include "autonomy/tools/aviz/common/config.hpp"

namespace YAML {
class Emitter;
}

namespace aviz {
namespace common {

class YamlConfigWriter
{
public:
    /// Constructor.
    /**
     * Writer starts in a non-error state.
     */
    YamlConfigWriter();

    /// Write config data to a file.
    /**
     * This potentially changes the return values of error() and statusMessage().
     */
    void writeFile(const Config& config, const QString& filename);

    /// Write config data to a string, and return it.
    /**
     * This potentially changes the return values of error() and statusMessage().
     */
    QString writeString(const Config& config, const QString& filename = "data string");

    /// Write config data to a std::ostream.
    /**
     * This potentially changes the return values of error() and statusMessage().
     */
    void writeStream(const Config& config, std::ostream& out, const QString& filename = "data stream");

    /// Return true if the latest write operation had an error.
    bool error();

    /// Return an error message if the latest write call had an error, else the empty string.
    QString errorMessage();

private:
    void writeConfigNode(const Config& config, YAML::Emitter& emitter);

    QString message_;
    bool error_;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__YAML_CONFIG_WRITER_HPP_
