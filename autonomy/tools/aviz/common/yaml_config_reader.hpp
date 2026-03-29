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

#ifndef AVIZ_COMMON__YAML_CONFIG_READER_HPP_
#define AVIZ_COMMON__YAML_CONFIG_READER_HPP_

#include <istream>

#include "autonomy/tools/aviz/common/config.hpp"

namespace YAML {
class Node;
}

namespace aviz {
namespace common {

class YamlConfigReader
{
public:
    /// Constructor.
    /**
     * Object begins in a no-error state.
     */
    YamlConfigReader();

    /// Read config data from a file.
    /**
     * This potentially changes the return value of error(), statusMessage(),
     * and config().
     */
    void readFile(Config& config, const QString& filename);

    /// Read config data from a string.
    /**
     * This potentially changes the return value of error(), statusMessage(),
     * and config().
     */
    void readString(Config& config, const QString& data,
                    const QString& filename = "data string");

    /// Read config data from a std::istream.
    /**
     * This potentially changes the return value of error(), statusMessage(),
     * and config().
     */
    void readStream(Config& config, std::istream& in,
                    const QString& filename = "data stream");

    /// Return true if the latest readFile() or readString() call had an error.
    bool error();

    /// Return an error message if the latest read call had an error, or the
    /// empty string if not.
    QString errorMessage();

private:
    void readYamlNode(Config& config, const YAML::Node& yaml_node);

    QString message_;
    bool error_;
};

}  // namespace common
}  // namespace aviz

#endif  // AVIZ_COMMON__YAML_CONFIG_READER_HPP_
