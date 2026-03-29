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

#include "autonomy/tools/aviz/common/yaml_config_reader.hpp"

#include <fstream>
#include <sstream>
#include <string>

// TODO(wjwwood): consider restoring support for yamlcpp < 0.5, force 0.5 for
// now
#define AVIZ_HAVE_YAMLCPP_05 1

#ifdef AVIZ_HAVE_YAMLCPP_05
#ifdef _WIN32
#define YAML_CPP_DLL
#endif
#include <yaml-cpp/yaml.h>
#else
#include <yaml-cpp/node.h>
#include <yaml-cpp/parser.h>
#endif

namespace aviz {
namespace common {

YamlConfigReader::YamlConfigReader() : error_(false) {}

void YamlConfigReader::readFile(Config& config, const QString& filename) {
    std::ifstream in(qPrintable(filename));
    readStream(config, in, filename);
}

void YamlConfigReader::readString(Config& config, const QString& data,
                                  const QString& filename) {
    std::stringstream ss(data.toStdString());
    readStream(config, ss, filename);
}

void YamlConfigReader::readStream(Config& config, std::istream& in,
                                  const QString& filename) {
    (void)filename;
    try {
        YAML::Node yaml_node;
#ifdef AVIZ_HAVE_YAMLCPP_05
        yaml_node = YAML::Load(in);
#else
        YAML::Parser parser(in);
        parser.GetNextDocument(yaml_node);
#endif
        error_ = false;
        message_ = "";
        readYamlNode(config, yaml_node);
    } catch (YAML::ParserException& ex) {
        message_ = ex.what();
        error_ = true;
    }
}

void YamlConfigReader::readYamlNode(Config& config,
                                    const YAML::Node& yaml_node) {
    switch (yaml_node.Type()) {
        case YAML::NodeType::Map: {
#ifdef AVIZ_HAVE_YAMLCPP_05
            for (YAML::const_iterator it = yaml_node.begin();
                 it != yaml_node.end(); ++it)
#else
            for (YAML::Iterator it = yaml_node.begin(); it != yaml_node.end();
                 ++it)
#endif
            {
                std::string key;
#ifdef AVIZ_HAVE_YAMLCPP_05
                key = it->first.as<std::string>();
#else
                it.first() >> key;
#endif
                Config child = config.mapMakeChild(QString::fromStdString(key));
#ifdef AVIZ_HAVE_YAMLCPP_05
                readYamlNode(child, it->second);
#else
                readYamlNode(child, it.second());
#endif
            }
            break;
        }
        case YAML::NodeType::Sequence: {
#ifdef AVIZ_HAVE_YAMLCPP_05
            for (YAML::const_iterator it = yaml_node.begin();
                 it != yaml_node.end(); ++it)
#else
            for (YAML::Iterator it = yaml_node.begin(); it != yaml_node.end();
                 ++it)
#endif
            {
                Config child = config.listAppendNew();
                readYamlNode(child, *it);
            }
            break;
        }
        case YAML::NodeType::Scalar: {
            std::string s;
#ifdef AVIZ_HAVE_YAMLCPP_05
            s = yaml_node.as<std::string>();
#else
            yaml_node >> s;
#endif
            config.setValue(QString::fromStdString(s));
            break;
        }
        case YAML::NodeType::Null:
        default:
            break;
    }
}

bool YamlConfigReader::error() {
    return error_;
}

QString YamlConfigReader::errorMessage() {
    return message_;
}

}  // namespace common
}  // namespace aviz
