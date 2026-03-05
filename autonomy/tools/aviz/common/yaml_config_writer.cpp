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

#include "autonomy/tools/aviz/common/yaml_config_writer.hpp"

#include <fstream>
#include <sstream>

#ifdef _WIN32
#define YAML_CPP_DLL
#endif
#include "yaml-cpp/emitter.h"

namespace aviz {
namespace common {

YamlConfigWriter::YamlConfigWriter() : error_(false) {}

void YamlConfigWriter::writeFile(const Config& config, const QString& filename) {
  try {
    std::ofstream out(qPrintable(filename));
    if (out) {
      writeStream(config, out, filename);
    } else {
      error_ = true;
      message_ = "Failed to open " + filename + " for writing.";
    }
  } catch (const std::exception& ex) {
    error_ = true;
    message_ = ex.what();
  }
}

QString YamlConfigWriter::writeString(const Config& config, const QString& filename) {
  std::stringstream out;
  writeStream(config, out, filename);
  if (!error_) {
    return QString::fromStdString(out.str());
  } else {
    return "";
  }
}

void YamlConfigWriter::writeStream(const Config& config, std::ostream& out, const QString& filename) {
  (void)filename;
  error_ = false;
  message_ = "";
  YAML::Emitter emitter;
  writeConfigNode(config, emitter);
  if (!error_) {
    out << emitter.c_str() << std::endl;
  }
}

bool YamlConfigWriter::error() { return error_; }

QString YamlConfigWriter::errorMessage() { return message_; }

void YamlConfigWriter::writeConfigNode(const Config& config, YAML::Emitter& emitter) {
  switch (config.getType()) {
    case Config::List: {
      emitter << YAML::BeginSeq;
      for (int i = 0; i < config.listLength(); i++) {
        writeConfigNode(config.listChildAt(i), emitter);
      }
      emitter << YAML::EndSeq;
      break;
    }
    case Config::Map: {
      emitter << YAML::BeginMap;
      Config::MapIterator map_iter = config.mapIterator();
      while (map_iter.isValid()) {
        Config child = map_iter.currentChild();

        emitter << YAML::Key;
        emitter << map_iter.currentKey().toStdString();
        emitter << YAML::Value;
        writeConfigNode(child, emitter);

        map_iter.advance();
      }
      emitter << YAML::EndMap;
      break;
    }
    case Config::Value: {
      QString value = config.getValue().toString();
      if (value.size() == 0) {
        emitter << YAML::DoubleQuoted << "";
      } else {
        emitter << value.toStdString();
      }
      break;
    }
    default:
      emitter << YAML::Null;
      break;
  }
}

}  // namespace common
}  // namespace aviz
