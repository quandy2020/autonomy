/*
 * Copyright 2025 The Openbot Authors (duyongquan)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "autonomy/transform/urdf/model.hpp"

#include <cassert>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "autolink/class_loader/class_loader_manager.hpp"
#include "autonomy/transform/urdf/parser.hpp"
#include "urdf_model/model.h"

namespace autonomy {
namespace transform {

class ModelImplementation final
{
public:
    ModelImplementation() {
        // Load the urdf_parser_plugin library
        loader_manager_.LoadLibrary("urdf_parser_plugin");
    }

    ~ModelImplementation() = default;

    std::shared_ptr<::urdf::URDFParser> load_plugin(
        const std::string& plugin_name);

    // Loader manager used to get plugins
    autolink::class_loader::ClassLoaderManager loader_manager_;
};

Model::Model() : impl_(new ModelImplementation) {}

Model::~Model() {
    clear();
    impl_.reset();
}

bool Model::initFile(const std::string& filename) {
    // get the entire file
    std::string xml_string;
    std::fstream xml_file(filename.c_str(), std::fstream::in);
    if (xml_file.is_open()) {
        while (xml_file.good()) {
            std::string line;
            std::getline(xml_file, line);
            xml_string += (line + "\n");
        }
        xml_file.close();
        return Model::initString(xml_string);
    } else {
        fprintf(stderr, "Could not open file [%s] for parsing.\n",
                filename.c_str());
        return false;
    }
}

std::shared_ptr<::urdf::URDFParser> ModelImplementation::load_plugin(
    const std::string& plugin_name) {
    std::shared_ptr<::urdf::URDFParser> plugin_instance =
        loader_manager_.CreateClassObj<::urdf::URDFParser>(plugin_name);
    if (!plugin_instance) {
        fprintf(stderr, "Failed to load urdf_parser_plugin [%s]\n",
                plugin_name.c_str());
    }
    return plugin_instance;
}

bool Model::initString(const std::string& data) {
    ::urdf::ModelInterfaceSharedPtr model;

    size_t best_score = std::numeric_limits<size_t>::max();
    std::shared_ptr<urdf::URDFParser> best_plugin = nullptr;
    std::string best_plugin_name;

    // Figure out what plugins might handle this format
    for (const std::string& plugin_name :
         impl_->loader_manager_.GetValidClassNames<::urdf::URDFParser>()) {
        std::shared_ptr<urdf::URDFParser> plugin_instance =
            impl_->load_plugin(plugin_name);
        if (!plugin_instance) {
            // Debug mode
            assert(plugin_instance);
            // Release mode
            continue;
        }
        size_t score = plugin_instance->might_handle(data);
        if (score < best_score) {
            best_score = score;
            best_plugin = std::move(plugin_instance);
            best_plugin_name = plugin_name;
        }
    }

    if (best_score >= data.size()) {
        // No plugin was confident ... try urdf anyways
        best_plugin_name = "urdf_xml_parser/URDFXMLParser";
        best_plugin = impl_->load_plugin(best_plugin_name);
    }

    if (!best_plugin) {
        fprintf(stderr, "No plugin found for given robot description.\n");
        return false;
    }

    model = best_plugin->parse(data);

    // copy data from model into this object
    if (!model) {
        fprintf(stderr, "Failed to parse robot description using: %s\n",
                best_plugin_name.c_str());
        return false;
    }

    this->links_ = model->links_;
    this->joints_ = model->joints_;
    this->materials_ = model->materials_;
    this->name_ = model->name_;
    this->root_link_ = model->root_link_;
    return true;
}

}  // namespace transform
}  // namespace autonomy
