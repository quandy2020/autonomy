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

#include "autonomy/visualization/common/visualization_interface.hpp"
#include "autonomy/common/logging.hpp"
#include "absl/strings/str_cat.h"

namespace autonomy {
namespace visualization {
namespace common { 

proto::VisualizationOptions LoadOptions(
    autonomy::common::LuaParameterDictionary* const parameter_dictionary)
{
    proto::VisualizationOptions options;
    
    // Load basic server configuration
    options.set_host(parameter_dictionary->GetString("host"));
    options.set_port(parameter_dictionary->GetInt("port"));
    options.set_mcap_filename(parameter_dictionary->GetString("mcap_filename"));
    options.set_write_mcap_data(parameter_dictionary->GetBool("write_mcap"));
    
    // Load topic subscriptions  
    // Note: For now, we'll use a simpler approach by directly parsing individual subscriptions
    // The subscriptions in Lua is an array of tables
    int subscription_index = 1;
    while (true) {
        try {
            auto sub_key = "subscriptions." + std::to_string(subscription_index);
            if (!parameter_dictionary->HasKey(sub_key)) {
                break;
            }
            
            auto sub_dict = parameter_dictionary->GetDictionary(sub_key);
            auto* subscription = options.add_subscriptions();
            subscription->set_topic_name(sub_dict->GetString("topic_name"));
            subscription->set_message_type(sub_dict->GetString("message_type"));
            
            LOG(INFO) << "  [" << subscription_index << "] Topic: " << subscription->topic_name() 
                      << " (Type: " << subscription->message_type() << ")";
            
            subscription_index++;
        } catch (const std::exception& e) {
            LOG(WARNING) << "Error loading subscription " << subscription_index << ": " << e.what();
            break;
        }
    }
    
    if (subscription_index > 1) {
        LOG(INFO) << "Loaded " << (subscription_index - 1) << " topic subscriptions";
    } else {
        LOG(WARNING) << "No subscriptions configured in lua file";
    }
    
    return options;
}

}   // namespace common 
}   // namespace visualization
}   // namespace autonomy