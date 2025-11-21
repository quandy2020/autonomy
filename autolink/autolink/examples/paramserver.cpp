/**
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

 #include "autolink/examples/proto/examples.pb.h"

 #include <unistd.h>
 #include <atomic>
 #include <chrono>
 #include <iostream>
 #include <thread>
 
 #include "autolink/autolink.hpp"
 #include "autolink/parameter/parameter_client.hpp"
 #include "autolink/parameter/parameter_server.hpp"
 
 using autolink::Parameter;
 using autolink::ParameterClient;
 using autolink::ParameterServer;
 
 
int main(int argc, char** argv) {
    autolink::Init(*argv);
    auto node_unique = autolink::CreateNode("parameter");
    auto node = std::shared_ptr<autolink::Node>(std::move(node_unique));
    auto param_server = std::make_shared<ParameterServer>(node);
    auto param_client = std::make_shared<ParameterClient>(node, "parameter");
    param_server->SetParameter(Parameter("int", 1));
    Parameter parameter;
    param_server->GetParameter("int", &parameter);
    AINFO << "int: " << parameter.AsInt64();
    param_client->SetParameter(Parameter("string", "test"));
    param_client->GetParameter("string", &parameter);
    AINFO << "string: " << parameter.AsString();
    param_client->GetParameter("int", &parameter);
    AINFO << "int: " << parameter.AsInt64();
    autolink::WaitForShutdown();
    return 0;
}