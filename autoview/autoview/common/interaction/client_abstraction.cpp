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

 #include "autoview/common/interaction/client_abstraction.hpp"

 #include <memory>
 #include <mutex>
 #include <string>
 
 #include "autolink/autolink.hpp"
 
 #include "autoview/common/interaction/node_abstraction.hpp"
 
 namespace autoview {
 namespace common {
 namespace interaction {
 
 ClientAbstraction::ClientAbstraction()   
 {}
 
 NodeAbstractionIface::WeakPtr
 ClientAbstraction::init(int argc, char ** argv, const std::string & name, bool anonymous_name)
 {
   std::string final_name = name;
   if (anonymous_name) {
     throw std::runtime_error("'anonymous_name' feature not implemented");
   }
   autolink::Init(argc > 0 ? argv[0] : "autoview", "");
   if (node_ && node_->get_node_name() == final_name) {
     throw std::runtime_error("Node with name " + final_name + " already exists.");
   }
   node_ = std::make_shared<NodeAbstraction>(final_name);
   return node_;
 }
 
 bool ClientAbstraction::ok()
 {
   return autolink::OK() && node_;
 }
 
void ClientAbstraction::shutdown()
{
  autolink::AsyncShutdown();
}
 
}  // namespace interaction
}  // namespace common
}  // namespace autoview
