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

 #pragma once

 #include <memory>
#include <string>

#include "autoview/common/interaction/client_abstraction_iface.hpp"
#include "autoview/common/interaction/node_abstraction_iface.hpp"


namespace autoview {
namespace common {
namespace interaction {

class ClientAbstraction : public ClientAbstractionIface
{
public:
  ClientAbstraction();

  // TODO(wjwwood): Figure out which exceptions can be raised and document them
  //                consider consolidating all possible exceptions to a few
  //                exceptions defined in this library, to avoid inconsistent
  //                exceptions based on the underly ROS version.
  //                Also define an exception for repeated calls to this function
  //                under ROS 1, which will not be allowed.
  /// Initialize ROS, create the ROS node and return it (inside a weak pointer).
  /**
   * argc and argv maybe mutate to remove any command line arguments consumed by ROS.
   *
   * The returned ROS node will be used for other API calls.
   * This function throws if called multiple times.
   *
   * \param argc number of elements in argv
   * \param argv command line arguments as an array of c-string
   * \param name desired node name, or base node name if using an anonymous name
   * \param anonymous_name if true then the ROS node name will be randomized
   * \return weak pointer of the created ros node
   */
  NodeAbstractionIface::WeakPtr
  init(int argc, char ** argv, const std::string & name, bool anonymous_name) override;

  /// Check if ROS is "ok" or not, usually if ROS has been shutdown or not.
  /**
   * \param node_name the name of the node returned by ros_integration::init()
   * \return true if ok, otherwise false
   */
  bool ok() override;

  /**
   * This will also destroy any nodes which were created with
   * ros_integration::init().
   */
  void shutdown() override;

private:
  std::shared_ptr<NodeAbstractionIface> node_;
};

}  // namespace interaction
}  // namespace common
}  // namespace autoview