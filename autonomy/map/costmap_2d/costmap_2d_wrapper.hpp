/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *********************************************************************/

#pragma once

#include <vector>
#include <string>

#include "autonomy/map/proto/map_options.pb.h"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/map_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/map/common/map_interface.hpp"

namespace autonomy {
namespace map {
namespace costmap_2d {

class Costmap2DWrapper : public common::MapInterface
{
public:
    /**
     * Define Costmap2DWrapper::SharedPtr type
     */
    AUTONOMY_SMART_PTR_DEFINITIONS(Costmap2DWrapper)

    /**
     * @brief A constructor for nautonomy::map::costmap_2d::Costmap2DWrapper
     * @param options Additional options to control creation of the node.
     */
    Costmap2DWrapper(const proto::Costmap2DOptions& options);

    /**
     * @brief A Destructor for autonomy::map::costmap_2d::Costmap2DWrapper
     */
    ~Costmap2DWrapper();


protected:
    // options for costmap 2D
    proto::Costmap2DOptions options_;
    
};

}  // namespace costmap_2d
}  // namespace map
}  // namespace autonomy
