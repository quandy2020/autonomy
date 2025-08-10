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

#pragma once 

#include <vector>
#include <string>

#include "autonomy/commsgs/proto/std_msgs.pb.h"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace commsgs {
namespace std_msgs {

// Standard metadata for higher-level stamped data types.
// This is generally used to communicate timestamped data
// in a particular coordinate frame.
struct Header 
{
    // Two-integer timestamp that is expressed as seconds and nanoseconds.
    builtin_interfaces::Time stamp;

    // Transform frame with which this data is associated.
    std::string frame_id;
};

struct ColorRGBA
{
    float r;
    float g;
    float b;
    float a;
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct MultiArrayDimension
{
    std::string label;   // label of given dimension
    uint32 size ;    // size of given dimension (in type units)
    uint32 stride ;  // stride of given dimension
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct MultiArrayLayout
{
    // The multiarray declares a generic multi-dimensional array of a
    // particular data type.  Dimensions are ordered from outer most
    // to inner most.
    //
    // Accessors should ALWAYS be written in terms of dimension stride
    // and specified outer-most dimension first.
    //
    // multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
    //
    // A standard, 3-channel 640x480 image with interleaved color channels
    // would be specified as:
    //
    // dim[0].label  = "height"
    // dim[0].size   80
    // dim[0].stride *640*480 = 921600  (note dim[0] stride is just size of image)
    // dim[1].label  = "width"
    // dim[1].size   = 640
    // dim[1].stride *640 920
    // dim[2].label  = "channel"
    // dim[2].size   
    // dim[2].stride 
    //
    // multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

    std::vector<MultiArrayDimension> dim ; // Array of dimension properties
    uint32 data_offset ;                        // padding bytes at front of data
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct Float32MultiArray
{
    // Please look at the MultiArrayLayout message definition for
    // documentation on all multiarrays.

    MultiArrayLayout layout ;        // specification of data layout
    std::vector<float> data ;                    // array of data
};

// This was originally provided as an example message.
// It is deprecated as of Foxy
// It is recommended to create your own semantically meaningful message.
// However if you would like to continue using this please use the equivalent in example_msgs.
struct String
{
    std::string data;
};

// Converts 'data' to a proto::std_msgs::Header.
proto::std_msgs::Header ToProto(const Header& data);

// Converts 'proto' to Header.
Header FromProto(const proto::std_msgs::Header& proto);

// Converts 'data' to a proto::std_msgs::ColorRGBA.
proto::std_msgs::ColorRGBA ToProto(const ColorRGBA& data);

// Converts 'proto' to ColorRGBA.
ColorRGBA FromProto(const proto::std_msgs::ColorRGBA& proto);

// Converts 'data' to a proto::std_msgs::MultiArrayDimension.
proto::std_msgs::MultiArrayDimension ToProto(const MultiArrayDimension& data);

// Converts 'proto' to MultiArrayDimension.
MultiArrayDimension FromProto(const proto::std_msgs::MultiArrayDimension& proto);

// Converts 'data' to a proto::std_msgs::MultiArrayLayout.
proto::std_msgs::MultiArrayLayout ToProto(const MultiArrayLayout& data);

// Converts 'proto' to MultiArrayLayout.
MultiArrayLayout FromProto(const proto::std_msgs::MultiArrayLayout& proto);

// Converts 'data' to a proto::std_msgs::Float32MultiArray.
proto::std_msgs::Float32MultiArray ToProto(const Float32MultiArray& data);

// Converts 'proto' to Float32MultiArray.
Float32MultiArray FromProto(const proto::std_msgs::Float32MultiArray& proto);

// Converts 'data' to a proto::std_msgs::String.
proto::std_msgs::String ToProto(const String& data);

// Converts 'proto' to String.
String FromProto(const proto::std_msgs::String& proto);

}  // namespace std_msgs
}  // namespace commsgs
}  // namespace autonomy