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

#include "autonomy/common/port.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace commsgs {
namespace vision_msgs {

// Represents a 2D point in pixel coordinates.
// XY matches the sensor_msgs/Image convention: X is positive right and Y is positive down.
struct Point2D
{
    double x;
    double y;
};

// Represents a 2D pose (coordinates and a radian rotation). Rotation is positive counterclockwise.
struct Pose2D
{
    Point2D position;
    double theta;
};

// A 2D bounding box that can be rotated about its center.
// All dimensions are in pixels, but represented using floating-point
//   values to allow sub-pixel precision. If an exact pixel crop is required
//   for a rotated bounding box, it can be calculated using Bresenham's line
//   algorithm.
struct BoundingBox2D
{
    // The 2D position (in pixels) and orientation of the bounding box center.
    Pose2D center;

    // The total size (in pixels) of the bounding box surrounding the object relative
    //   to the pose of its center.
    double size_x;
    double size_y;
};

struct BoundingBox2DArray
{
    std_msgs::Header header;
    std::vector<BoundingBox2D> boxes;
};

// A 3D bounding box that can be positioned and rotated about its center (6 DOF)
// Dimensions of this box are in meters, and as such, it may be migrated to
//   another package, such as geometry_msgs, in the future.
struct BoundingBox3D
{
    // The 3D position and orientation of the bounding box center
    geometry_msgs::Pose center;
    
    // The total size of the bounding box, in meters, surrounding the object's center
    //   pose.
    geometry_msgs::Vector3 size;
};

struct BoundingBox3DArray
{
    std_msgs::Header header;
    std::vector<BoundingBox3D> boxes;
};

// An object hypothesis that contains no pose information.
// If you would like to define an array of ObjectHypothesis structs,
//   please see the Classification struct type.
struct ObjectHypothesis
{
    // The unique ID of the object class. To get additional information about
    //   this ID, such as its human-readable class name, listeners should perform a
    //   lookup in a metadata database. See vision_msgs/VisionInfo.msg for more detail.
    std::string class_id;

    // The probability or confidence value of the detected object. By convention,
    //   this value should lie in the range [0-1].
    double score;
};

// Defines a classification result.
//
// This result does not contain any position information. It is designed for
//   classifiers, which simply provide class probabilities given an instance of
//   source data (e.g., an image or a point cloud).
struct Classification
{
    std_msgs::Header header;
    
    // A list of class probabilities. This list need not provide a probability for
    //   every possible class, just ones that are nonzero, or above some
    //   user-defined threshold.
    std::vector<ObjectHypothesis> results;
    
    // Source data that generated this classification are not a part of the struct.
    // If you need to access them, use an exact or approximate time synchronizer in
    // your code, as this struct's header should match the header of the source
    // data.
};

// An object hypothesis that contains pose information.
// If you would like to define an array of ObjectHypothesisWithPose structs,
//   please see the Detection2D or Detection3D struct types.
struct ObjectHypothesisWithPose
{
    // The object hypothesis (ID and score).
    ObjectHypothesis hypothesis;

    // The 6D pose of the object hypothesis. This pose should be
    //   defined as the pose of some fixed reference point on the object, such as
    //   the geometric center of the bounding box, the center of mass of the
    //   object or the origin of a reference mesh of the object.
    // Note that this pose is not stamped; frame information can be defined by
    //   parent structs.
    // Also note that different classes predicted for the same input data may have
    //   different predicted 6D poses.
    geometry_msgs::PoseWithCovariance pose;
};

// Defines a 2D detection result.
//
// This is similar to a 2D classification, but includes position information,
//   allowing a classification result for a specific crop or image point to
//   to be located in the larger image.
struct Detection2D
{
    std_msgs::Header header;
    
    // Class probabilities
    std::vector<ObjectHypothesisWithPose> results;
    
    // 2D bounding box surrounding the object.
    BoundingBox2D bbox;
    
    // ID used for consistency across multiple detection structs. Detections
    // of the same object in different detection structs should have the same id.
    // This field may be empty.
    std::string id;
    
    // Source data that generated this detection are not a part of the struct.
    // If you need to access them, use an exact or approximate time synchronizer in
    // your code, as this struct's header should match the header of the source
    // data.
};

// A list of 2D detections, for a multi-object 2D detector.
struct Detection2DArray
{
    std_msgs::Header header;
    
    // A list of the detected proposals. A multi-proposal detector might generate
    //   this list with many candidate detections generated from a single input.
    std::vector<Detection2D> detections;
};

// Defines a 3D detection result.
//
// This extends a basic 3D classification by including the pose of the
// detected object.
struct Detection3D
{
    std_msgs::Header header;
    
    // Class probabilities. Does not have to include hypotheses for all possible
    //   object ids, the scores for any ids not listed are assumed to be 0.
    std::vector<ObjectHypothesisWithPose> results;
    
    // 3D bounding box surrounding the object.
    BoundingBox3D bbox;
    
    // ID used for consistency across multiple detection structs. Detections
    // of the same object in different detection structs should have the same id.
    // This field may be empty.
    std::string id;
    
    // Source data that generated this classification are not a part of the struct.
    // If you need to access them, use an exact or approximate time synchronizer in
    // your code, as this struct's header should match the header of the source
    // data.
};

// A list of 3D detections, for a multi-object 3D detector.
struct Detection3DArray
{
    std_msgs::Header header;
    
    // A list of the detected proposals. A multi-proposal detector might generate
    //   this list with many candidate detections generated from a single input.
    std::vector<Detection3D> detections;
};

// A key value pair that maps an integer class_id to a std::string class label
//   in computer vision systems.
struct VisionClass
{
    // The int value that identifies the class.
    // Elements identified with 65535, the maximum uint16 value are assumed
    //   to belong to the "UNLABELED" class. For vision pipelines using less
    //   than 255 classes the "UNLABELED" is the maximum value in the uint8
    //   range.
    uint32 class_id;

    // The name of the class represented by the class_id
    std::string class_name;
};

// Provides meta-information about a visual pipeline.
//
// This struct serves a similar purpose to sensor_msgs/CameraInfo, but instead
//   of being tied to hardware, it represents information about a specific
//   computer vision pipeline. This information stays constant (or relatively
//   constant) over time, and so it is wasteful to send it with each individual
//   result. By listening to these structs, subscribers will receive
//   the context in which published vision structs are to be interpreted.
// Each vision pipeline should publish its LabelInfo structs to its own topic,
//   in a manner similar to CameraInfo.
// This struct is meant to allow converting data from vision pipelines that
//   return id based classifications back to human readable std::string class names.
struct LabelInfo
{
    // Used for sequencing
    std_msgs::Header header;

    // An array of uint16 keys and std::string values containing the association
    //   between class identifiers and their names. According to the amount
    //   of classes and the datatype used to store their ids internally, the
    //   maxiumum class id allowed (65535 for uint16 and 255 for uint8) belongs to
    //   the "UNLABELED" class.
    std::vector<VisionClass> class_map;

    // The value between 0-1 used as confidence threshold for the inference.
    float threshold;
};

// Provides meta-information about a visual pipeline.
//
// This struct serves a similar purpose to sensor_msgs/CameraInfo, but instead
//   of being tied to hardware, it represents information about a specific
//   computer vision pipeline. This information stays constant (or relatively
//   constant) over time, and so it is wasteful to send it with each individual
//   result. By listening to these structs, subscribers will receive
//   the context in which published vision structs are to be interpreted.
// Each vision pipeline should publish its VisionInfo structs to its own topic,
//   in a manner similar to CameraInfo.
struct VisionInfo
{
    // Used for sequencing
    std_msgs::Header header;
    
    // Name of the vision pipeline. This should be a value that is meaningful to an
    //   outside user.
    std::string method;
    
    // Location where the metadata database is stored. The recommended location is
    //   as an XML std::string on the ROS parameter server, but the exact implementation
    //   and information is left up to the user.
    // The database should store information attached to class ids. Each
    //   class id should map to an atomic, visually recognizable element. This
    //   definition is intentionally vague to allow extreme flexibility. The
    //   elements could be classes in a pixel segmentation algorithm, object classes
    //   in a detector, different people's faces in a face detection algorithm, etc.
    //   Vision pipelines report results in terms of numeric IDs, which map into
    //   this  database.
    // The information stored in this database is, again, left up to the user. The
    //   database could be as simple as a map from ID to class name, or it could
    //   include information such as object meshes or colors to use for
    //   visualization.
    std::string database_location;
    
    // Metadata database version. This counter is incremented
    //   each time the pipeline begins using a new version of the database (useful
    //   in the case of online training or user modifications).
    //   The counter value can be monitored by listeners to ensure that the pipeline
    //   and the listener are using the same metadata.
    int32 database_version;
};

}  // namespace vision_msgs
}  // namespace commsgs
}  // namespace autonomy