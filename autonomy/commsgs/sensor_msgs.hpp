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

#include "autonomy/commsgs/proto/sensor_msgs.pb.h"

#include "autonomy/common/port.hpp"
#include "autonomy/common/macros.hpp"
#include "autonomy/commsgs/std_msgs.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace commsgs {
namespace sensor_msgs {

// This struct is used to specify a region of interest within an image.
//
// When used to specify the ROI setting of the camera when the image was
// taken, the height and width fields should either match the height and
// width fields for the associated image; or height = width = 0
// indicates that the full resolution image was captured.
struct RegionOfInterest
{
    uint32 x_offset;  // Leftmost pixel of the ROI
                          // (0 if the ROI includes the left edge of the image)
    uint32 y_offset;  // Topmost pixel of the ROI (0 if the ROI includes the top edge of the image)
    uint32 height;    // Height of ROI
    uint32 width ;    // Width of ROI

    // True if a distinct rectified ROI should be calculated from the "raw"
    // ROI in this message. Typically this should be False if the full image
    // is captured (ROI not used), and True if a subwindow is captured (ROI
    // used).
    bool do_rectify;
};

// This struct defines meta information for a camera. It should be in a
// camera namespace on topic "camera_info" and accompanied by up to five
// image topics named:
//
//   image_raw - raw data from the camera driver, possibly Bayer encoded
//   image            - monochrome, distorted
//   image_color      - color, distorted
//   image_rect       - monochrome, rectified
//   image_rect_color - color, rectified
//
// The image_pipeline contains packages (image_proc, stereo_image_proc)
// for producing the four processed image topics from image_raw and
// camera_info. The meaning of the camera parameters are described in
// detail at http://www.ros.org/wiki/image_pipeline/CameraInfo.
//
// The image_geometry package provides a user-friendly interface to
// common operations using this meta information. If you want to, e.g.,
// project a 3d point into image coordinates, we strongly recommend
// using image_geometry.
//
// If the camera is uncalibrated, the matrices D, K, R, P should be left
// zeroed out. In particular, clients may assume that K[0] == 0.0
// indicates an uncalibrated camera.

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                     Image acquisition info                          //
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct CameraInfo
{
    
    // Time of image acquisition, camera coordinate frame ID
    std_msgs::Header header;    // Header timestamp should be acquisition time of image
                                // Header frame_id should be optical frame of camera
                                // origin of frame should be optical center of camera
                                // +x should point to the right in the image
                                // +y should point down in the image
                                // +z should point into the plane of the image


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                      Calibration Parameters                         //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // These are fixed during camera calibration. Their values will be the //
    // same in all messages until the camera is recalibrated. Note that    //
    // self-calibrating systems may "recalibrate" frequently.              //
    //                                                                     //
    // The internal parameters can be used to warp a raw (distorted) image //
    // to:                                                                 //
    //   1. An undistorted image (requires D and K)                        //
    //   2. A rectified image (requires D, K, R)                           //
    // The projection matrix P projects 3D points into the rectified image.//
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // The image dimensions with which the camera was calibrated.
    // Normally this will be the full camera resolution in pixels.
    uint32 height;
    uint32 width;

    // The distortion model used. Supported models are listed in
    // sensor_msgs/distortion_models.hpp. For most cameras, "plumb_bob" - a
    // simple model of radial and tangential distortion - is sufficent.
    std::string distortion_model;

    // The distortion parameters, size depending on the distortion model.
    // For "plumb_bob", the 5 parameters are: (k1, k2, t1, t2, k3).
    std::vector<double> d;

    // Intrinsic camera matrix for the raw (distorted) images.
    //     [fx  0 cx]
    // K = [ 0 fy cy]
    //     [ 0  0  1]
    // Projects 3D points in the camera coordinate frame to 2D pixel
    // coordinates using the focal lengths (fx, fy) and principal point
    // (cx, cy).
    std::vector<double> k; // float64[9]  k // 3x3 row-major matrix

    // Rectification matrix (stereo cameras only)
    // A rotation matrix aligning the camera coordinate system to the ideal
    // stereo image plane so that epipolar lines in both stereo images are
    // parallel.
    // float64[9]  r // 3x3 row-major matrix
    std::vector<double> r; 

    // Projection/camera matrix
    //     [fx'  0  cx' Tx]
    // P = [ 0  fy' cy' Ty]
    //     [ 0   0   1   0]
    // By convention, this matrix specifies the intrinsic (camera) matrix
    //  of the processed (rectified) image. That is, the left 3x3 portion
    //  is the normal camera intrinsic matrix for the rectified image.
    // It projects 3D points in the camera coordinate frame to 2D pixel
    //  coordinates using the focal lengths (fx', fy') and principal point
    //  (cx', cy') - these may differ from the values in K.
    // For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
    //  also have R = the identity and P[1:3,1:3] = K.
    // For a stereo pair, the fourth column [Tx Ty 0]' is related to the
    //  position of the optical center of the second camera in the first
    //  camera's frame. We assume Tz = 0 so both cameras are in the same
    //  stereo image plane. The first camera always has Tx = Ty = 0. For
    //  the right (second) camera of a horizontal stereo pair, Ty = 0 and
    //  Tx = -fx' * B, where B is the baseline between the cameras.
    // Given a 3D point [X Y Z]', the projection (x, y) of the point onto
    //  the rectified image is given by:
    //  [u v w]' = P * [X Y Z 1]'
    //         x = u / w
    //         y = v / w
    //  This holds for both images of a stereo pair.
    // float64[12] p // 3x4 row-major matrix
    std::vector<double> p; 


    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //                      Operational Parameters                         //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // These define the image region actually captured by the camera       //
    // driver. Although they affect the geometry of the output image, they //
    // may be changed freely without recalibrating the camera.             //
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    // Binning refers here to any camera setting which combines rectangular
    //  neighborhoods of pixels into larger "super-pixels." It reduces the
    //  resolution of the output image to
    //  (width / binning_x) x (height / binning_y).
    // The default values binning_x = binning_y = 0 is considered the same
    //  as binning_x = binning_y (no subsampling).
    uint32 binning_x0;
    uint32 binning_y1;

    // Region of interest (subwindow of full camera resolution), given in
    //  full resolution (unbinned) image coordinates. A particular ROI
    //  always denotes the same window of pixels on the camera sensor,
    //  regardless of binning settings.
    // The default setting of roi (all values 0) is considered the same as
    //  full resolution (roi.width = width, roi.height = height).
    RegionOfInterest roi2;
};

// This struct is used by the PointCloud struct to hold optional data
// associated with each point in the cloud. The length of the values
// array should be the same as the length of the points array in the
// PointCloud, and each value should be associated with the corresponding
// point.
//
// Channel names in existing practice include:
//   "u", "v" - row and column (respectively) in the left stereo image.
//              This is opposite to usual conventions but remains for
//              historical reasons. The newer PointCloud2 struct has no
//              such problem.
//   "rgb" - For point clouds produced by color stereo cameras. uint8
//           (R,G,B) values packed into the least significant 24 bits,
//           in order.
//   "intensity" - laser or pixel intensity.
//   "distance"
struct ChannelFloat32
{
    // The channel name should give semantics of the channel (e.g.
    // "intensity" instead of "value").
    std::string name;

    // The values array should be 1-1 with the elements of the associated
    // PointCloud.
    std::vector<float> values;
};

// This struct contains a compressed image.
struct CompressedImage
{
    // Define CompressedImage::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(CompressedImage)

    std_msgs::Header header;  // Header timestamp should be acquisition time of image
                                // Header frame_id should be optical frame of camera
                                // origin of frame should be optical center of cameara
                                // +x should point to the right in the image
                                // +y should point down in the image
                                // +z should point into to plane of the image

    std::string format;          // Specifies the format of the data
                                //   Acceptable values:
                                //     jpeg, png, tiff

    std::vector<uint32> data;   // Compressed image buffer
};

// Single photometric illuminance measurement.  Light should be assumed to be
// measured along the sensor's x-axis (the area of detection is the y-z plane).
// The illuminance should have a 0 or positive value and be received with
// the sensor's +X axis pointing toward the light source.
//
// Photometric illuminance is the measure of the human eye's sensitivity of the
// intensity of light encountering or passing through a surface.
//
// All other Photometric and Radiometric measurements should not use this message.
// This struct cannot represent:
//  - Luminous intensity (candela/light source output)
//  - Luminance (nits/light output per area)
//  - Irradiance (watt/area), etc.
struct Illuminance
{
    std_msgs::Header header; // timestamp is the time the illuminance was measured
                                // frame_id is the location and direction of the reading

    float illuminance;      // Measurement of the Photometric Illuminance in Lux.

    float variance;         // 0 is interpreted as variance unknown
};

struct Image 
{
    // Define Image::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(Image)

    std_msgs::Header header;     // Header timestamp should be acquisition time of image
                                 // Header frame_id should be optical frame of camera
                                 // origin of frame should be optical center of cameara
                                 // +x should point to the right in the image
                                 // +y should point down in the image
                                 // +z should point into to plane of the image
                                 // If the frame_id here and the frame_id of the CameraInfo
                                 // struct associated with the image conflict
                                 // the behavior is undefined

    // image height, that is, number of rows
    uint32 height;       
    
    // image width, that is, number of columns
    uint32 width;            

    // The legal values for encoding are in file src/image_encodings.cpp
    // If you want to standardize a new std::string format, join
    // ros-users@lists.ros.org and send an email proposing a new encoding.
    std::string encoding;    // Encoding of pixels -- channel meaning, ordering, size
                            // taken from the list of std::strings in include/sensor_msgs/image_encodings.hpp


    // is this data bigendian?
    uint32 is_bigendian;

    // Full row length in bytes
    uint32 step;

    // actual matrix data, size is (step * rows)
    std::vector<uint8> data;
};

// This is a struct to hold data from an IMU (Inertial Measurement Unit)
//
// Accelerations should be in m/s^2 (not in g's), and rotational velocity should be in rad/sec
//
// If the covariance of the measurement is known, it should be filled in (if all you know is the
// variance of each measurement, e.g. from the datasheet, just put those along the diagonal)
// A covariance matrix of all zeros will be interpreted as "covariance unknown", and to use the
// data a covariance will have to be assumed or gotten from some other source
//
// If you have no estimate for one of the data elements (e.g. your IMU doesn't produce an
// orientation estimate), please set element 0 of the associated covariance matrix to -1
// If you are interpreting this message, please check for a value of -1 in the first element of each
// covariance matrix, and disregard the associated estimate.
struct Imu
{
    // Define Imu::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(Imu)

    std_msgs::Header header;

    geometry_msgs::Quaternion orientation;
    std::vector<double> orientation_covariance; // float64[9] orientation_covariance 
                                                // Row major about x, y, z axes

    geometry_msgs::Vector3 angular_velocity;
    std::vector<double> angular_velocity_covariance;  // float64[9] angular_velocity_covariance  
                                                      // Row major about x, y, z axes

    geometry_msgs::Vector3 linear_acceleration;
    std::vector<double> linear_acceleration_covariance;;  // float64[9] linear_acceleration_covariance 
                                                          // Row major x, y z
};

// Single scan from a planar laser range-finder
//
// If you have another ranging device with different behavior (e.g. a sonar
// array), please find or create a different message, since applications
// will make fairly laser-specific assumptions about this data
struct LaserScan
{
    // Define LaserScan::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(LaserScan)

    std_msgs::Header header;    // timestamp in the header is the acquisition time of
                                // the first ray in the scan.
                                //
                                // in frame frame_id, angles are measured around
                                // the positive Z axis (counterclockwise, if Z is up)
                                // with zero angle being forward along the x axis

    float angle_min;           // start angle of the scan [rad]
    float angle_max;           // end angle of the scan [rad]
    float angle_increment;     // angular distance between measurements [rad]

    float time_increment;       // time between measurements [seconds] - if your scanner
                                    // is moving, this will be used in interpolating position
                                    // of 3d points
    float scan_time;            // time between scans [seconds]

    float range_min;            // minimum range value [m]
    float range_max;            // maximum range value [m]

    std::vector<float> ranges;           // range data [m]
                                         // (Note: values < range_min or > range_max should be discarded)
    std::vector<float> intensities;      // intensity data [device-specific units].  If your
                                         // device does not provide intensities, please leave the array empty.
};

// THIS MESSAGE IS DEPRECATED AS OF FOXY
// Please use sensor_msgs/PointCloud2

// This struct holds a collection of 3d points, plus optional additional
// information about each point.
struct PointCloud
{
    // Define PointCloud::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(PointCloud)

    // Time of sensor data acquisition, coordinate frame ID.
    std_msgs::Header header;

    // Array of 3d points. Each Point32 should be interpreted as a 3d point
    // in the frame given in the header.
    std::vector<geometry_msgs::Point32> points;

    // Each channel should have the same number of elements as points array,
    // and the data in each channel should correspond 1:1 with each point.
    // Channel names in common practice are listed in ChannelFloat32.msg.
    std::vector<ChannelFloat32> channels;
};


// This struct holds the description of one point entry in the
// PointCloud2 struct format.
// uint8 INT8   
// uint8 UINT8  
// uint8 INT16  
// uint8 UINT16 
// uint8 INT32  
// uint8 UINT32 
// uint8 FLOAT32
// uint8 FLOAT64
struct PointField
{
    // Common PointField names are x, y, z, intensity, rgb, rgba
    std::string name;      // Name of field
    uint32 offset;    // Offset from start of point struct
    uint32  datatype; // Datatype enumeration, see above
    uint32 count;     // How many elements in the field
};

// This struct holds a collection of N-dimensional points, which may
// contain additional information such as normals, intensity, etc. The
// point data is stored as a binary blob, its layout described by the
// contents of the "fields" array.
//
// The point cloud data may be organized 2d (image-like) or 1d (unordered).
// Point clouds organized as 2d images may be produced by camera depth sensors
// such as stereo or time-of-flight.
struct PointCloud2
{
    // Define PointCloud2::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(PointCloud2)

    // Time of sensor data acquisition, and the coordinate frame ID (for 3d points).
    std_msgs::Header header;

    // 2D structure of the point cloud. If the cloud is unordered, height is
    // 1 and width is the length of the point cloud.
    uint32 height;
    uint32 width;

    // Describes the channels and their layout in the binary data blob.
    std::vector<PointField> fields;

    bool    is_bigendian; // Is this data bigendian?
    uint32  point_step;   // Length of a point in bytes
    uint32  row_step;     // Length of a row in bytes
    std::vector<uint32> data; // Actual point data, size is (row_step*height)

    bool is_dense;        // True if there are no invalid points
};

// Single range reading from an active ranger that emits energy and reports
// one range reading that is valid along an arc at the distance measured.
// This struct is  not appropriate for laser scanners. See the LaserScan
// struct if you are working with a laser scanner.
//
// This struct also can represent a fixed-distance (binary) ranger.  This
// sensor will have min_range===max_range===distance of detection.
// These sensors follow REP 117 and will output -Inf if the object is detected
// and +Inf if the object is outside of the detection range.

struct Range
{
    // Define Range::SharedPtr type
    AUTONOMY_SMART_PTR_DEFINITIONS(Range)

    std_msgs::Header header; // timestamp in the header is the time the ranger
                                // returned the distance reading

    // // Radiation type enums
    // // If you want a value added to this list, send an email to the ros-users list
    // uint8 ULTRASOUND=0
    // uint8 INFRARED=1

    uint32 radiation_type;  // the type of radiation used by the sensor(sound, IR, etc) [enum]

    float field_of_view;    // the size of the arc that the distance reading is
                                // valid for [rad]
                                // the object causing the range reading may have
                                // been anywhere within -field_of_view/2 and
                                // field_of_view/2 at the measured range.
                                // 0 angle corresponds to the x-axis of the sensor.

    float min_range;       // minimum range value [m]
    float max_range;       // maximum range value [m]
                               // Fixed distance rangers require min_range==max_range

    float range;          // range data [m]  
                              // (Note: values < range_min or > range_max should be discarded)
                              // Fixed distance rangers only output -Inf or +Inf.
                              // -Inf represents a detection within fixed distance.
                              // (Detection too close to the sensor to quantify)
                              // +Inf represents no detection within the fixed distance.
                              // (Object out of range)
};

// Reports the state of a joystick's axes and buttons.
struct Joy
{
    // The timestamp is the time at which data is received from the joystick.
    std_msgs::Header header; // timestamp in the header is the time the ranger
                                // returned the distance reading

    // The axes measurements from a joystick.
    std::vector<float> axes; 
 
    // The buttons measurements from a joystick.
    std::vector<int32> buttons;
};

// Converts 'data' to a proto::sensor_msgs::RegionOfInterest.
proto::sensor_msgs::RegionOfInterest ToProto(const RegionOfInterest& data);

// Converts 'proto' to RegionOfInterest.
RegionOfInterest FromProto(const proto::sensor_msgs::RegionOfInterest& proto);

// Converts 'data' to a proto::sensor_msgs::CameraInfo.
proto::sensor_msgs::CameraInfo ToProto(const CameraInfo& data);

// Converts 'proto' to CameraInfo.
CameraInfo FromProto(const proto::sensor_msgs::CameraInfo& proto);

// Converts 'data' to a proto::sensor_msgs::ChannelFloat32.
proto::sensor_msgs::ChannelFloat32 ToProto(const ChannelFloat32& data);

// Converts 'proto' to ChannelFloat32.
ChannelFloat32 FromProto(const proto::sensor_msgs::ChannelFloat32& proto);

// Converts 'data' to a proto::sensor_msgs::CompressedImage.
proto::sensor_msgs::CompressedImage ToProto(const CompressedImage& data);

// Converts 'proto' to CompressedImage.
CompressedImage FromProto(const proto::sensor_msgs::CompressedImage& proto);

// Converts 'data' to a proto::sensor_msgs::Illuminance.
proto::sensor_msgs::Illuminance ToProto(const Illuminance& data);

// Converts 'proto' to Illuminance.
Illuminance FromProto(const proto::sensor_msgs::Illuminance& proto);

// Converts 'data' to a proto::sensor_msgs::Image.
proto::sensor_msgs::Image ToProto(const Image& data);

// Converts 'proto' to Image.
Image FromProto(const proto::sensor_msgs::Image& proto);

// Converts 'data' to a proto::sensor_msgs::Imu.
proto::sensor_msgs::Imu ToProto(const Imu& data);

// Converts 'proto' to Imu.
Imu FromProto(const proto::sensor_msgs::Imu& proto);

// Converts 'data' to a proto::sensor_msgs::LaserScan.
proto::sensor_msgs::LaserScan ToProto(const LaserScan& data);

// Converts 'proto' to LaserScan.
LaserScan FromProto(const proto::sensor_msgs::LaserScan& proto);

// Converts 'data' to a proto::sensor_msgs::PointCloud.
proto::sensor_msgs::PointCloud ToProto(const PointCloud& data);

// Converts 'proto' to PointCloud.
PointCloud FromProto(const proto::sensor_msgs::PointCloud& proto);

// Converts 'data' to a proto::sensor_msgs::PointField.
proto::sensor_msgs::PointField ToProto(const PointField& data);

// Converts 'proto' to PointField.
PointField FromProto(const proto::sensor_msgs::PointField& proto);

// Converts 'data' to a proto::sensor_msgs::PointCloud2.
proto::sensor_msgs::PointCloud2 ToProto(const PointCloud2& data);

// Converts 'proto' to PointCloud2.
PointCloud2 FromProto(const proto::sensor_msgs::PointCloud2& proto);

// Converts 'data' to a proto::sensor_msgs::Range.
proto::sensor_msgs::Range ToProto(const Range& data);

// Converts 'proto' to Range.
Range FromProto(const proto::sensor_msgs::Range& proto);

// Converts 'data' to a proto::sensor_msgs::Joy.
proto::sensor_msgs::Joy ToProto(const Joy& data);

// Converts 'proto' to Joy.
Joy FromProto(const proto::sensor_msgs::Joy& proto);



}  // namespace sensor_msgs
}  // namespace commsgs
}  // namespace autonomy