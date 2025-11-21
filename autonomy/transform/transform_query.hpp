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

#include <string>
#include <memory>

#include "autonomy/transform/buffer.hpp"
#include "autonomy/common/math/matrix.hpp"
#include "autonomy/common/math/euler_angles_zxy.hpp"
#include "autonomy/commsgs/geometry_msgs.hpp"

namespace autonomy {
namespace transform {

/**
 * @brief Transform query tool options configuration
 */
struct TransformQueryOptions
{
    // Required parameters
    std::string source_frame;        ///< Source coordinate frame ID
    std::string target_frame;        ///< Target coordinate frame ID
    
    // Query options
    double timeout = 1.0;            ///< Query timeout (seconds)
    double wait_time = 0.0;          ///< Wait time after startup (seconds)
    
    // Display options
    bool verbose_output = false;     ///< Display detailed information
    bool euler_angles = false;       ///< Display Euler angles
    bool matrix_format = false;      ///< Display in matrix format
    
    // Continuous mode options
    bool continuous = false;         ///< Continuous monitoring mode
    double rate = 1.0;               ///< Update frequency (Hz)
    
    // Other options
    bool list_frames = false;        ///< List all available coordinate frames
};

/**
 * @brief Transform query tool class
 * 
 * Provides functionality to query and display TF transforms
 */
class TransformQuery
{
public:
    /**
     * @brief Constructor
     * 
     * @param options Query options
     */
    explicit TransformQuery(const TransformQueryOptions& options);
    
    /**
     * @brief Destructor
     */
    ~TransformQuery() = default;
    
    /**
     * @brief Run TF query tool
     * 
     * @return 0 on success, error code on failure
     */
    int Run();
    
    /**
     * @brief Query and display TF transform once
     * 
     * @param buffer TF buffer
     * @param source_frame Source coordinate frame
     * @param target_frame Target coordinate frame
     * @param timeout_sec Timeout (seconds)
     * @return true on success, false on failure
     */
    bool QueryAndDisplayTransform(Buffer* buffer,
                                   const std::string& source_frame,
                                   const std::string& target_frame,
                                   float timeout_sec);
    
    /**
     * @brief List all available coordinate frames
     * 
     * @param buffer TF buffer
     */
    void ListAvailableFrames(Buffer* buffer);
    
    /**
     * @brief Print transform information (basic format)
     * 
     * @param transform Transform data
     */
    void PrintTransformBasic(const commsgs::geometry_msgs::TransformStamped& transform);
    
    /**
     * @brief Print transform information (matrix format)
     * 
     * @param transform Transform data
     */
    void PrintTransformMatrix(const commsgs::geometry_msgs::TransformStamped& transform);
    
    /**
     * @brief Print detailed transform information
     * 
     * @param transform Transform data
     */
    void PrintTransformVerbose(const commsgs::geometry_msgs::TransformStamped& transform);

private:
    TransformQueryOptions options_;  ///< Query options
};

/**
 * @brief Convert quaternion to Euler angles (ZYX order: Yaw-Pitch-Roll)
 * 
 * @param qx Quaternion x component
 * @param qy Quaternion y component
 * @param qz Quaternion z component
 * @param qw Quaternion w component
 * @param roll Output: rotation angle around X-axis (radians)
 * @param pitch Output: rotation angle around Y-axis (radians)
 * @param yaw Output: rotation angle around Z-axis (radians)
 */
void QuaternionToEuler(double qx, double qy, double qz, double qw,
                       double& roll, double& pitch, double& yaw);

/**
 * @brief Convert quaternion to rotation matrix (3x3)
 * 
 * @param qx Quaternion x component
 * @param qy Quaternion y component
 * @param qz Quaternion z component
 * @param qw Quaternion w component
 * @param matrix Output: 3x3 rotation matrix
 */
void QuaternionToRotationMatrix(double qx, double qy, double qz, double qw,
                                 double matrix[3][3]);

} // namespace transform
} // namespace autonomy
