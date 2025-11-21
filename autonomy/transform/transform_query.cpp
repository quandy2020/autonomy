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

#include "autonomy/transform/transform_query.hpp"

#include <glog/logging.h>
#include <iostream>
#include <iomanip>
#include <chrono>
#include <thread>
#include <cmath>
#include <vector>

#include "autonomy/common/version.hpp"
#include "autonomy/commsgs/builtin_interfaces.hpp"

namespace autonomy {
namespace transform {

// ============================================================================
// Math utility function implementations
// ============================================================================

void QuaternionToEuler(double qx, double qy, double qz, double qw,
                       double& roll, double& pitch, double& yaw)
{
    // Roll (x-axis rotation)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (y-axis rotation)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1)
        pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
    else
        pitch = std::asin(sinp);

    // Yaw (z-axis rotation)
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    yaw = std::atan2(siny_cosp, cosy_cosp);
}

void QuaternionToRotationMatrix(double qx, double qy, double qz, double qw,
                                 double matrix[3][3])
{
    double xx = qx * qx;
    double xy = qx * qy;
    double xz = qx * qz;
    double xw = qx * qw;
    double yy = qy * qy;
    double yz = qy * qz;
    double yw = qy * qw;
    double zz = qz * qz;
    double zw = qz * qw;

    matrix[0][0] = 1.0 - 2.0 * (yy + zz);
    matrix[0][1] = 2.0 * (xy - zw);
    matrix[0][2] = 2.0 * (xz + yw);
    
    matrix[1][0] = 2.0 * (xy + zw);
    matrix[1][1] = 1.0 - 2.0 * (xx + zz);
    matrix[1][2] = 2.0 * (yz - xw);
    
    matrix[2][0] = 2.0 * (xz - yw);
    matrix[2][1] = 2.0 * (yz + xw);
    matrix[2][2] = 1.0 - 2.0 * (xx + yy);
}

// ============================================================================
// TransformQuery class implementation
// ============================================================================

TransformQuery::TransformQuery(const TransformQueryOptions& options)
    : options_(options)
{
}

int TransformQuery::Run()
{
    // Display version information (FLAGS_verbose is declared in gflags.hpp)
    // Not displaying version here because FLAGS_verbose is not in TFQueryTool's scope
    // If version display is needed, it can be called in main function
    
    // Create TF Buffer (using singleton)
    auto buffer = Buffer::Instance();
    if (!buffer) {
        LOG(ERROR) << "Failed to create TF Buffer";
        return EXIT_FAILURE;
    }
    
    // If only listing all frames
    if (options_.list_frames) {
        ListAvailableFrames(buffer);
        return EXIT_SUCCESS;
    }
    
    // Validate required parameters
    if (options_.source_frame.empty() || options_.target_frame.empty()) {
        LOG(ERROR) << "Both --source_frame and --target_frame are required!";
        LOG(ERROR) << "Example: tf_query --source_frame=base_link --target_frame=map";
        return EXIT_FAILURE;
    }
    
    LOG(INFO) << "TF Query Tool";
    LOG(INFO) << "  Source Frame: " << options_.source_frame;
    LOG(INFO) << "  Target Frame: " << options_.target_frame;
    LOG(INFO) << "  Timeout: " << options_.timeout << " s";
    
    // Wait for some time (if needed)
    if (options_.wait_time > 0.0) {
        LOG(INFO) << "Waiting for " << options_.wait_time << " seconds...";
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>(options_.wait_time * 1000))
        );
    }
    
    // Continuous mode
    if (options_.continuous) {
        LOG(INFO) << "Continuous mode enabled (rate: " << options_.rate << " Hz)";
        LOG(INFO) << "Press Ctrl+C to exit";
        
        auto period = std::chrono::milliseconds(static_cast<int>(1000.0 / options_.rate));
        
        while (true) {
            auto start = std::chrono::steady_clock::now();
            
            QueryAndDisplayTransform(
                buffer,
                options_.source_frame,
                options_.target_frame,
                static_cast<float>(options_.timeout)
            );
            
            auto end = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            auto sleep_time = period - elapsed;
            
            if (sleep_time.count() > 0) {
                std::this_thread::sleep_for(sleep_time);
            }
        }
    } 
    // Single query mode
    else {
        bool success = QueryAndDisplayTransform(
            buffer,
            options_.source_frame,
            options_.target_frame,
            static_cast<float>(options_.timeout)
        );
        
        return success ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    
    return EXIT_SUCCESS;
}

bool TransformQuery::QueryAndDisplayTransform(Buffer* buffer,
                                               const std::string& source_frame,
                                               const std::string& target_frame,
                                               float timeout_sec)
{
    try {
        auto current_time = commsgs::builtin_interfaces::Time::Now();
        
        // Query transform
        auto transform = buffer->lookupTransform(
            target_frame,
            source_frame,
            current_time,
            timeout_sec
        );
        
        // Display transform
        if (options_.matrix_format) {
            PrintTransformMatrix(transform);
        } else if (options_.verbose_output) {
            PrintTransformVerbose(transform);
        } else {
            PrintTransformBasic(transform);
        }
        
        return true;
        
    } catch (const std::exception& e) {
        LOG(ERROR) << "Failed to lookup transform from '" << source_frame 
                   << "' to '" << target_frame << "': " << e.what();
        return false;
    }
}

void TransformQuery::ListAvailableFrames(Buffer* buffer)
{
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              Available Coordinate Frames                       ║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    
    try {
        // Get all frames (requires Buffer to provide corresponding interface, using placeholder implementation)
        std::vector<std::string> frames = {"map", "odom", "base_link", "laser", "camera"};
        
        for (const auto& frame : frames) {
            std::cout << "║  • " << std::left << std::setw(59) << frame << "║\n";
        }
        
    } catch (const std::exception& e) {
        std::cout << "║  Error listing frames: " << std::left << std::setw(39) 
                  << e.what() << "║\n";
    }
    
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
    std::cout << "\n";
    std::cout << "Note: Use 'tf_query <source_frame> <target_frame>' to query transforms\n";
    std::cout << std::endl;
}

void TransformQuery::PrintTransformBasic(const commsgs::geometry_msgs::TransformStamped& transform)
{
    std::cout << "\n";
    std::cout << "╔════════════════════════════════════════════════════════════════╗\n";
    std::cout << "║              Transform: " << std::left << std::setw(36) 
              << (transform.header.frame_id + " → " + transform.child_frame_id) << "║\n";
    std::cout << "╠════════════════════════════════════════════════════════════════╣\n";
    
    // Translation
    std::cout << "║ Translation:                                                   ║\n";
    std::cout << "║   x: " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
              << transform.transform.translation.x << " m                                      ║\n";
    std::cout << "║   y: " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
              << transform.transform.translation.y << " m                                      ║\n";
    std::cout << "║   z: " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
              << transform.transform.translation.z << " m                                      ║\n";
    
    // Rotation (Quaternion)
    std::cout << "║                                                                ║\n";
    std::cout << "║ Rotation (Quaternion):                                         ║\n";
    std::cout << "║   x: " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.x << "                                       ║\n";
    std::cout << "║   y: " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.y << "                                       ║\n";
    std::cout << "║   z: " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.z << "                                       ║\n";
    std::cout << "║   w: " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.w << "                                       ║\n";
    
    // Euler angles (if enabled)
    if (options_.euler_angles) {
        double roll, pitch, yaw;
        QuaternionToEuler(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
            roll, pitch, yaw
        );
        
        std::cout << "║                                                                ║\n";
        std::cout << "║ Rotation (Euler Angles - ZYX):                                 ║\n";
        std::cout << "║   Roll  (X): " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
                  << roll << " rad (" << std::setw(8) << std::setprecision(3) << (roll * 180.0 / M_PI) << "°)       ║\n";
        std::cout << "║   Pitch (Y): " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
                  << pitch << " rad (" << std::setw(8) << std::setprecision(3) << (pitch * 180.0 / M_PI) << "°)       ║\n";
        std::cout << "║   Yaw   (Z): " << std::right << std::setw(10) << std::fixed << std::setprecision(6) 
                  << yaw << " rad (" << std::setw(8) << std::setprecision(3) << (yaw * 180.0 / M_PI) << "°)       ║\n";
    }
    
    std::cout << "╚════════════════════════════════════════════════════════════════╝\n";
    std::cout << std::endl;
}

void TransformQuery::PrintTransformMatrix(const commsgs::geometry_msgs::TransformStamped& transform)
{
    double rot_matrix[3][3];
    QuaternionToRotationMatrix(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
        rot_matrix
    );
    
    std::cout << "\n";
    std::cout << "Transformation Matrix (4x4):\n";
    std::cout << "╔                                                           ╗\n";
    
    for (int i = 0; i < 3; ++i) {
        std::cout << "║ ";
        for (int j = 0; j < 3; ++j) {
            std::cout << std::setw(12) << std::fixed << std::setprecision(6) << rot_matrix[i][j] << " ";
        }
        
        // Translation column
        if (i == 0) {
            std::cout << std::setw(12) << std::fixed << std::setprecision(6) 
                      << transform.transform.translation.x << " ║\n";
        } else if (i == 1) {
            std::cout << std::setw(12) << std::fixed << std::setprecision(6) 
                      << transform.transform.translation.y << " ║\n";
        } else {
            std::cout << std::setw(12) << std::fixed << std::setprecision(6) 
                      << transform.transform.translation.z << " ║\n";
        }
    }
    
    // Last row [0 0 0 1]
    std::cout << "║ " << std::setw(12) << "0.000000" << " "
              << std::setw(12) << "0.000000" << " "
              << std::setw(12) << "0.000000" << " "
              << std::setw(12) << "1.000000" << " ║\n";
    
    std::cout << "╚                                                           ╝\n";
    std::cout << std::endl;
}

void TransformQuery::PrintTransformVerbose(const commsgs::geometry_msgs::TransformStamped& transform)
{
    std::cout << "\n";
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    std::cout << "  Detailed Transform Information\n";
    std::cout << "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    
    std::cout << "\n[Header]\n";
    std::cout << "  Frame ID:       " << transform.header.frame_id << "\n";
    std::cout << "  Child Frame ID: " << transform.child_frame_id << "\n";
    std::cout << "  Timestamp:      " << transform.header.stamp.sec << "." 
              << std::setfill('0') << std::setw(9) << transform.header.stamp.nanosec 
              << std::setfill(' ') << " s\n";
    
    std::cout << "\n[Translation]\n";
    std::cout << "  x: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << transform.transform.translation.x << " m\n";
    std::cout << "  y: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << transform.transform.translation.y << " m\n";
    std::cout << "  z: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << transform.transform.translation.z << " m\n";
    
    double distance = std::sqrt(
        transform.transform.translation.x * transform.transform.translation.x +
        transform.transform.translation.y * transform.transform.translation.y +
        transform.transform.translation.z * transform.transform.translation.z
    );
    std::cout << "  Distance: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << distance << " m\n";
    
    std::cout << "\n[Rotation - Quaternion]\n";
    std::cout << "  x: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.x << "\n";
    std::cout << "  y: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.y << "\n";
    std::cout << "  z: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.z << "\n";
    std::cout << "  w: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << transform.transform.rotation.w << "\n";
    
    double quat_norm = std::sqrt(
        transform.transform.rotation.x * transform.transform.rotation.x +
        transform.transform.rotation.y * transform.transform.rotation.y +
        transform.transform.rotation.z * transform.transform.rotation.z +
        transform.transform.rotation.w * transform.transform.rotation.w
    );
    std::cout << "  Norm: " << std::setw(12) << std::fixed << std::setprecision(6) 
              << quat_norm << " (should be 1.0)\n";
    
    // Euler angles
    double roll, pitch, yaw;
    QuaternionToEuler(
        transform.transform.rotation.x,
        transform.transform.rotation.y,
        transform.transform.rotation.z,
        transform.transform.rotation.w,
        roll, pitch, yaw
    );
    
    std::cout << "\n[Rotation - Euler Angles (ZYX)]\n";
    std::cout << "  Roll  (X): " << std::setw(10) << std::fixed << std::setprecision(6) 
              << roll << " rad  (" << std::setw(8) << std::setprecision(3) 
              << (roll * 180.0 / M_PI) << "°)\n";
    std::cout << "  Pitch (Y): " << std::setw(10) << std::fixed << std::setprecision(6) 
              << pitch << " rad  (" << std::setw(8) << std::setprecision(3) 
              << (pitch * 180.0 / M_PI) << "°)\n";
    std::cout << "  Yaw   (Z): " << std::setw(10) << std::fixed << std::setprecision(6) 
              << yaw << " rad  (" << std::setw(8) << std::setprecision(3) 
              << (yaw * 180.0 / M_PI) << "°)\n";
    
    std::cout << "\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n";
    std::cout << std::endl;
}

} // namespace transform
} // namespace autonomy
