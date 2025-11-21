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

#include <gtest/gtest.h>
#include <cmath>

namespace autonomy {
namespace transform {
namespace {

// ============================================================================
// 数学工具函数测试
// ============================================================================

TEST(QuaternionToEulerTest, IdentityQuaternion)
{
    double roll, pitch, yaw;
    QuaternionToEuler(0.0, 0.0, 0.0, 1.0, roll, pitch, yaw);
    
    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST(QuaternionToEulerTest, QuarterRotationZ)
{
    // 绕Z轴旋转90度 (pi/2)
    double qz = std::sin(M_PI / 4.0);  // 45度 sin
    double qw = std::cos(M_PI / 4.0);  // 45度 cos
    
    double roll, pitch, yaw;
    QuaternionToEuler(0.0, 0.0, qz, qw, roll, pitch, yaw);
    
    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_NEAR(yaw, M_PI / 2.0, 1e-6);
}

TEST(QuaternionToEulerTest, QuarterRotationX)
{
    // 绕X轴旋转90度
    double qx = std::sin(M_PI / 4.0);
    double qw = std::cos(M_PI / 4.0);
    
    double roll, pitch, yaw;
    QuaternionToEuler(qx, 0.0, 0.0, qw, roll, pitch, yaw);
    
    EXPECT_NEAR(roll, M_PI / 2.0, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST(QuaternionToEulerTest, QuarterRotationY)
{
    // 绕Y轴旋转90度
    double qy = std::sin(M_PI / 4.0);
    double qw = std::cos(M_PI / 4.0);
    
    double roll, pitch, yaw;
    QuaternionToEuler(0.0, qy, 0.0, qw, roll, pitch, yaw);
    
    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, M_PI / 2.0, 1e-6);
    EXPECT_NEAR(yaw, 0.0, 1e-6);
}

TEST(QuaternionToEulerTest, NegativeRotation)
{
    // 绕Z轴旋转-90度
    double qz = -std::sin(M_PI / 4.0);
    double qw = std::cos(M_PI / 4.0);
    
    double roll, pitch, yaw;
    QuaternionToEuler(0.0, 0.0, qz, qw, roll, pitch, yaw);
    
    EXPECT_NEAR(roll, 0.0, 1e-6);
    EXPECT_NEAR(pitch, 0.0, 1e-6);
    EXPECT_NEAR(yaw, -M_PI / 2.0, 1e-6);
}

TEST(QuaternionToRotationMatrixTest, IdentityQuaternion)
{
    double matrix[3][3];
    QuaternionToRotationMatrix(0.0, 0.0, 0.0, 1.0, matrix);
    
    // 应该是单位矩阵
    EXPECT_NEAR(matrix[0][0], 1.0, 1e-6);
    EXPECT_NEAR(matrix[0][1], 0.0, 1e-6);
    EXPECT_NEAR(matrix[0][2], 0.0, 1e-6);
    
    EXPECT_NEAR(matrix[1][0], 0.0, 1e-6);
    EXPECT_NEAR(matrix[1][1], 1.0, 1e-6);
    EXPECT_NEAR(matrix[1][2], 0.0, 1e-6);
    
    EXPECT_NEAR(matrix[2][0], 0.0, 1e-6);
    EXPECT_NEAR(matrix[2][1], 0.0, 1e-6);
    EXPECT_NEAR(matrix[2][2], 1.0, 1e-6);
}

TEST(QuaternionToRotationMatrixTest, QuarterRotationZ)
{
    // 绕Z轴旋转90度
    double qz = std::sin(M_PI / 4.0);
    double qw = std::cos(M_PI / 4.0);
    
    double matrix[3][3];
    QuaternionToRotationMatrix(0.0, 0.0, qz, qw, matrix);
    
    // 旋转矩阵应该是：
    // [ 0 -1  0]
    // [ 1  0  0]
    // [ 0  0  1]
    EXPECT_NEAR(matrix[0][0], 0.0, 1e-6);
    EXPECT_NEAR(matrix[0][1], -1.0, 1e-6);
    EXPECT_NEAR(matrix[0][2], 0.0, 1e-6);
    
    EXPECT_NEAR(matrix[1][0], 1.0, 1e-6);
    EXPECT_NEAR(matrix[1][1], 0.0, 1e-6);
    EXPECT_NEAR(matrix[1][2], 0.0, 1e-6);
    
    EXPECT_NEAR(matrix[2][0], 0.0, 1e-6);
    EXPECT_NEAR(matrix[2][1], 0.0, 1e-6);
    EXPECT_NEAR(matrix[2][2], 1.0, 1e-6);
}

TEST(QuaternionToRotationMatrixTest, MatrixOrthogonal)
{
    // 测试旋转矩阵的正交性: R * R^T = I
    double qx = 0.1, qy = 0.2, qz = 0.3, qw = std::sqrt(1.0 - qx*qx - qy*qy - qz*qz);
    
    double matrix[3][3];
    QuaternionToRotationMatrix(qx, qy, qz, qw, matrix);
    
    // 计算 R * R^T
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += matrix[i][k] * matrix[j][k];
            }
            if (i == j) {
                EXPECT_NEAR(sum, 1.0, 1e-6);
            } else {
                EXPECT_NEAR(sum, 0.0, 1e-6);
            }
        }
    }
}

TEST(QuaternionToRotationMatrixTest, Determinant)
{
    // 测试行列式应该为1
    double qx = 0.1, qy = 0.2, qz = 0.3, qw = std::sqrt(1.0 - qx*qx - qy*qy - qz*qz);
    
    double matrix[3][3];
    QuaternionToRotationMatrix(qx, qy, qz, qw, matrix);
    
    // 计算行列式（使用Sarrus规则）
    double det = matrix[0][0] * (matrix[1][1] * matrix[2][2] - matrix[1][2] * matrix[2][1])
               - matrix[0][1] * (matrix[1][0] * matrix[2][2] - matrix[1][2] * matrix[2][0])
               + matrix[0][2] * (matrix[1][0] * matrix[2][1] - matrix[1][1] * matrix[2][0]);
    
    EXPECT_NEAR(det, 1.0, 1e-6);
}

// ============================================================================
// TransformQueryOptions 测试
// ============================================================================

TEST(TransformQueryOptionsTest, DefaultValues)
{
    TransformQueryOptions options;
    
    // 检查默认值
    EXPECT_TRUE(options.source_frame.empty());
    EXPECT_TRUE(options.target_frame.empty());
    EXPECT_DOUBLE_EQ(options.timeout, 1.0);
    EXPECT_DOUBLE_EQ(options.wait_time, 0.0);
    EXPECT_FALSE(options.verbose_output);
    EXPECT_FALSE(options.euler_angles);
    EXPECT_FALSE(options.matrix_format);
    EXPECT_FALSE(options.continuous);
    EXPECT_DOUBLE_EQ(options.rate, 1.0);
    EXPECT_FALSE(options.list_frames);
}

TEST(TransformQueryOptionsTest, CustomValues)
{
    TransformQueryOptions options;
    
    options.source_frame = "base_link";
    options.target_frame = "map";
    options.timeout = 2.0;
    options.wait_time = 1.0;
    options.verbose_output = true;
    options.euler_angles = true;
    options.matrix_format = true;
    options.continuous = true;
    options.rate = 10.0;
    options.list_frames = true;
    
    EXPECT_EQ(options.source_frame, "base_link");
    EXPECT_EQ(options.target_frame, "map");
    EXPECT_DOUBLE_EQ(options.timeout, 2.0);
    EXPECT_DOUBLE_EQ(options.wait_time, 1.0);
    EXPECT_TRUE(options.verbose_output);
    EXPECT_TRUE(options.euler_angles);
    EXPECT_TRUE(options.matrix_format);
    EXPECT_TRUE(options.continuous);
    EXPECT_DOUBLE_EQ(options.rate, 10.0);
    EXPECT_TRUE(options.list_frames);
}

// ============================================================================
// TransformQuery 类测试
// ============================================================================

TEST(TransformQueryTest, Construction)
{
    TransformQueryOptions options;
    options.source_frame = "base_link";
    options.target_frame = "map";
    
    // 应该能够成功构造
    EXPECT_NO_THROW({
        TransformQuery query(options);
    });
}

TEST(TransformQueryTest, EmptyFramesReturnError)
{
    TransformQueryOptions options;
    // 不设置 source_frame 和 target_frame
    
    TransformQuery query(options);
    
    // 运行应该返回失败
    int result = query.Run();
    EXPECT_NE(result, EXIT_SUCCESS);
}

TEST(TransformQueryTest, OptionsPreserved)
{
    TransformQueryOptions options;
    options.source_frame = "base_link";
    options.target_frame = "map";
    options.timeout = 5.0;
    options.euler_angles = true;
    options.verbose_output = true;
    
    TransformQuery query(options);
    
    // 选项应该被保存（虽然我们不能直接访问私有成员，但可以通过行为测试）
    // 这里主要测试构造不会抛出异常
    EXPECT_NO_THROW({
        TransformQuery query2(options);
    });
}

// ============================================================================
// 集成测试
// ============================================================================

TEST(TransformQueryIntegrationTest, QuaternionEulerRoundTrip)
{
    // 测试四元数 -> 欧拉角 -> 四元数的往返转换
    // 注意：由于欧拉角的奇异性，这个测试可能在某些角度失败
    
    double original_qx = 0.1;
    double original_qy = 0.2;
    double original_qz = 0.3;
    double original_qw = std::sqrt(1.0 - original_qx*original_qx 
                                      - original_qy*original_qy 
                                      - original_qz*original_qz);
    
    // 转换为欧拉角
    double roll, pitch, yaw;
    QuaternionToEuler(original_qx, original_qy, original_qz, original_qw, 
                      roll, pitch, yaw);
    
    // 从欧拉角转回四元数（这里需要实现逆转换函数，暂时跳过）
    // 这个测试展示了测试的完整性思路
}

TEST(TransformQueryIntegrationTest, MatrixQuaternionConsistency)
{
    // 测试四元数和旋转矩阵表示的一致性
    double qx = 0.1, qy = 0.2, qz = 0.3;
    double qw = std::sqrt(1.0 - qx*qx - qy*qy - qz*qz);
    
    // 获取旋转矩阵
    double matrix[3][3];
    QuaternionToRotationMatrix(qx, qy, qz, qw, matrix);
    
    // 获取欧拉角
    double roll, pitch, yaw;
    QuaternionToEuler(qx, qy, qz, qw, roll, pitch, yaw);
    
    // 从欧拉角手动构建旋转矩阵并比较
    // R = Rz(yaw) * Ry(pitch) * Rx(roll)
    double cr = std::cos(roll), sr = std::sin(roll);
    double cp = std::cos(pitch), sp = std::sin(pitch);
    double cy = std::cos(yaw), sy = std::sin(yaw);
    
    double expected[3][3];
    expected[0][0] = cy * cp;
    expected[0][1] = cy * sp * sr - sy * cr;
    expected[0][2] = cy * sp * cr + sy * sr;
    
    expected[1][0] = sy * cp;
    expected[1][1] = sy * sp * sr + cy * cr;
    expected[1][2] = sy * sp * cr - cy * sr;
    
    expected[2][0] = -sp;
    expected[2][1] = cp * sr;
    expected[2][2] = cp * cr;
    
    // 比较矩阵
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            EXPECT_NEAR(matrix[i][j], expected[i][j], 1e-6) 
                << "Mismatch at [" << i << "][" << j << "]";
        }
    }
}

} // namespace
} // namespace transform
} // namespace autonomy

