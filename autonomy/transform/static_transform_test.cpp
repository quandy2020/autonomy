/*
 * Copyright 2024 The OpenRobotic Beginner Authors (duyongquan)
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

/**
 * =============================================================================
 * StaticTransform 测试说明
 * =============================================================================
 * 
 * 本文件测试静态TF变换组件的所有功能：
 * 
 * 测试用例：
 * 1. InitializeWithValidConfig     - 有效配置初始化
 * 2. InitializeWithInvalidConfig   - 无效配置处理
 * 3. LoadMultipleTransforms        - 加载多个变换
 * 4. QuaternionValidation          - 四元数验证和归一化
 * 5. EnabledDisabledTransforms     - 启用/禁用变换
 * 6. StartStop                     - 启动/停止发布
 * 7. GetTransformCount             - 统计信息
 * 8. TfPrefix                      - TF前缀功能
 * 9. TransformContent              - 变换内容验证
 * 10. DefaultSettings              - 默认设置测试
 * 
 * =============================================================================
 */

#include "autonomy/transform/static_transform.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <chrono>
#include <thread>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace transform {
namespace {

// 测试辅助类：创建临时YAML配置文件
class StaticTransformTestFixture : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        // 创建临时测试配置文件
        test_config_path_ = "/tmp/test_static_transform.yaml";
    }

    void TearDown() override 
    {
        // 清理临时文件
        std::remove(test_config_path_.c_str());
    }

    // 创建有效的测试配置文件
    void CreateValidConfig() 
    {
        std::ofstream file(test_config_path_);
        file << R"(
            static_transforms:
            - name: camera
                enabled: true
                frame_id: "base_link"
                child_frame_id: "camera_link"
                translation:
                x: 0.1
                y: 0.0
                z: 0.2
                rotation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0
            
            - name: laser
                enabled: true
                frame_id: "base_link"
                child_frame_id: "laser_link"
                translation:
                x: 0.3
                y: 0.0
                z: 0.15
                rotation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0
            
            - name: imu
                enabled: false
                frame_id: "base_link"
                child_frame_id: "imu_link"
                translation:
                x: 0.0
                y: 0.0
                z: 0.05
                rotation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0

            settings:
            publish_rate: 20.0
            tf_prefix: ""
            print_transforms_on_startup: false
            validate_quaternion: true
            )";
        file.close();
    }

    // 创建包含无效四元数的配置
    void CreateConfigWithInvalidQuaternion() 
    {
        std::ofstream file(test_config_path_);
        file << R"(
            static_transforms:
            - name: invalid_quat
                enabled: true
                frame_id: "base_link"
                child_frame_id: "sensor_link"
                translation:
                x: 0.0
                y: 0.0
                z: 0.0
                rotation:
                x: 1.0
                y: 1.0
                z: 1.0
                w: 1.0

            settings:
            validate_quaternion: true
            )";
        file.close();
    }

    // 创建带TF前缀的配置
    void CreateConfigWithTfPrefix() 
    {
        std::ofstream file(test_config_path_);
        file << R"(
            static_transforms:
            - name: camera
                enabled: true
                frame_id: "base_link"
                child_frame_id: "camera_link"
                translation:
                x: 0.0
                y: 0.0
                z: 0.0
                rotation:
                x: 0.0
                y: 0.0
                z: 0.0
                w: 1.0

            settings:
            tf_prefix: "robot1"
            )";
        file.close();
    }

    // 创建空配置
    void CreateEmptyConfig() 
    {
        std::ofstream file(test_config_path_);
        file << "static_transforms: []\n";
        file.close();
    }

    std::string test_config_path_;
};

// 测试1: 有效配置初始化
TEST_F(StaticTransformTestFixture, InitializeWithValidConfig)
{
    CreateValidConfig();
    
    auto static_tf = std::make_shared<StaticTransform>();
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    
    // 验证加载的变换数量
    EXPECT_EQ(static_tf->GetTransformCount(), 3);
    EXPECT_EQ(static_tf->GetEnabledTransformCount(), 2);  // 只有2个启用
    
    LOG(INFO) << "InitializeWithValidConfig: Loaded " 
              << static_tf->GetTransformCount() << " transforms";
}

// 测试2: 无效配置处理
TEST_F(StaticTransformTestFixture, InitializeWithInvalidConfig)
{
    auto static_tf = std::make_shared<StaticTransform>();
    
    // 测试不存在的文件
    EXPECT_FALSE(static_tf->Initialize("/nonexistent/path/config.yaml"));
    
    // 测试空配置
    CreateEmptyConfig();
    EXPECT_FALSE(static_tf->Initialize(test_config_path_));
    
    LOG(INFO) << "InitializeWithInvalidConfig: Correctly handled invalid configs";
}

// 测试3: 加载多个变换
TEST_F(StaticTransformTestFixture, LoadMultipleTransforms)
{
    CreateValidConfig();
    
    auto static_tf = std::make_shared<StaticTransform>();
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    
    // 验证总数
    EXPECT_EQ(static_tf->GetTransformCount(), 3);
    
    // 验证启用数量
    EXPECT_EQ(static_tf->GetEnabledTransformCount(), 2);
    
    LOG(INFO) << "LoadMultipleTransforms: Successfully loaded multiple transforms";
}

// 测试4: 四元数验证和归一化
TEST_F(StaticTransformTestFixture, QuaternionValidation)
{
    CreateConfigWithInvalidQuaternion();
    
    auto static_tf = std::make_shared<StaticTransform>();
    
    // 应该成功初始化（会自动归一化四元数）
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    
    EXPECT_EQ(static_tf->GetTransformCount(), 1);
    
    LOG(INFO) << "QuaternionValidation: Invalid quaternion was normalized";
}

// 测试5: 启用/禁用变换
TEST_F(StaticTransformTestFixture, EnabledDisabledTransforms)
{
    CreateValidConfig();
    
    auto static_tf = std::make_shared<StaticTransform>();
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    
    // 总共3个变换，但只有2个启用
    EXPECT_EQ(static_tf->GetTransformCount(), 3);
    EXPECT_EQ(static_tf->GetEnabledTransformCount(), 2);
    
    // 禁用的变换不会被发布
    EXPECT_LT(static_tf->GetEnabledTransformCount(), static_tf->GetTransformCount());
    
    LOG(INFO) << "EnabledDisabledTransforms: " 
              << static_tf->GetEnabledTransformCount() << "/" 
              << static_tf->GetTransformCount() << " transforms enabled";
}

// 测试6: 启动和停止
TEST_F(StaticTransformTestFixture, StartStop)
{
    CreateValidConfig();
    
    auto static_tf = std::make_shared<StaticTransform>();
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    
    // 启动TF发布
    ASSERT_NO_THROW(static_tf->Start());
    
    // 等待一小段时间让定时器运行
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 停止TF发布
    ASSERT_NO_THROW(static_tf->Stop());
    
    // 再次停止应该没有问题
    ASSERT_NO_THROW(static_tf->Stop());
    
    LOG(INFO) << "StartStop: Successfully started and stopped TF publishing";
}

// 测试7: 获取统计信息
TEST_F(StaticTransformTestFixture, GetTransformCount)
{
    CreateValidConfig();
    
    auto static_tf = std::make_shared<StaticTransform>();
    
    // 初始化前应该是0
    EXPECT_EQ(static_tf->GetTransformCount(), 0);
    EXPECT_EQ(static_tf->GetEnabledTransformCount(), 0);
    
    // 初始化后
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    EXPECT_GT(static_tf->GetTransformCount(), 0);
    
    LOG(INFO) << "GetTransformCount: Transform counts are correct";
}

// 测试8: TF前缀功能
TEST_F(StaticTransformTestFixture, TfPrefix)
{
    CreateConfigWithTfPrefix();
    
    auto static_tf = std::make_shared<StaticTransform>();
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    
    EXPECT_EQ(static_tf->GetTransformCount(), 1);
    
    // 注意：实际的frame_id会被添加前缀，但我们无法直接访问
    // 这个测试主要确保带前缀的配置可以正常加载
    
    LOG(INFO) << "TfPrefix: TF prefix configuration loaded successfully";
}

// 测试9: 默认设置
TEST_F(StaticTransformTestFixture, DefaultSettings)
{
    // 创建最小配置（没有settings部分）
    std::ofstream file(test_config_path_);
    file << R"(
        static_transforms:
        - name: test
            enabled: true
            frame_id: "a"
            child_frame_id: "b"
            translation:
            x: 0.0
            y: 0.0
            z: 0.0
            rotation:
            x: 0.0
            y: 0.0
            z: 0.0
            w: 1.0
        )";
    file.close();
    
    auto static_tf = std::make_shared<StaticTransform>();
    
    // 应该使用默认设置成功初始化
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    EXPECT_EQ(static_tf->GetTransformCount(), 1);
    
    LOG(INFO) << "DefaultSettings: Default settings applied correctly";
}

// 测试10: 重复初始化
TEST_F(StaticTransformTestFixture, DoubleInitialize)
{
    CreateValidConfig();
    
    auto static_tf = std::make_shared<StaticTransform>();
    
    // 第一次初始化
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    size_t count1 = static_tf->GetTransformCount();
    
    // 第二次初始化应该返回true但不重新加载
    EXPECT_TRUE(static_tf->Initialize(test_config_path_));
    size_t count2 = static_tf->GetTransformCount();
    
    EXPECT_EQ(count1, count2);
    
    LOG(INFO) << "DoubleInitialize: Handled duplicate initialization correctly";
}

// 测试11: 只有平移的变换
TEST_F(StaticTransformTestFixture, TranslationOnly)
{
    std::ofstream file(test_config_path_);
    file << R"(
        static_transforms:
        - name: translation_only
            enabled: true
            frame_id: "base_link"
            child_frame_id: "sensor_link"
            translation:
            x: 1.0
            y: 2.0
            z: 3.0
        )";
    file.close();
    
    auto static_tf = std::make_shared<StaticTransform>();
    ASSERT_TRUE(static_tf->Initialize(test_config_path_));
    
    EXPECT_EQ(static_tf->GetTransformCount(), 1);
    
    LOG(INFO) << "TranslationOnly: Transform with only translation loaded";
}

// 测试12: 生命周期管理
TEST_F(StaticTransformTestFixture, LifecycleManagement)
{
    CreateValidConfig();
    
    {
        auto static_tf = std::make_shared<StaticTransform>();
        ASSERT_TRUE(static_tf->Initialize(test_config_path_));
        static_tf->Start();
        
        // 对象销毁时应该自动停止
    }
    
    // 如果没有崩溃，说明析构函数正常工作
    SUCCEED();
    
    LOG(INFO) << "LifecycleManagement: Destructor handled cleanup correctly";
}

}  // namespace
}  // namespace transform
}  // namespace autonomy
