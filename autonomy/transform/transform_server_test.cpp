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

/**
 * =============================================================================
 * TransformServer 测试说明
 * =============================================================================
 * 
 * 本文件测试 Transform Server 的所有核心功能：
 * 
 * 测试用例：
 * 1. BasicInitialization          - 基本初始化
 * 2. InitializeFromYaml           - 从YAML配置初始化
 * 3. InitializeFromConfig         - 从配置结构初始化
 * 4. StartStop                    - 启动和停止
 * 5. GetComponents                - 获取组件
 * 6. DoubleInitialize             - 重复初始化处理
 * 7. StartWithoutInitialize       - 未初始化时启动
 * 8. IsInitializedIsRunning       - 状态查询
 * 
 * =============================================================================
 */

#include "autonomy/transform/transform_server.hpp"

#include <gtest/gtest.h>
#include <fstream>
#include <thread>
#include <chrono>

#include "autonomy/common/logging.hpp"

namespace autonomy {
namespace transform {
namespace {

// 测试辅助类
class TransformServerTestFixture : public ::testing::Test 
{
protected:
    void SetUp() override 
    {
        test_yaml_path_ = "/tmp/test_transform_server.yaml";
        test_lua_path_ = "/tmp/test_transform_server.lua";
    }

    void TearDown() override 
    {
        std::remove(test_yaml_path_.c_str());
        std::remove(test_lua_path_.c_str());
    }

    // 创建简单的测试YAML配置
    void CreateTestYamlConfig() 
    {
        std::ofstream file(test_yaml_path_);
        file << R"(
            static_transforms:
            - name: test_camera
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

            settings:
            publish_rate: 20.0
            tf_prefix: ""
            print_transforms_on_startup: false
            validate_quaternion: true
            )";
        file.close();
    }

    // 创建测试Lua配置
    void CreateTestLuaConfig() 
    {
        std::ofstream file(test_lua_path_);
        file << R"(
            AUTONOMY_TRANSFORM = {
                static_transform_config = ")" << test_yaml_path_ << R"(",
                buffer_cache_time = 10.0,
                default_timeout = 0.01,
                debug = false,
                publish_rate = 10.0,
            }
            return AUTONOMY_TRANSFORM
        )";
        file.close();
    }

    std::string test_yaml_path_;
    std::string test_lua_path_;
};

// 测试1: 基本初始化
TEST_F(TransformServerTestFixture, BasicInitialization)
{
    auto tf_server = std::make_shared<TransformServer>();
    
    // 初始化前状态
    EXPECT_FALSE(tf_server->IsInitialized());
    EXPECT_FALSE(tf_server->IsRunning());
    
    // 创建配置
    TransformServerConfig config;
    config.buffer_cache_time = 10.0;
    config.default_timeout = 0.01f;
    
    // 初始化
    ASSERT_TRUE(tf_server->InitializeFromConfig(config));
    
    // 初始化后状态
    EXPECT_TRUE(tf_server->IsInitialized());
    EXPECT_FALSE(tf_server->IsRunning());
    
    LOG(INFO) << "BasicInitialization: Server initialized successfully";
}

// 测试2: 从YAML配置初始化
TEST_F(TransformServerTestFixture, InitializeFromYaml)
{
    // 先创建测试配置文件
    CreateTestYamlConfig();
    
    auto tf_server = std::make_shared<TransformServer>();
    
    // 从YAML初始化
    ASSERT_TRUE(tf_server->InitializeFromYaml(test_yaml_path_));
    
    EXPECT_TRUE(tf_server->IsInitialized());
    
    // 验证组件已创建
    ASSERT_NE(tf_server->GetBuffer(), nullptr);
    ASSERT_NE(tf_server->GetBroadcaster(), nullptr);
    ASSERT_NE(tf_server->GetStaticTransform(), nullptr);
    
    // 注意：如果静态TF配置文件打开失败，GetTransformCount可能为0
    // 但系统应该仍然可以初始化
    LOG(INFO) << "InitializeFromYaml: Loaded " 
              << tf_server->GetStaticTransform()->GetTransformCount() 
              << " static transforms (file may not be accessible in container)";
}

// 测试3: 从配置结构初始化
TEST_F(TransformServerTestFixture, InitializeFromConfig)
{
    CreateTestYamlConfig();
    
    auto tf_server = std::make_shared<TransformServer>();
    
    TransformServerConfig config;
    config.static_transform_config_path = test_yaml_path_;
    config.buffer_cache_time = 15.0;
    config.default_timeout = 0.05f;
    config.debug = true;
    config.publish_rate = 20.0;
    
    ASSERT_TRUE(tf_server->InitializeFromConfig(config));
    
    EXPECT_TRUE(tf_server->IsInitialized());
    
    LOG(INFO) << "InitializeFromConfig: Configuration loaded successfully";
}

// 测试4: 启动和停止
TEST_F(TransformServerTestFixture, StartStop)
{
    CreateTestYamlConfig();
    
    auto tf_server = std::make_shared<TransformServer>();
    ASSERT_TRUE(tf_server->InitializeFromYaml(test_yaml_path_));
    
    // 启动前
    EXPECT_FALSE(tf_server->IsRunning());
    
    // 启动
    ASSERT_NO_THROW(tf_server->Start());
    EXPECT_TRUE(tf_server->IsRunning());
    
    // 等待一小段时间
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // 停止
    ASSERT_NO_THROW(tf_server->Stop());
    EXPECT_FALSE(tf_server->IsRunning());
    
    // 再次停止应该没问题
    ASSERT_NO_THROW(tf_server->Stop());
    
    LOG(INFO) << "StartStop: Server lifecycle managed correctly";
}

// 测试5: 获取组件
TEST_F(TransformServerTestFixture, GetComponents)
{
    auto tf_server = std::make_shared<TransformServer>();
    
    // 初始化前组件也应该存在
    ASSERT_NE(tf_server->GetBuffer(), nullptr);
    ASSERT_NE(tf_server->GetBroadcaster(), nullptr);
    ASSERT_NE(tf_server->GetStaticTransform(), nullptr);
    
    TransformServerConfig config;
    ASSERT_TRUE(tf_server->InitializeFromConfig(config));
    
    // 初始化后组件仍然有效
    EXPECT_NE(tf_server->GetBuffer(), nullptr);
    EXPECT_NE(tf_server->GetBroadcaster(), nullptr);
    EXPECT_NE(tf_server->GetStaticTransform(), nullptr);
    
    LOG(INFO) << "GetComponents: All components accessible";
}

// 测试6: 重复初始化
TEST_F(TransformServerTestFixture, DoubleInitialize)
{
    auto tf_server = std::make_shared<TransformServer>();
    
    TransformServerConfig config;
    
    // 第一次初始化
    ASSERT_TRUE(tf_server->InitializeFromConfig(config));
    EXPECT_TRUE(tf_server->IsInitialized());
    
    // 第二次初始化（应该返回true但有警告）
    EXPECT_TRUE(tf_server->InitializeFromConfig(config));
    EXPECT_TRUE(tf_server->IsInitialized());
    
    LOG(INFO) << "DoubleInitialize: Duplicate initialization handled correctly";
}

// 测试7: 未初始化时启动
TEST_F(TransformServerTestFixture, StartWithoutInitialize)
{
    auto tf_server = std::make_shared<TransformServer>();
    
    // 未初始化就尝试启动
    ASSERT_NO_THROW(tf_server->Start());
    
    // 应该仍然未运行
    EXPECT_FALSE(tf_server->IsRunning());
    
    LOG(INFO) << "StartWithoutInitialize: Start without init handled gracefully";
}

// 测试8: 状态查询
TEST_F(TransformServerTestFixture, IsInitializedIsRunning)
{
    CreateTestYamlConfig();
    
    auto tf_server = std::make_shared<TransformServer>();
    
    // 初始状态
    EXPECT_FALSE(tf_server->IsInitialized());
    EXPECT_FALSE(tf_server->IsRunning());
    
    // 初始化后
    ASSERT_TRUE(tf_server->InitializeFromYaml(test_yaml_path_));
    EXPECT_TRUE(tf_server->IsInitialized());
    EXPECT_FALSE(tf_server->IsRunning());
    
    // 启动后
    tf_server->Start();
    EXPECT_TRUE(tf_server->IsInitialized());
    EXPECT_TRUE(tf_server->IsRunning());
    
    // 停止后
    tf_server->Stop();
    EXPECT_TRUE(tf_server->IsInitialized());
    EXPECT_FALSE(tf_server->IsRunning());
    
    LOG(INFO) << "IsInitializedIsRunning: State queries work correctly";
}

// 测试9: 生命周期管理
TEST_F(TransformServerTestFixture, LifecycleManagement)
{
    CreateTestYamlConfig();
    
    {
        auto tf_server = std::make_shared<TransformServer>();
        ASSERT_TRUE(tf_server->InitializeFromYaml(test_yaml_path_));
        tf_server->Start();
        
        // 对象销毁时应该自动停止
    }
    
    // 如果没有崩溃，说明析构函数正常工作
    SUCCEED();
    
    LOG(INFO) << "LifecycleManagement: Destructor handled cleanup correctly";
}

// 测试10: TF查询接口存在性
TEST_F(TransformServerTestFixture, QueryInterfaceExists)
{
    auto tf_server = std::make_shared<TransformServer>();
    
    TransformServerConfig config;
    ASSERT_TRUE(tf_server->InitializeFromConfig(config));
    
    // 这些方法应该存在并可调用（即使可能抛出异常因为没有TF数据）
    commsgs::builtin_interfaces::Time test_time;
    test_time.sec = 0;
    test_time.nanosec = 0;
    
    std::string errstr;
    
    // CanTransform 应该可以调用
    ASSERT_NO_THROW({
        tf_server->CanTransform("map", "base_link", test_time, 0.01f, &errstr);
    });
    
    LOG(INFO) << "QueryInterfaceExists: Query interfaces are callable";
}

}  // namespace
}  // namespace transform
}  // namespace autonomy

