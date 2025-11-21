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

#include "autonomy/common/param_handler.hpp"
#include <vector>

#include "gtest/gtest.h"

namespace autonomy {
namespace common {

// # test_params.yaml
// str_param: "hello world"
// int_param: 42
// float_param: 3.14
// bool_param: true

class ParamHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        param_handler_ = new ParamHandler("test_params.yaml");
    }
    
    void TearDown() override {
        delete param_handler_;
    }

    ParamHandler* param_handler_;
};

TEST_F(ParamHandlerTest, GetString) 
{
    std::string str_value;  
    EXPECT_TRUE(param_handler_->GetString("str_param", str_value));
    EXPECT_EQ(str_value, "hello world");
}

// 可以添加更多测试用例
TEST_F(ParamHandlerTest, GetInteger) 
{
    int int_value;
    EXPECT_TRUE(param_handler_->GetInteger("int_param", int_value));
    EXPECT_EQ(int_value, 42);
}

TEST_F(ParamHandlerTest, GetDouble) 
{
    double double_value;
    EXPECT_TRUE(param_handler_->GetDouble("float_param", double_value));
    EXPECT_EQ(double_value, 3.14);
}

TEST_F(ParamHandlerTest, GetBoolean) 
{
    bool bool_value;
    EXPECT_TRUE(param_handler_->GetBoolean("bool_param", bool_value));
    EXPECT_EQ(bool_value, true);
}

TEST_F(ParamHandlerTest, InvalidKey) 
{
    std::string str_value;
    EXPECT_FALSE(param_handler_->GetString("nonexistent_param", str_value));
}

}  // namespace common
}  // namespace autonomy
