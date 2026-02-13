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
 *
 * Basic demo component in autonomy/examples/basic, modeled after
 * autolink/examples/cpp/common_component.
 */

#pragma once

#include <memory>

#include "autolink/component/component.hpp"
#include "proto/examples.pb.h"

using autolink::Component;
using autolink::ComponentBase;
using autolink::examples::proto::Driver;

/**
 * @class CommonComponentSample
 * @brief 简单示例组件：订阅两个 Driver 消息并打印其 msg_id。
 *
 * 在 DAG / Launch 中配置两个输入通道后，框架会调用 Proc()。
 */
class CommonComponentSample : public Component<Driver, Driver>
{
public:
    bool Init() override;

    bool Proc(const std::shared_ptr<Driver>& msg0,
              const std::shared_ptr<Driver>& msg1) override;
};

// 注册组件到 autolink 组件系统
AUTOLINK_REGISTER_COMPONENT(CommonComponentSample)

