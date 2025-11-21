/**
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

#include "autolink/autolink.hpp"
#include "autolink/message/raw_message.hpp"
#include "autolink/tools/autolink_monitor/general_message_base.hpp"

class Screen;

class GeneralMessage : public GeneralMessageBase
{
public:
    GeneralMessage(GeneralMessageBase* parent,
                   const google::protobuf::Message* msg,
                   const google::protobuf::Reflection* reflection,
                   const google::protobuf::FieldDescriptor* field);

    ~GeneralMessage() {
        field_ = nullptr;
        message_ptr_ = nullptr;
        reflection_ptr_ = nullptr;
    }

    int Render(const Screen* s, int key) override;

private:
    GeneralMessage(const GeneralMessage&) = delete;
    GeneralMessage& operator=(const GeneralMessage&) = delete;

    int item_index_;
    bool is_folded_;
    const google::protobuf::FieldDescriptor* field_;
    const google::protobuf::Message* message_ptr_;
    const google::protobuf::Reflection* reflection_ptr_;
};
