// /**
//  * Copyright 2025 The Openbot Authors (duyongquan)
//  *
//  * Licensed under the Apache License, Version 2.0 (the "License");
//  * you may not use this file except in compliance with the License.
//  * You may obtain a copy of the License at
//  *
//  *      http://www.apache.org/licenses/LICENSE-2.0
//  *
//  * Unless required by applicable law or agreed to in writing, software
//  * distributed under the License is distributed on an "AS IS" BASIS,
//  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
//  * See the License for the specific language governing permissions and
//  * limitations under the License.
//  */

#pragma once

#include "autolink/class_loader/utility/class_loader_utility.hpp"

#define CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)      \
    namespace {                                                            \
    struct ProxyType##UniqueID {                                           \
        ProxyType##UniqueID() {                                            \
            autolink::class_loader::utility::RegisterClass<Derived, Base>( \
                #Derived, #Base);                                          \
        }                                                                  \
    };                                                                     \
    static ProxyType##UniqueID g_register_class_##UniqueID;                \
    }

#define CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, UniqueID) \
    CLASS_LOADER_REGISTER_CLASS_INTERNAL(Derived, Base, UniqueID)

// register class macro
#define CLASS_LOADER_REGISTER_CLASS(Derived, Base) \
    CLASS_LOADER_REGISTER_CLASS_INTERNAL_1(Derived, Base, __COUNTER__)
