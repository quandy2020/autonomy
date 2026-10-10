/*
 * Copyright 2026 Automanip contributors duyongquan (quandy2020@126.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * @file backend_register.hpp
 * @brief Static registration macro for arm backends.
 */

#ifndef AUTOMANIP_ARM_BACKEND_REGISTER_HPP_
#define AUTOMANIP_ARM_BACKEND_REGISTER_HPP_

#include "arm/backend_registry.hpp"

/**
 * @brief Register an arm backend at static initialization.
 * @param tag Unique C++ identifier suffix.
 * @param name Canonical backend string.
 * @param factory Creator returning an owning ArmDriver*.
 * @param alias Optional alias (pass "" for none).
 */
#define REGISTER_ARM_BACKEND(tag, name, factory, alias)             \
  namespace {                                                       \
  struct ArmBackendRegistrar_##tag {                                \
    ArmBackendRegistrar_##tag() {                                   \
      ::automanip::arm::ArmBackendRegistry::Instance().Register(    \
          name, factory);                                           \
      if ((alias)[0] != '\0') {                                     \
        ::automanip::arm::ArmBackendRegistry::Instance()            \
            .RegisterAlias(alias, name);                            \
      }                                                             \
    }                                                               \
  };                                                                \
  static ArmBackendRegistrar_##tag g_arm_backend_registrar_##tag;   \
  }

#endif  // AUTOMANIP_ARM_BACKEND_REGISTER_HPP_
