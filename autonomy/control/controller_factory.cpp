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

#include "autonomy/control/controller_factory.hpp"

#include <algorithm>
#include <cctype>

#include "autonomy/common/logging.hpp"
#include "autonomy/control/controller/graceful_controller/graceful_controller.hpp"
#include "autonomy/control/controller/nmpc_controller/nmpc_controller.hpp"
#include "autonomy/control/controller/tdmpc_controller/tdmpc_controller.hpp"

namespace autonomy {
namespace control {
namespace {

std::string ToLower(std::string s) {
    for (char& c : s) {
        c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    }
    return s;
}

template <typename ControllerT>
std::unique_ptr<common::ControllerInterface> MakeController(
    const ControllerCreateContext& ctx) {
    auto controller = std::make_unique<ControllerT>();
    controller->Configure(ctx.options, ctx.controller_id, ctx.tf,
                          ctx.costmap_wrapper);
    return controller;
}

}  // namespace

ControllerFactoryRegistry& ControllerFactory::RegistryInstance() {
    static ControllerFactoryRegistry registry;
    return registry;
}

void ControllerFactory::EnsureBuiltinsRegistered() {
    static bool registered = false;
    if (registered) {
        return;
    }
    registered = RegisterBuiltinControllers(RegistryInstance());
}

std::string ControllerFactory::ResolveControllerTypeId(
    const std::string& controller_type) {
    const std::string key = ToLower(controller_type);

    if (key.find("tdmpc") != std::string::npos ||
        key.find("t-mpc") != std::string::npos ||
        key.find("t_mpc") != std::string::npos) {
        return "tdmpc_controller";
    }
    if (key.find("nmpc") != std::string::npos) {
        return "nmpc_controller";
    }
    if (key.find("graceful") != std::string::npos) {
        return "graceful_controller";
    }
    if (key.find("pure_pursuit") != std::string::npos ||
        key.find("purepursuit") != std::string::npos) {
        return "graceful_controller";
    }
    if (key.find("mppi") != std::string::npos) {
        return "graceful_controller";
    }

    return controller_type;
}

bool ControllerFactory::HasController(const std::string& id) {
    EnsureBuiltinsRegistered();
    const std::string resolved = ResolveControllerTypeId(id);
    return RegistryInstance().Contains(resolved);
}

common::ControllerInterface::SharedPtr ControllerFactory::Create(
    const std::string& controller_type, const ControllerCreateContext& context) {
    EnsureBuiltinsRegistered();

    const std::string resolved = ResolveControllerTypeId(controller_type);
    std::unique_ptr<common::ControllerInterface> instance =
        RegistryInstance().CreateObjectOrNull(resolved, context);

    if (!instance) {
        AWARN << "ControllerFactory: unknown type '" << controller_type
              << "' (resolved='" << resolved << "'), falling back to "
              << "graceful_controller";
        instance = RegistryInstance().CreateObjectOrNull("graceful_controller",
                                                         context);
    }

    if (!instance) {
        AERROR << "ControllerFactory: failed to create controller for type '"
               << controller_type << "'";
        return nullptr;
    }

    return common::ControllerInterface::SharedPtr(instance.release());
}

bool RegisterBuiltinControllers(ControllerFactoryRegistry& factory) {
    bool ok = true;
    ok &= factory.Register(
        "graceful_controller",
        [](const ControllerCreateContext& ctx) {
            return MakeController<controller::GracefulController>(ctx);
        });
    ok &= factory.Register(
        "nmpc_controller", [](const ControllerCreateContext& ctx) {
            return MakeController<controller::NmpcController>(ctx);
        });
    ok &= factory.Register(
        "tdmpc_controller", [](const ControllerCreateContext& ctx) {
            return MakeController<controller::TdmpcController>(ctx);
        });

    // Legacy Nav2 / fully-qualified type strings (same creators).
    ok &= factory.Register(
        "autonomy::control::controller::GracefulController",
        [](const ControllerCreateContext& ctx) {
            return MakeController<controller::GracefulController>(ctx);
        });
    ok &= factory.Register(
        "autonomy::control::controller::NmpcController",
        [](const ControllerCreateContext& ctx) {
            return MakeController<controller::NmpcController>(ctx);
        });
    ok &= factory.Register(
        "autonomy::control::controller::TdmpcController",
        [](const ControllerCreateContext& ctx) {
            return MakeController<controller::TdmpcController>(ctx);
        });

    if (!ok) {
        AERROR << "ControllerFactory: failed to register one or more builtin "
                  "controllers";
    }
    return ok;
}

common::ControllerInterface::SharedPtr CreateController(
    const std::string& controller_type, const proto::ControllerOptions& options,
    const std::string& controller_id,
    std::shared_ptr<transform::Buffer> tf,
    std::shared_ptr<map::costmap_2d::Costmap2DWrapper> costmap_wrapper) {
    ControllerCreateContext context{options, controller_id, std::move(tf),
                                    std::move(costmap_wrapper)};
    return ControllerFactory::Create(controller_type, context);
}

}  // namespace control
}  // namespace autonomy
