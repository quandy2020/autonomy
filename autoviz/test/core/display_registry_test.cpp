/******************************************************************************
 * Copyright 2026 The Openbot Authors (duyongquan)
 *****************************************************************************/

/**
 * @file display_registry_test.cpp
 * @brief Factory regression: every registered display type must build.
 *
 * Catches a plugin being listed in a catalog but missing from
 * DisplayRegistry::registerBuiltinTypes(), or a default config that the
 * factory cannot instantiate.
 */

#include <algorithm>
#include <cstdio>
#include <string>
#include <vector>

#include <QApplication>

#include "autoviz/common/display_catalog.hpp"
#include "autoviz/common/display_registry.hpp"
#include "autoviz/plugins/displays/display.hpp"

namespace {

int g_failures = 0;

void Check(bool condition, const std::string& what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++g_failures;
  }
}

}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);

  auto& registry = autoviz::common::DisplayRegistry::instance();
  const std::vector<std::string> types = registry.supportedTypes();
  std::printf("registered display types: %zu\n", types.size());
  Check(types.size() >= 40, "registry exposes the built-in display set");

  for (const std::string& type : types) {
    const autoviz::common::DisplayConfig config =
        registry.defaultForType(type);
    Check(config.type == type, "default config keeps the type id: " + type);

    auto display = registry.create(config);
    Check(display != nullptr, "factory creates: " + type);
    if (display == nullptr) {
      continue;
    }
    // "PointCloud" is a deliberate alias of PointCloud2 (legacy sessions).
    const std::string reported = display->typeId();
    Check(!reported.empty(), "created display reports a type: " + type);
    if (type == "PointCloud") {
      Check(reported == "PointCloud2",
            "PointCloud alias maps to PointCloud2");
    } else {
      Check(reported == type, "created display reports its type: " + type);
    }
    // Defaults must be materialised so the property editor has values.
    for (const auto& spec : display->propertySpecs()) {
      const bool present =
          display->properties().find(spec.key) != display->properties().end();
      Check(present, "default property present: " + type + "." + spec.key);
    }
  }

  // Every user-visible catalog entry must resolve through the factory.
  const std::vector<autoviz::common::DisplayTypeInfo> catalog =
      autoviz::common::DisplayCatalog::allTypes();
  std::printf("catalog display types: %zu\n", catalog.size());
  for (const auto& info : catalog) {
    const bool registered =
        std::find(types.begin(), types.end(), info.type) != types.end();
    Check(registered, "catalog type is registered: " + info.type);
  }

  if (g_failures != 0) {
    std::fprintf(stderr, "%d display registry test(s) failed\n", g_failures);
    return 1;
  }
  std::printf("display registry tests passed\n");
  return 0;
}

