/*
 * Copyright 2026 The Openbot Authors
 */

#include "autonomy/orbisview/backend/common/plugins/plugin_host.h"
#include "autonomy/orbisview/backend/common/plugins/registry.h"

#include <gtest/gtest.h>

using autonomy::orbisview::plugins::PluginHost;
using autonomy::orbisview::plugins::PluginInfo;
using autonomy::orbisview::plugins::PluginRegistry;

TEST(PluginRegistryTest, DuplicateIdFails) {
  PluginRegistry reg;
  ASSERT_TRUE(reg.Register({"a", "tool", "A", "1.0", true, "builtin", {}}));
  EXPECT_FALSE(reg.Register({"a", "tool", "A2", "1.0", true, "builtin", {}}));
  const std::string json = reg.ToJson();
  EXPECT_NE(json.find("duplicate plugin id"), std::string::npos);
}

TEST(PluginHostTest, BadPathIsolated) {
  PluginRegistry reg;
  PluginHost host(&reg);
  EXPECT_FALSE(host.Load("/no/such/orbisview_plugin.so"));
  const std::string status = host.StatusJson();
  EXPECT_NE(status.find("dlopen failed"), std::string::npos);
  EXPECT_TRUE(reg.Register({"builtin_ok", "panel", "OK", "1.0", true, "builtin", {}}));
  EXPECT_NE(reg.ToJson().find("builtin_ok"), std::string::npos);
}
