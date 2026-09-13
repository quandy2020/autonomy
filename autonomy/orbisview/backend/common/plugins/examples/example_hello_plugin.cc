/*
 * Copyright 2026 The Openbot Authors
 *
 * Example OrbisView native plugin (hot-load via dlopen).
 */

#include "autonomy/orbisview/backend/common/plugins/orbisview_plugin_abi.h"

extern "C" {

int orbisview_plugin_register(const OrbisPluginHostApi* host,
                              OrbisPluginInfo* out_info) {
  if (!host || !out_info) return -1;
  if (host->abi_version != ORBISVIEW_PLUGIN_ABI_VERSION) {
    if (host->log_error) {
      host->log_error("example_hello: ABI version mismatch");
    }
    return -2;
  }
  out_info->id = "example_hello";
  out_info->kind = "tool";
  out_info->title = "Example Hello Plugin";
  out_info->version = "1.0.0";
  if (host->log_info) {
    host->log_info("example_hello: registered");
  }
  return 0;
}

void orbisview_plugin_unregister(void) {
  // no-op
}

}  // extern "C"
