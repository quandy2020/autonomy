/*
 * Copyright 2026 The Openbot Authors
 *
 * Stable C ABI for OrbisView native plugins (dlopen hot-load).
 * Plugins MUST only use this header; a failing plugin cannot abort the host.
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define ORBISVIEW_PLUGIN_ABI_VERSION 1

typedef struct OrbisPluginInfo {
  const char* id;
  const char* kind;   /* panel | source | tool */
  const char* title;
  const char* version;
} OrbisPluginInfo;

typedef struct OrbisPluginHostApi {
  int abi_version;
  void (*log_info)(const char* msg);
  void (*log_error)(const char* msg);
} OrbisPluginHostApi;

/** Required export: return 0 on success. */
typedef int (*OrbisPluginRegisterFn)(const OrbisPluginHostApi* host,
                                     OrbisPluginInfo* out_info);

/** Optional export. */
typedef void (*OrbisPluginUnregisterFn)(void);

#ifdef __cplusplus
}  /* extern "C" */
#endif
