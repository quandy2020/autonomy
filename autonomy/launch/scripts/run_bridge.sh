#!/usr/bin/env bash
# 启动 autonomy bridge 模块（gRPC/MQTT 桥接）
set -e
exec autonomy.bridge.launcher "$@"
