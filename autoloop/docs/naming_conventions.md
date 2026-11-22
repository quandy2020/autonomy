# Naming conventions

- Include：`#include "common/..."`, `"robot/..."`, `"cloud/..."`（`AUTOLOOP_SRC_ROOT`）。
- 目标名：`autoloop_<domain>_<unit>`，例如 `autoloop_robot_ring_buffer`。
- 测试：`<unit>_test.cpp` + `autoloop_test()`。
- Proto package：`autoloop.common`。
- 配置 YAML 蛇形；C++ 类型 PascalCase。
- 身份：`episode_id` = `ep-<utc_ns>-<kind>`；`policy_version` 禁止 `latest`。
- 归档键：`robot_id / episode_id / content_sha256`。
