# tools/

仓库根目录工具（对齐 Apollo `tools/`）：

| 路径 | 内容 |
|------|------|
| [`package.bzl`](package.bzl) | 域图 + `autonomy_domain_library` / `autonomy_cc_*` |
| [`dependencies.bzl`](dependencies.bzl) | `AUTONOMY_THIRD_PARTY_DEPS` / `AUTONOMY_MIDDLEWARE_DEPS` |
| [`repositories.bzl`](repositories.bzl) | `@autonomy_prefix` module extension |
| [`prefix.BUILD`](prefix.BUILD) / [`prefix_stub.BUILD`](prefix_stub.BUILD) | CMake 前缀 BUILD 模板 |
| [`common.bzl`](common.bzl) | `clean_dep` |
| [`bazel.rc`](bazel.rc) | 默认 Bazel 标志 |
| [`python/`](python/) | 格式化、打包、`generate_version_cpp.py` |

## 命名约定

| 种类 | 风格 | 例 |
|------|------|-----|
| 常量 | `AUTONOMY_<AREA>_…` | `AUTONOMY_DOMAIN_NAMES`、`AUTONOMY_THIRD_PARTY_DEPS` |
| 宏 / 函数 | `autonomy_<verb>_…` | `autonomy_domain_library`、`autonomy_module_copts` |
| 文件 | 全词、无冗余前缀 | `dependencies.bzl`、`package.bzl`、`repositories.bzl` |

## 使用

```bash
./autonomy.sh deps                 # 第三方一览
./autonomy.sh modules
export AUTONOMY_PREFIX="$PWD/build"
./autonomy.sh build -m planning,control
```

域 `BUILD.bazel` 对齐 Apollo：`autonomy_cc_library` + **显式** `srcs`/`hdrs`/`deps`，进程入口用 `autonomy_cc_binary`（如 `autonomy.planning`），运行时资源用 `filegroup(name = "runtime_data")`。范例见 `autonomy/bridge/BUILD.bazel`。
