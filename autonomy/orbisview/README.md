# OrbisView

CivetWeb HTTP/WS 可视化 HMI（companion，与 Foxglove / autoviz 并存）。

构建依赖 **Autolink** + **automsgs**；默认接 **live Autolink**（autosim 等），可用 `--mock=true` 离线演示。

## 布局

```text
orbisview/
├── backend/          # C++ 服务
├── frontend/         # Vite + React（CMakeLists.txt + npm）
├── script/           # install_npm / run_backend / run_frontend
├── conf/             # gflags（orbisview.conf）
├── launch/           # orbisview.launch（autosim + orbisview）
├── proto/            # schema 文档（*.proto，不做 C++ codegen）
├── thirdparty/       # civetweb
└── main.cpp
```

## 构建

在 autonomy 工程根目录（需 `-DBUILD_ORBISVIEW=ON`，默认一般已开；前端需 Node/npm ≥18）：

```bash
# 容器/机器无 npm 时先装（Debian/Ubuntu 用 NodeSource）
bash src/autonomy/autonomy/orbisview/script/install_npm.sh

cmake -S src/autonomy -B build/autonomy
# 默认一并 npm ci + npm run build → frontend/dist（目标 orbisview_frontend）
cmake --build build/autonomy --target autonomy.orbisview -j
# 跳过前端：cmake -DORBISVIEW_BUILD_FRONTEND=OFF ...
```

安装（静态页进 `share/autonomy/orbisview/www`）：

```bash
cmake --install build/autonomy
```

## 运行

```bash
# 推荐：autosim + orbisview（live channels；需 habitat-sim + frontend/dist）
export PATH=$PWD/build/autonomy/bin:$PATH
export LD_LIBRARY_PATH=$PWD/build/autonomy/lib:${LD_LIBRARY_PATH:-}
autolink launch start src/autonomy/autonomy/orbisview/launch/orbisview.launch
# 浏览器: http://127.0.0.1:8766/  WS: ws://127.0.0.1:8766/ws

# 仅 orbisview（autosim 已在跑）
./build/autonomy/bin/autonomy.orbisview \
  --flagfile=src/autonomy/autonomy/orbisview/conf/orbisview.conf \
  --document_root=src/autonomy/autonomy/orbisview/frontend/dist

# 离线 mock（不启 autosim）
./build/autonomy/bin/autonomy.orbisview --mock=true --autolink=false --port=8766 \
  --document_root=src/autonomy/autonomy/orbisview/frontend/dist

# 开发：Vite 热更新（可选）
bash src/autonomy/autonomy/orbisview/script/run_frontend.sh
```

Live 模式下 UI 自动订阅 autosim 通道（`/odom` `/scan` `/map` `/tf` `/camera/*` 等）。

## 常用 flags

| flag | 说明 |
|------|------|
| `--host` / `--port` | 默认 `0.0.0.0:8766`（conf） |
| `--mock` | 默认 `false`（live）；离线 UI 设 `true` |
| `--autolink` | 默认 `true`；订阅 Autolink 拓扑上的通道 |
| `--document_root` | 静态前端根目录（`frontend/dist` 或 `share/.../www`） |
| `--plugin_dir` | 可选 native 插件目录 |
| `--cmd_vel_channel` | 遥控发布通道，默认 `/cmd_vel` |

## 测试

```bash
cmake --build build/autonomy --target autonomy.orbisview -j
# -DBUILD_TEST=ON 时会注册 orbisview.* 单元测试
cd src/autonomy/autonomy/orbisview/frontend && npm test
```

设计说明：[`docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md`](../../docs/superpowers/specs/2026-09-13-orbisview-layered-migration-design.md)
