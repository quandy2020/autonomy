# bolza/thirdparty

`libbolza` 自带的第三方钉扎（git submodule）。**不依赖** autonomy 库代码。

| 子模块 | 路径 | 上游 | 钉扎说明 |
|:---|:---|:---|:---|
| `bolza-hpipm` | `hpipm/` | [giaf/hpipm](https://github.com/giaf/hpipm) | 与 OCS2 `hpipm_catkin` 同 commit |
| `bolza-coal` | `coal/` | [coal-library/coal](https://github.com/coal-library/coal) (`devel`) | hpp-fcl 继任；含嵌套 `cmake`（jrl-cmakemodules） |

**BLASFEO** 不再作为子模块：`cmake/bolza_hpipm.cmake` 在配置期用 `FetchContent` 拉取 [giaf/blasfeo](https://github.com/giaf/blasfeo)（需网络）。

## 初始化

在含 `.gitmodules` 的仓库根执行：

```bash
git submodule update --init --recursive \
  automanip/bolza/thirdparty/hpipm \
  automanip/bolza/thirdparty/coal
```

或一次性：

```bash
git submodule update --init --recursive
```

## CMake

- `cmake/bolza_hpipm.cmake`：BLASFEO = `FetchContent`；HPIPM **优先** `thirdparty/hpipm`，缺失时再 `FetchContent`。
- `coal`：`cmake/modules/FindCoal.cmake`（CONFIG / pkg-config / 库探测）；树内 submodule 便于 API 对齐（`COAL_BACKWARD_COMPATIBILITY_WITH_HPP_FCL`）。

## 更新钉扎

```bash
cd automanip/bolza/thirdparty/<name>
git fetch
git checkout <new-commit>
cd "$(git rev-parse --show-toplevel)"
git add automanip/bolza/thirdparty/<name>
# 再按需提交
```
