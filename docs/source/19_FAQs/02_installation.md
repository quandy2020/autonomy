# 2. 安装问题

常见症状：apt 冲突、第三方脚本中断、板端空间不足，或 Python/C++ protobuf 版本打架。

官方 `install_deps` 面向 **Ubuntu 22.04**；板端常 **关 BUILD_AUTOVIZ**、降低 ninja 并行。Bridge 需要 gRPC 第三方，最小栈可 `BUILD_GRPC=OFF` 但勿再期望 5005。

## 问答

### Q: apt 失败 / 依赖冲突？

```bash
sudo apt-get -y --fix-broken install
python3 -m install_deps --apt-only
```

非 Ubuntu 对照 `docker/install/` 手工装。

### Q: 第三方脚本装到一半断了？

```bash
python3 -m install_deps --resume-from install_opencv.sh --skip-installed
```

### Q: 找不到 ceres / BehaviorTree / gRPC？

```bash
python3 -m install_deps --thirdparty-only --skip-installed
```

### Q: 板端 apt 慢、空间紧？

见 [02 Installation · 板端](../02_Installation/09_embedded_board.md)：关 `BUILD_AUTOVIZ`、减小并行度、build 放本地盘。

### Q: Python protobuf 版本和 C++ 不一致？

Autolink Python 常要求较老 protobuf（见 [06 Autolink](06_autolink.md)）；`pip` 与 `apt` 勿混装冲突版本。
