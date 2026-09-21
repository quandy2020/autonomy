# 2. 安装问题

### Q: `apt-get install` 失败或包冲突？

```bash
sudo apt-get -y --fix-broken install
python3 scripts/install_dependencies.py --apt-only
```

详见 [02 Installation · 依赖](../02_Installation/04_dependencies.md)。

### Q: `install_opencv.sh` 等第三方脚本中断？

```bash
python3 scripts/install_dependencies.py --resume-from install_opencv.sh --skip-installed
```

### Q: `libceres.so` / OSQP / BehaviorTree 找不到？

```bash
python3 scripts/install_dependencies.py --thirdparty-only --skip-installed
# 或单库：
bash docker/install/install_ceres_solver.sh
bash docker/install/install_osqp.sh
bash docker/install/install_behaviortree_cpp.sh
```

确认产物在 `/usr/local`。

### Q: 非 Ubuntu 能用依赖脚本吗？

脚本面向 Ubuntu 22.04。其他发行版需对照 APT 列表与 `docker/install/` 自行安装。

### Q: 板端 apt 无法解析 / 编译极慢？

见 [§9 嵌入式板端](../02_Installation/09_embedded_board.md) 与 [§8 故障排查](../02_Installation/08_troubleshooting.md)。
