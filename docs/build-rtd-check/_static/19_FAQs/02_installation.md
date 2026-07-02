# 2. 安装问题

### Q: `apt-get install` 失败或包冲突？

```bash
sudo apt-get -y --fix-broken install
python3 scripts/install_dependency.py --apt-only
```

详见 [02 Installation · 依赖](../02_Installation/04_dependencies.md)。

### Q: `install_opencv.sh` 等第三方脚本中断？

使用 `--resume-from` 从中断处继续：

```bash
python3 scripts/install_dependency.py --resume-from install_opencv.sh
```

### Q: `libceres.so not found`？

第三方库未装全：

```bash
python3 scripts/install_dependency.py --thirdparty-only
```

### Q: `behaviortree_cpp` 找不到？

确认 `docker/install/install_behaviortree_cpp.sh` 执行成功。

### Q: 非 Ubuntu 系统能用 install_dependency.py 吗？

脚本面向 Ubuntu 22.04。其他发行版需手动对照 `APT_PACKAGES` 与 `docker/install/` 脚本安装等效包。

### Q: Habitat-Sim 安装内存不足？

```bash
python3 setup.py build_ext --parallel 1 install --headless --no-update-submodules
```

详见 [02 Installation · 故障排查 §8.6](../02_Installation/08_troubleshooting.md#86-habitat-sim可选)。
