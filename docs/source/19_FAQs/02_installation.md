# 2. 安装问题

### Q: apt 失败？

```bash
sudo apt-get -y --fix-broken install
python3 -m install_deps --apt-only
```

### Q: 第三方脚本中断？

```bash
python3 -m install_deps --resume-from install_opencv.sh --skip-installed
```

### Q: 找不到 ceres / BehaviorTree？

```bash
python3 -m install_deps --thirdparty-only --skip-installed
```

### Q: 非 Ubuntu？

脚本面向 Ubuntu 22.04；其它发行版对照 `docker/install/`。

### Q: 板端 apt/编译慢？

见 [02 · 板端](../02_Installation/09_embedded_board.md)。
