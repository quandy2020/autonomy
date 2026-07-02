# 2. 快速开始

### 2.1 安装依赖

```bash
python3 scripts/install_dependency.py
```

### 2.2 格式化代码

```bash
# 预览
python3 scripts/format.py --dry-run

# 格式化
python3 scripts/format.py

# CI 检查
python3 scripts/format.py --check
```

### 2.3 离线导航测试

```bash
cmake -G Ninja -B build && ninja -C build
./build/bin/autonomy_nav_test --configuration_directory=config \
  --start_x=1 --start_y=1 --goal_x=5 --goal_y=5
```

> 若 `autonomy_nav_test` 未生成，见 [§4 离线导航测试](04_nav_test.md#44-构建说明)。

### 2.4 Docker 开发环境

```bash
python3 docker/run_autonomy.py -p x86_64
docker exec -it SpaceHero /bin/bash
```

### 2.5 下一步

| 目标 | 文档 |
|------|------|
| 脚本详解 | [§3 开发脚本](03_dev_scripts.md) |
| nav_test 参数 | [§4 离线导航测试](04_nav_test.md) |
| Autolink 调试 | [§6 Autolink 工具](06_autolink_tools.md) |
