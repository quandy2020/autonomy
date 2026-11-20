# Autonomy 部署

修改目标机器与编译配置后，在本目录执行：

```bash
./deploy.sh release robots -l robot01 -K
```

自动准备工具、编译、打包、校验、部署和检查服务。配置位置及其他场景见 [使用文档](docs/README.md)。

ARM 目标可在控制机交叉编译并分发。配置 Profile 后，执行 `./deploy.sh cross-build` 生成制品，或执行 `./deploy.sh cross-release robots -l robot01 -K` 编译并部署。见 [交叉编译发布](docs/cross-release.md)。

按需查阅：[本机/远程编译](docs/getting-started.md) · [日常操作与回滚](docs/deployment.md) · [CI/CD](docs/ci-cd.md) · [排障](docs/troubleshooting.md)。设计说明见 [DESIGN.md](DESIGN.md)。
