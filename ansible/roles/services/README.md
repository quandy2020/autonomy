# services 能力契约

输入：autonomy_services、autonomy_service_binaries、autonomy_install_prefix、autonomy_run_verify。

输出：安装 unit、按模式启动/重启并验证 active；业务端口取决于组件配置，不在角色里另造默认值。

依赖：deployment_defaults；systemd、可执行文件、配置和运行用户须存在。

元数据中的 Role 依赖为运行前自动加载的依赖；系统前提由场景入口按顺序提供。
