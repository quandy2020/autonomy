# services 能力契约

输入：autonomy_services、autonomy_install_prefix、autonomy_play_mode、autonomy_run_verify；可接收 autonomy_artifact_activation.changed、autonomy_cmake_install.changed、autonomy_config_changed 触发重启。

输出：安装 unit，按变更启动/重启所选服务并验证 active，停止未选择的服务；业务端口由组件配置决定。

依赖：deployment_defaults；systemd、可执行文件、配置和运行用户须存在。
