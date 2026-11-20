# runtime_bootstrap 能力契约

输入：autonomy_user、autonomy_data_dir、autonomy_log_dir、autonomy_build_dir、autonomy_system_install、autonomy_install_dependencies 和 APT 超时。

输出：建立运行账号家目录下的 .autonomy 配置、日志、地图、模型、录制、制品、缓存和状态目录；系统安装时创建专用用户，用户安装时仅创建目录；仅在允许安装依赖时更新 APT 并安装基础工具，不启动服务。

依赖：deployment_defaults；系统安装时目标机须允许所需的 sudo/APT 操作。
