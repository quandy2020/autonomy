# runtime_bootstrap 能力契约

输入：autonomy_user、autonomy_data_dir、autonomy_log_dir、autonomy_build_dir、autonomy_install_dependencies 和 APT 超时。

输出：创建应用专用用户和目录；仅在允许安装依赖时更新 APT 并安装基础工具；不启动服务，不做系统通用加固。

依赖：deployment_defaults；目标机须允许所需的 sudo/APT 操作。
