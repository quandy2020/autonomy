# runtime_bootstrap 能力契约

输入：autonomy_user、autonomy_data_dir、autonomy_log_dir、autonomy_build_dir 和 APT 超时。

输出：部署基础工具、应用专用用户和目录；不启动服务，不做系统通用加固。

依赖：deployment_defaults；目标机须允许所需的 sudo/APT 操作。

元数据中的 Role 依赖为运行前自动加载的依赖；系统前提由场景入口按顺序提供。
