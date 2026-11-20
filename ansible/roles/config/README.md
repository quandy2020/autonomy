# config 能力契约

输入：autonomy_config_dir、autonomy_config_components、autonomy_config_source_dir、autonomy_config_from_artifact、autonomy_system_install。

输出：同步组件配置并设置 0640/0750 权限；报告 autonomy_config_changed，不删除机器人专有文件。

依赖：deployment_defaults；运行用户和已安装制品/源码须存在。
