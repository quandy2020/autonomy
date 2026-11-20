# preflight 能力契约

输入：autonomy_supported_ubuntu_versions、autonomy_supported_architectures、autonomy_min_free_disk_mb。

输出：只读检查 OS、架构和可用磁盘；失败阻断发布。

依赖：deployment_defaults；主机事实。
