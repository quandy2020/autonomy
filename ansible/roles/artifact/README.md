# artifact 能力契约

输入：autonomy_artifact_path、autonomy_artifact_sha256、autonomy_expected_git_sha、autonomy_install_prefix、autonomy_release_dir、autonomy_system_install。

输出：校验并发布完整 release，切换安装链接、保留 previous，并报告 autonomy_artifact_activation；不修改运行数据。

依赖：deployment_defaults；Python、tar 和运行用户/依赖由调用方准备。
