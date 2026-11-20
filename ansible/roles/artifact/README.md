# artifact 能力契约

输入：autonomy_artifact_path、autonomy_artifact_sha256、autonomy_expected_git_sha、autonomy_install_prefix、autonomy_release_dir、autonomy_system_install、autonomy_artifact_incremental。

输出：校验并发布完整 release，切换安装链接、保留 previous，并报告 autonomy_artifact_activation；不修改运行数据。

依赖：deployment_defaults；Python、tar 和运行用户/依赖由调用方准备。

增量传输复用当前受管 release 中 SHA-256 相同的普通文件。目标在独立暂存目录复制并复核复用文件，随后校验传入差分归档。发布 ID 仍使用完整制品的 SHA-256，旧 release 可继续用于回滚。
