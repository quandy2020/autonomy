# artifact 能力契约

输入：autonomy_artifact_path、autonomy_artifact_sha256、autonomy_expected_git_sha、autonomy_install_prefix、autonomy_release_dir。

输出：校验并发布完整 release，切换安装链接、保留 previous，并报告 autonomy_artifact_activation；不修改运行数据。

依赖：deployment_defaults；Python、tar 和运行用户/依赖由调用方准备。

元数据中的 Role 依赖为运行前自动加载的依赖；系统前提由场景入口按顺序提供。
