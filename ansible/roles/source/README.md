# source 能力契约

输入：autonomy_use_local_source、autonomy_local_source_dir，或 autonomy_repo_url、autonomy_repo_version、autonomy_source_dir。

输出：验证本地树或检出包含 submodule 的源码；不强制覆盖本地改动。

依赖：deployment_defaults；Git、源码父目录和访问权限由调用方准备。

元数据中的 Role 依赖为运行前自动加载的依赖；系统前提由场景入口按顺序提供。
