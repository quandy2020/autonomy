# build 能力契约

输入：autonomy_source_dir、autonomy_cmake_build_dir、autonomy_install_prefix、autonomy_use_ccache、autonomy_cmake_jobs、autonomy_cmake_extra_args。

输出：缓存配置、增量编译、可选 CTest 和安装；保护只读 release 链接。

依赖：deployment_defaults；源码与第三方编译依赖由调用方准备。

元数据中的 Role 依赖为运行前自动加载的依赖；系统前提由场景入口按顺序提供。
