# build 能力契约

输入：autonomy_source_dir、autonomy_cmake_build_dir、autonomy_install_prefix、autonomy_use_ccache、autonomy_cmake_jobs、autonomy_cmake_extra_args、autonomy_build_modules。

输出：缓存配置、增量编译、可选 CTest 和安装；通过 autonomy_cmake_install 报告安装变更，保护 release 链接。指定 autonomy_build_modules 时，输出独立构建目录，跳过安装。

依赖：deployment_defaults；源码与第三方编译依赖由调用方准备。
