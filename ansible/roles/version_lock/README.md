# version_lock 能力契约

输入：use_locked_versions 显式启用；autonomy_version_lock_file 可选择受版本管理的锁文件。

输出：验证平台/发布身份；需要安装依赖时应用精确 APT pin（优先级 1001，允许降级）。

依赖：deployment_defaults；锁文件、主机事实与发布身份。

元数据中的 Role 依赖为运行前自动加载的依赖；系统前提由场景入口按顺序提供。
