# deployment_defaults 能力契约

输入：autonomy_play_mode、autonomy_deploy_mode、autonomy_services、autonomy_controller_root；保留 autonomy_* 公共接口。

输出：验证调用契约；只在控制机加载可选加密 vault，不打印秘密。

依赖：无。

元数据中的 Role 依赖为运行前自动加载的依赖；系统前提由场景入口按顺序提供。
