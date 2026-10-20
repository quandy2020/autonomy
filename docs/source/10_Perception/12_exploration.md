# 12. 探索

仓库里有探索相关配置（`PerceptionOptions.enable_rgbd_exploration`、`exploration_*.lua`），但当前构建**没有**对应后端。`PerceptionServer::Start()` 打开开关时只会打：

```text
PerceptionServer: exploration backend is not built.
```

能用的视觉能力看 [Base](10_base.md) 和 [Follow](11_follow.md)，别把探索配置当成已上线功能。
