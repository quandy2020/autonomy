# minimal scene

默认 `habitat.path` 为空时使用 Habitat 空舞台（或 CI 下 Mock），无需下载数据集。

切换真实场景：

```yaml
habitat:
  path: /data/hm3d/scene.basis.glb
```

支持 HM3D / Replica / MP3D 等 Habitat 可加载资产。
