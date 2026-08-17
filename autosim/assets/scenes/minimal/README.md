# minimal scene

默认 `scene.backend: minimal` 使用 Habitat 空舞台（或 CI 下 `MockWorld`），无需下载数据集。

切换真实场景：

```yaml
scene:
  backend: habitat
  path: /data/hm3d/scene.basis.glb
```

支持 HM3D / Replica / MP3D 等 Habitat 可加载资产。
