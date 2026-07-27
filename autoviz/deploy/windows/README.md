# Windows 部署

Autoviz 可在 Windows（MSVC + Qt 6）上构建与运行，步骤见 [`docs/DEPLOYMENT.md`](../../docs/DEPLOYMENT.md)。

## 构建概要

```powershell
cmake -S .. -B build -DBUILD_AUTOVIZ=ON -DCMAKE_PREFIX_PATH=C:\Qt\6.x\msvc2019_64
cmake --build build --target autoviz --config Release
```

## 环境变量

| 变量 | 说明 |
|------|------|
| `AUTOVIZ_PLUGIN_PATH` | 插件目录（`;` 分隔） |
| `AUTOVIZ_RESOURCE_PATH` | `package://` mesh 搜索路径 |

安装包（NSIS / WiX）尚未纳入本目录；可参考 QGC [`deploy/windows/nullsoft_installer.nsi`](https://github.com/mavlink/qgroundcontrol/tree/master/deploy/windows) 自行扩展。
