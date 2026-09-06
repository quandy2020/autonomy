# smartereye

厂商立体相机骨架（独立 smartereye 分包（不塞进 camera/））。

通过 `REGISTER_CAMERA_BACKEND(..., "smartereye", ...)` 挂到 `CameraBackendRegistry`；无专有 SDK 时 `Create` 返回 `nullptr`。

后续可在本目录加 UDP/device hub；图像仍由 `CameraModule` 消费。

YAML：`camera` + `backend: smartereye`。详见 [backends](../../docs/source/guide/backends.md)。
