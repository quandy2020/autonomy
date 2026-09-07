# Camera vendor **device params** (not full sensor lists)

Main process config stays `config/autodriver_hardware.yaml` (channels,
enable, stream, resolution). Point each camera / point_cloud / imu entry at a
vendor file with `params_file:`:

```yaml
camera:
  - name: orbbec_color
    enable: true
    channel: /camera/color/image_raw
    backend: orbbec
    stream: color
    params_file: camera/orbbec/gemini_330.yaml
    params:
      frame_id: camera_color_optical_frame   # overrides file
```

| Vendor | Params file |
|---|---|
| Orbbec Gemini 330 | `orbbec/gemini_330.yaml` |
| RealSense D455 | `realsense/d455.yaml` |
| SmarterEye | `smartereye/autodriver.yaml` |

File format: flat key/value map (or `{ params: {…} }`). Inline `params:` win over the file.
