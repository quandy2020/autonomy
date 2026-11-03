# Overlay GPS display

Port of [gps_rviz_plugin](https://github.com/hakuturu583/gps_rviz_plugin)
`OverlayGpsDisplay` for Autoviz.

- **Type id:** `OverlayGps`
- **Message:** `sensor_msgs/NavSatFix` (`automsgs.msgs.sensor_msgs.NavSatFix`)
- **Map source:** Google Static Maps HTTP API (Qt Network; no Python/OpenCV)

Set the `API Key` property to a Google Static Maps key. Zoom / size / map type /
history / alpha / screen position match the upstream RViz plugin.
