# Conversion schema

MCAP → LeRobot，且 **meta 必须带回 `episode_id` 与 VersionTuple**，否则不得进 `lerobot_builder`。

- 通道：`field_mapping`（odom / cmd / image / state）
- 时钟：`clock_alignment` 用 `mono_ns`
- 视频：`video_encoder`（可选）
- 表：`parquet_writer` + `lerobot_layout`

preset：`teleop_imitation` / `navigation_imitation` / `perception_only`。P1 接管默认不进 teleop 专家集，除非人审放行。
