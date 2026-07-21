-- Copyright 2026 Autodriver contributors
--
-- Intel RealSense preset (D435i + D455). Requires librealsense2 at build time.
-- List devices: rs-enumerate-devices

AUTODRIVER = {
  register_builtin_mocks = false,

  hub = {
    alignment_window_ms = 50,
    publish_period_ms = 33,
    buffer_capacity = 32,
  },

  drivers = {
    -- D435i: RGB + onboard IMU (use serial or model filter)
    {
      factory_name = "realsense_camera",
      sensor_id = "camera/color",
      params = {
        model = "D435",   -- matches D435 / D435i
        -- serial = "0123456789",  -- prefer exact serial when multiple devices
        stream = "color",
        width = 640,
        height = 480,
        fps = 30,
      },
    },
    {
      factory_name = "realsense_depth",
      sensor_id = "camera/depth",
      params = {
        model = "D435",
        width = 640,
        height = 480,
        fps = 30,
      },
    },
    {
      factory_name = "realsense_imu",
      sensor_id = "imu/realsense",
      params = {
        model = "D435",   -- IMU available on D435i (ignored on D455)
      },
    },

    -- D455 example (no IMU): duplicate entries with model = "D455" only.
    -- {
    --   factory_name = "realsense_camera",
    --   sensor_id = "camera/d455/color",
    --   params = { model = "D455", stream = "color", width = 848, height = 480, fps = 30 },
    -- },
  },
}

return AUTODRIVER
