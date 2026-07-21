-- Copyright 2026 Autodriver contributors
--
-- Sensor HAL hub configuration (loaded via driver/autodriver.lua).

AUTODRIVER = {
  -- Register mock_* factories before instantiating drivers below.
  register_builtin_mocks = true,

  hub = {
    alignment_window_ms = 100,
    publish_period_ms = 20,
    buffer_capacity = 32,
  },

  -- Each entry: factory_name + sensor_id + optional params table.
  drivers = {
    { factory_name = "mock_imu", sensor_id = "imu/primary" },
    { factory_name = "mock_wheel_odom", sensor_id = "wheel_odom/primary" },
    { factory_name = "mock_lidar", sensor_id = "lidar/front" },
    { factory_name = "mock_gps", sensor_id = "gps/primary" },
    { factory_name = "mock_range", sensor_id = "range/front" },
    { factory_name = "mock_camera", sensor_id = "camera/front" },

    -- Hardware examples (disabled by default; see driver/autodriver_hardware.lua):
    -- { factory_name = "serial_imu", sensor_id = "imu/serial",
    --   params = { device = "/dev/ttyUSB0", baud = 115200 } },
    -- { factory_name = "serial_gps", sensor_id = "gps/serial",
    --   params = { device = "/dev/ttyUSB1", baud = 9600 } },
    -- { factory_name = "can_imu", sensor_id = "imu/can",
    --   params = { interface = "can0", accel_can_id = 0x100, gyro_can_id = 0x101 } },
    -- { factory_name = "can_gps", sensor_id = "gps/can",
    --   params = { interface = "can0", can_id = 0x12902500 } },
    -- RealSense (see driver/autodriver_realsense.lua):
    -- { factory_name = "realsense_camera", sensor_id = "camera/color",
    --   params = { model = "D435", stream = "color", width = 640, height = 480, fps = 30 } },
    -- { factory_name = "realsense_depth", sensor_id = "camera/depth",
    --   params = { model = "D455", width = 848, height = 480, fps = 30 } },
    -- { factory_name = "realsense_imu", sensor_id = "imu/realsense",
    --   params = { model = "D435" } },
  },
}

-- LuaParameterDictionary expects a table return when loaded directly.
return AUTODRIVER
