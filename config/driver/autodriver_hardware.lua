-- Copyright 2026 Autodriver contributors
--
-- Hardware sensor HAL preset (serial + CAN IMU/GPS).
-- Usage: point autodriver_hub or driver_hub_node configuration_directory here,
-- or merge drivers into AUTODRIVER.drivers in driver/autodriver.lua.

AUTODRIVER = {
  register_builtin_mocks = false,

  hub = {
    alignment_window_ms = 50,
    publish_period_ms = 10,
    buffer_capacity = 64,
  },

  drivers = {
    {
      factory_name = "serial_imu",
      sensor_id = "imu/serial",
      params = {
        device = "/dev/ttyUSB0",
        baud = 115200,
      },
    },
    {
      factory_name = "serial_gps",
      sensor_id = "gps/serial",
      params = {
        device = "/dev/ttyUSB1",
        baud = 9600,
      },
    },
    {
      factory_name = "can_imu",
      sensor_id = "imu/can",
      params = {
        interface = "can0",
        accel_can_id = 0x100,
        gyro_can_id = 0x101,
        accel_scale = 0.001,
        gyro_scale = 0.0001,
      },
    },
    {
      factory_name = "can_gps",
      sensor_id = "gps/can",
      params = {
        interface = "can0",
        can_id = 0x12902500,
      },
    },
  },
}

return AUTODRIVER
