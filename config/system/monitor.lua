-- Copyright 2026 The Openbot Authors
--
-- System monitor: host metrics + autolink channel/latency/hazard/MRM.

monitor = {
  enable_cpu_monitor = true,
  enable_mem_monitor = true,
  enable_gpu_monitor = false,
  enable_hdd_monitor = true,
  enable_net_monitor = true,
  enable_ntp_monitor = false,
  enable_process_monitor = true,
  enable_voltage_monitor = false,

  enable_channel_monitor = true,
  enable_latency_monitor = true,
  enable_hazard_monitor = true,
  enable_mrm_handler = false,

  channel_watches = {
    { channel = "/cmd_vel", timeout_sec = 1.0, min_rate_hz = 0.0 },
    { channel = "/autonomy/task/teleop/goal", timeout_sec = 5.0, min_rate_hz = 0.0 },
    { channel = "/autonomy/task/teleop/feedback", timeout_sec = 5.0, min_rate_hz = 0.0 },
  },

  latency_watches = {
    { channel = "/cmd_vel", max_age_sec = 0.5 },
  },

  mrm = {
    cmd_vel_channel = "/cmd_vel",
    emergency_stop_on_error = true,
  },

  enable_prometheus = true,
  prometheus_bind_address = "0.0.0.0:9090",

  collect_interval_sec = 1.0,

  enable_cpu_profile = false,
  cpu_profile_filename = "",
  enable_heap_profile = false,
  heap_profile_filename = "",
}
