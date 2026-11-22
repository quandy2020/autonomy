# Copyright 2026 The OpenRobotic Beginner Authors (duyongquan)
# Licensed under the Apache License, Version 2.0.


# DAG / launch assets; install when a prefix is set.
install(FILES
    data_agent.dag model_sync.dag navigation.dag perception.dag
    localization.dag drivers.dag monitor.dag
  DESTINATION share/autoloop/bringup)
install(DIRECTORY launch DESTINATION share/autoloop/bringup)
