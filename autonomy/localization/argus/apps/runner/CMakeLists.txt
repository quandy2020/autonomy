# apps/runner — Voxel-SLAM offline entry

argus_binary(argus_runner SRCS main.cpp)
argus_binary(offline_voxel_slam SRCS offline_voxel_slam.cpp)

if(ARGUS_BUILD_INSTALL)
  install(DIRECTORY configs DESTINATION share/argus/apps/runner)
endif()
