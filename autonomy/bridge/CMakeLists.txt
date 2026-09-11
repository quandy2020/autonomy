# autonomy/bridge — standalone gRPC process (autonomy.bridge) + module assets.
#
# Runtime Lua config lives under share/autonomy/config/bridge/ (repo config/bridge/).
# This directory ships optional protobuf-text defaults and channel topology docs.

if(NOT BUILD_GRPC)
  return()
endif()

google_binary(autonomy.bridge
  SRCS bridge_main.cpp)

# Module assets (install layout mirrors audio / perception components).
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/bridge/conf
  FILES_MATCHING
  PATTERN "*.pb.txt")
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/bridge/dag
  FILES_MATCHING
  PATTERN "*.dag")
install(
  DIRECTORY launch/
  DESTINATION share/autonomy/bridge/launch
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
  PATTERN "*.launch")
