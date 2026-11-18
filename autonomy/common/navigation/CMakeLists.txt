# Shared non-ROS geometry and observation support for native domain consumers.
# Sources outside this directory remain in perception's public source layout;
# its domain excludes these implementations to prevent duplicate definitions.
file(GLOB AUTONOMY_NAVIGATION_GEOMETRY_SRCS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/*.cc")
add_library(autonomy_navigation_support STATIC
  ${AUTONOMY_NAVIGATION_GEOMETRY_SRCS}
  ${PROJECT_SOURCE_DIR}/autonomy/perception/data/map_reader.cpp
  ${PROJECT_SOURCE_DIR}/autonomy/perception/data/sensor_observation.cpp
  ${PROJECT_SOURCE_DIR}/autonomy/perception/data/observation_buffer.cpp
  ${PROJECT_SOURCE_DIR}/autonomy/perception/observation/lidar_observation_adapter.cpp
  ${PROJECT_SOURCE_DIR}/autonomy/perception/observation/rgbd_observation_adapter.cpp
  ${PROJECT_SOURCE_DIR}/autonomy/perception/observation/sensor_message_decoder.cpp
  ${PROJECT_SOURCE_DIR}/autonomy/perception/observation/observation_ingress.cpp)
add_library(autonomy::navigation_support ALIAS autonomy_navigation_support)
target_include_directories(autonomy_navigation_support PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}>
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(autonomy_navigation_support PUBLIC cxx_std_17)
target_link_libraries(autonomy_navigation_support PUBLIC Threads::Threads)
set_target_properties(autonomy_navigation_support PROPERTIES
  POSITION_INDEPENDENT_CODE ON CXX_EXTENSIONS OFF)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/opt/navigation
  DESTINATION include/opt FILES_MATCHING PATTERN "*.h" PATTERN "bt" EXCLUDE)
