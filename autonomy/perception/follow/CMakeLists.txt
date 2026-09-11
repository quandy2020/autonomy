# Follow is a loadable autolink component. It consumes base tracks + depth and
# publishes target / path / grid for TrackingClient + MPPI GridObstaclesCritic.
if(NOT TARGET autolink)
  return()
endif()

add_library(follow_component SHARED
  follow_component.cpp
  options.cpp
  localizer.cpp
  grid.cpp
  planner.cpp)
target_include_directories(follow_component PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR})
target_link_libraries(follow_component PRIVATE autonomy autolink automsgs)
set_target_properties(follow_component PROPERTIES OUTPUT_NAME follow_component)