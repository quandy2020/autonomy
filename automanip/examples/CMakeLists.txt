# Example binaries. Built when libbolza is available (CppAD, OpenMP, …).
# Each process publishes MarkerArray, JointState, and Path for autoviz and
# subscribes to a PoseStamped target.

if(NOT TARGET bolza)
  message(STATUS "automanip examples skipped (libbolza is not available)")
  return()
endif()

# Pure libbolza tutorial (no Autolink): docs/04_robotic_examples.md §4.1
if(TARGET bolza)
  add_executable(bolza_tutorial_di
    "${CMAKE_CURRENT_SOURCE_DIR}/tutorial/solve_double_integrator.cpp")
  target_link_libraries(bolza_tutorial_di PRIVATE bolza)
  target_include_directories(bolza_tutorial_di PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../bolza")
endif()

set(_EXAMPLE_RUNTIME
  "${CMAKE_CURRENT_SOURCE_DIR}/runtime/demo_loop.cpp")

get_filename_component(BOLZA_ASSETS_DIR
  "${CMAKE_CURRENT_SOURCE_DIR}/../bolza/examples/assets" ABSOLUTE)

# name, absolute URDF under bolza/examples/assets, then sources.
function(automanip_example name urdf)
  autocmake_binary(${name}_demo
    SOURCES ${ARGN} ${_EXAMPLE_RUNTIME}
    DEPENDENCIES automanip autolink automsgs bolza)
  target_compile_definitions(${name}_demo PRIVATE
    AUTOMANIP_EXAMPLE_TASK="${CMAKE_CURRENT_SOURCE_DIR}/${name}/config/mpc/task.info"
    AUTOMANIP_EXAMPLE_URDF="${urdf}"
    AUTOMANIP_ROBOTIC_ASSETS="${BOLZA_ASSETS_DIR}")
  target_include_directories(${name}_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include)
endfunction()

automanip_example(cartpole
  "${BOLZA_ASSETS_DIR}/cartpole/urdf/cartpole.urdf"
  cartpole/cartpole.cpp)

automanip_example(double_integrator
  "${BOLZA_ASSETS_DIR}/double_integrator/urdf/double_integrator.urdf"
  double_integrator/double_integrator.cpp)

automanip_example(quadrotor
  "${BOLZA_ASSETS_DIR}/quadrotor/urdf/quadrotor.urdf"
  quadrotor/quadrotor.cpp
  quadrotor/quadrotor_system_dynamics.cpp)

find_path(IIT_RBD_INCLUDE_DIR iit/rbd/rbd.h)
if(IIT_RBD_INCLUDE_DIR)
  automanip_example(ballbot
    "${BOLZA_ASSETS_DIR}/ballbot/urdf/ballbot.urdf"
    ballbot/ballbot.cpp
    ballbot/dynamics/ballbot_system_dynamics.cpp
    ballbot/generated/miscellaneous.cpp)
  target_include_directories(ballbot_demo PRIVATE "${IIT_RBD_INCLUDE_DIR}")
else()
  message(STATUS "automanip: ballbot example skipped (iit/rbd/rbd.h not found)")
endif()

if(TARGET pinocchio::pinocchio)
  file(GLOB_RECURSE _LEGGED_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/legged_robot/*.cpp")
  list(FILTER _LEGGED_SOURCES EXCLUDE REGEX "/demo\\.cpp$")
  automanip_example(legged_robot
    "${BOLZA_ASSETS_DIR}/anymal_c/urdf/anymal.urdf"
    ${_LEGGED_SOURCES} legged_robot/demo.cpp)
  target_compile_definitions(legged_robot_demo PRIVATE
    AUTOMANIP_EXAMPLE_REFERENCE="${CMAKE_CURRENT_SOURCE_DIR}/legged_robot/config/command/reference.info")
  file(GLOB_RECURSE _ARM_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/mobile_manipulator/*.cpp")
  list(FILTER _ARM_SOURCES EXCLUDE REGEX "/demo\\.cpp$")
  automanip_example(mobile_manipulator
    "${BOLZA_ASSETS_DIR}/mobile_manipulator/franka/urdf/panda.urdf"
    ${_ARM_SOURCES} mobile_manipulator/demo.cpp)
else()
  message(STATUS "automanip: legged_robot and mobile_manipulator examples skipped (pinocchio not found)")
endif()

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch/"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/launch"
  FILES_MATCHING PATTERN "*.launch")
