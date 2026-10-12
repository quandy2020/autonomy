# Example binaries. Built when libbolza is available (CppAD, OpenMP, …).
# Each process publishes MarkerArray, JointState, and Path for autoviz and
# subscribes to a PoseStamped target.

if(NOT AUTOMANIP_WITH_SOLVER)
  message(STATUS "automanip examples skipped (solver dependencies are missing)")
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

Function(automanip_example name)
  autocmake_binary(${name}_demo
    SOURCES ${ARGN} ${_EXAMPLE_RUNTIME}
    DEPENDENCIES automanip autolink automsgs)
  target_compile_definitions(${name}_demo PRIVATE
    AUTOMANIP_EXAMPLE_TASK="${CMAKE_CURRENT_SOURCE_DIR}/${name}/config/mpc/task.info"
    AUTOMANIP_EXAMPLE_URDF="${CMAKE_CURRENT_SOURCE_DIR}/${name}/urdf/${name}.urdf")
  target_include_directories(${name}_demo PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/..
    ${CMAKE_CURRENT_SOURCE_DIR}/../autolink/thirdparty/CLI11/include)
endfunction()

automanip_example(cartpole
  cartpole/cartpole.cpp)

automanip_example(double_integrator
  double_integrator/double_integrator.cpp)

automanip_example(quadrotor
  quadrotor/quadrotor.cpp
  quadrotor/quadrotor_system_dynamics.cpp)

automanip_example(ballbot
  ballbot/ballbot.cpp
  ballbot/dynamics/ballbot_system_dynamics.cpp
  ballbot/generated/miscellaneous.cpp)

if(AUTOMANIP_WITH_PINOCCHIO)
  file(GLOB_RECURSE _LEGGED_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/legged_robot/*.cpp")
  list(FILTER _LEGGED_SOURCES EXCLUDE REGEX "/demo\\.cpp$")
  automanip_example(legged_robot ${_LEGGED_SOURCES} legged_robot/demo.cpp)
  target_compile_definitions(legged_robot_demo PRIVATE
    AUTOMANIP_EXAMPLE_REFERENCE="${CMAKE_CURRENT_SOURCE_DIR}/legged_robot/config/command/reference.info")
  if(hppfcl_FOUND)
    file(GLOB_RECURSE _ARM_SOURCES CONFIGURE_DEPENDS
      "${CMAKE_CURRENT_SOURCE_DIR}/mobile_manipulator/*.cpp")
    list(FILTER _ARM_SOURCES EXCLUDE REGEX "/demo\\.cpp$")
    automanip_example(mobile_manipulator ${_ARM_SOURCES} mobile_manipulator/demo.cpp)
  else()
    message(STATUS "automanip: mobile_manipulator example skipped (hpp-fcl not found)")
  endif()
else()
  message(STATUS "automanip: legged_robot and mobile_manipulator examples skipped (pinocchio not found)")
endif()

install(
  DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/launch/"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/launch"
  FILES_MATCHING PATTERN "*.launch")
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/cartpole/urdf/cartpole.urdf"
  "${CMAKE_CURRENT_SOURCE_DIR}/double_integrator/urdf/double_integrator.urdf"
  "${CMAKE_CURRENT_SOURCE_DIR}/quadrotor/urdf/quadrotor.urdf"
  "${CMAKE_CURRENT_SOURCE_DIR}/ballbot/urdf/ballbot.urdf"
  "${CMAKE_CURRENT_SOURCE_DIR}/legged_robot/urdf/legged_robot.urdf"
  "${CMAKE_CURRENT_SOURCE_DIR}/mobile_manipulator/urdf/mobile_manipulator.urdf"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/urdf")
install(FILES
  "${CMAKE_CURRENT_SOURCE_DIR}/ballbot/meshes/base.obj"
  "${CMAKE_CURRENT_SOURCE_DIR}/quadrotor/meshes/quadrotor.obj"
  DESTINATION "${CMAKE_INSTALL_DATAROOTDIR}/automanip/meshes")
