cmake_minimum_required(VERSION 3.5)
project(class_loader CXX)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(Boost REQUIRED COMPONENTS thread system)

set(Poco_DIR cmake)
find_package(Poco REQUIRED COMPONENTS Foundation)
set(CATKIN_GLOBAL_BIN_DESTINATION bin)
set(CATKIN_GLOBAL_LIB_DESTINATION lib)
set(CATKIN_GLOBAL_LIBEXEC_DESTINATION lib)
set(CATKIN_GLOBAL_INCLUDE_DESTINATION include)
set(CATKIN_PACKAGE_LIB_DESTINATION ${CATKIN_GLOBAL_LIB_DESTINATION})
set(CATKIN_PACKAGE_BIN_DESTINATION ${CATKIN_GLOBAL_LIBEXEC_DESTINATION}/${PROJECT_NAME})
set(CATKIN_PACKAGE_INCLUDE_DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/${PROJECT_NAME})


include_directories(include ${console_bridge_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${Poco_INCLUDE_DIRS})

set(${PROJECT_NAME}_SRCS
  src/class_loader.cpp
  src/class_loader_core.cpp
  src/meta_object.cpp
  src/multi_library_class_loader.cpp
)
set(${PROJECT_NAME}_HDRS
  include/class_loader/class_loader.hpp
  include/class_loader/class_loader_core.hpp
  include/class_loader/exceptions.hpp
  include/class_loader/meta_object.hpp
  include/class_loader/multi_library_class_loader.hpp
  include/class_loader/register_macro.hpp
)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SRCS} ${${PROJECT_NAME}_HDRS})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES} ${console_bridge_LIBRARIES} ${Poco_LIBRARIES})


install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/class_loader/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

set(TARGET_NAME ${PROJECT_NAME})
set(PKGCONFIG_LIBS
  ${Boost_LIBRARIES}
  ${console_bridge_LIBRARIES}
  ${Poco_LIBRARIES}
)

