

add_library(${PROJECT_NAME}_TestPlugins1 EXCLUDE_FROM_ALL plugins1.cpp)
target_link_libraries(${PROJECT_NAME}_TestPlugins1 ${PROJECT_NAME})
class_loader_hide_library_symbols(${PROJECT_NAME}_TestPlugins1)

add_library(${PROJECT_NAME}_TestPlugins2 EXCLUDE_FROM_ALL plugins2.cpp)
target_link_libraries(${PROJECT_NAME}_TestPlugins2 ${PROJECT_NAME})
class_loader_hide_library_symbols(${PROJECT_NAME}_TestPlugins2)

catkin_add_gtest(${PROJECT_NAME}_utest utest.cpp)
if(TARGET ${PROJECT_NAME}_utest)
  target_link_libraries(${PROJECT_NAME}_utest ${Boost_LIBRARIES} ${class_loader_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_utest ${PROJECT_NAME}_TestPlugins1 ${PROJECT_NAME}_TestPlugins2)
endif()

catkin_add_gtest(${PROJECT_NAME}_shared_ptr_test shared_ptr_test.cpp)
if(TARGET ${PROJECT_NAME}_shared_ptr_test)
  target_link_libraries(${PROJECT_NAME}_shared_ptr_test ${Boost_LIBRARIES} ${class_loader_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_shared_ptr_test ${PROJECT_NAME}_TestPlugins1 ${PROJECT_NAME}_TestPlugins2)
endif()

catkin_add_gtest(${PROJECT_NAME}_unique_ptr_test unique_ptr_test.cpp)
if(TARGET ${PROJECT_NAME}_unique_ptr_test)
  target_link_libraries(${PROJECT_NAME}_unique_ptr_test ${Boost_LIBRARIES} ${class_loader_LIBRARIES})
  add_dependencies(${PROJECT_NAME}_unique_ptr_test ${PROJECT_NAME}_TestPlugins1 ${PROJECT_NAME}_TestPlugins2)
endif()
