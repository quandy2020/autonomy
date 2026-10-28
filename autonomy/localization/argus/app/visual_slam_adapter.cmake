if(ARGUS_BUILD_MESSAGE_ADAPTERS)
  add_library(argus_visual_slam_messages STATIC
    visual_slam_message_adapter.cpp
  )
  target_include_directories(argus_visual_slam_messages
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${ARGUS_LIBRARY_ROOT}
      ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(argus_visual_slam_messages
    PUBLIC
      argus::visual_slam
      argus_automsgs
      ${ARGUS_AUTOMSGS_TARGET}
      ${OpenCV_LIBS}
  )
  target_compile_features(argus_visual_slam_messages PUBLIC cxx_std_17)
  set_target_properties(
    argus_visual_slam_messages PROPERTIES POSITION_INDEPENDENT_CODE ON)
  add_library(argus::visual_slam_messages ALIAS argus_visual_slam_messages)
endif()
