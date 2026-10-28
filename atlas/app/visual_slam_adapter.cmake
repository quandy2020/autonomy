if(ATLAS_BUILD_MESSAGE_ADAPTERS)
  add_library(atlas_visual_slam_messages STATIC
    visual_slam_message_adapter.cpp
  )
  target_include_directories(atlas_visual_slam_messages
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${ATLAS_LIBRARY_ROOT}
      ${OpenCV_INCLUDE_DIRS}
  )
  target_link_libraries(atlas_visual_slam_messages
    PUBLIC
      atlas::visual_slam
      atlas_automsgs
      ${ATLAS_AUTOMSGS_TARGET}
      ${OpenCV_LIBS}
  )
  target_compile_features(atlas_visual_slam_messages PUBLIC cxx_std_17)
  set_target_properties(
    atlas_visual_slam_messages PROPERTIES POSITION_INDEPENDENT_CODE ON)
  add_library(atlas::visual_slam_messages ALIAS atlas_visual_slam_messages)
endif()
