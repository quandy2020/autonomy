# Audio is a loadable autolink component (direction / moving / optional ASR).
if(NOT TARGET autolink)
  return()
endif()

set(_AUDIO_ASR_SRCS inference/asr/asr_engine.cpp)

add_library(audio_component SHARED
  audio_component.cpp
  options.cpp
  common/audio_info.cpp
  common/message_process.cpp
  inference/fft.cpp
  inference/direction_detection.cpp
  inference/moving_detection.cpp
  ${_AUDIO_ASR_SRCS})
target_include_directories(audio_component PRIVATE
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_BINARY_DIR})
target_link_libraries(audio_component PRIVATE autonomy autolink automsgs yaml-cpp)

if(BUILD_SHERPA_ONNX AND SherpaOnnx_FOUND)
  target_compile_definitions(audio_component PRIVATE AUTONOMY_HAS_SHERPA_ONNX)
  target_include_directories(audio_component PRIVATE ${SherpaOnnx_INCLUDE_DIRS})
  target_link_libraries(audio_component PRIVATE ${SherpaOnnx_LIBRARIES})
  message(STATUS "audio_component: sherpa-onnx ASR enabled")
else()
  message(STATUS "audio_component: ASR stub (no sherpa-onnx)")
endif()

set_target_properties(audio_component PROPERTIES OUTPUT_NAME audio_component)
unset(_AUDIO_ASR_SRCS)

# Ship conf/dag next to the source tree layout used by audio.dag paths.
install(
  DIRECTORY conf/
  DESTINATION share/autonomy/audio/conf
  FILES_MATCHING
  PATTERN "*.pb.txt"
  PATTERN "*.yaml")
install(
  DIRECTORY dag/
  DESTINATION share/autonomy/audio/dag
  FILES_MATCHING
  PATTERN "*.dag")
