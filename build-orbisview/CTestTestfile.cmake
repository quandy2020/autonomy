# CMake generated Testfile for 
# Source directory: /Users/quandy/Workspace/github/autonomy/autonomy/orbisview
# Build directory: /Users/quandy/Workspace/github/autonomy/build-orbisview
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test("orbisview_throttle_queue_test" "/Users/quandy/Workspace/github/autonomy/build-orbisview/orbisview_throttle_queue_test")
set_tests_properties("orbisview_throttle_queue_test" PROPERTIES  _BACKTRACE_TRIPLES "/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;141;add_test;/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;0;")
add_test("orbisview_stream_envelope_test" "/Users/quandy/Workspace/github/autonomy/build-orbisview/orbisview_stream_envelope_test")
set_tests_properties("orbisview_stream_envelope_test" PROPERTIES  _BACKTRACE_TRIPLES "/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;148;add_test;/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;0;")
add_test("orbisview_envelope_recorder_test" "/Users/quandy/Workspace/github/autonomy/build-orbisview/orbisview_envelope_recorder_test")
set_tests_properties("orbisview_envelope_recorder_test" PROPERTIES  _BACKTRACE_TRIPLES "/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;155;add_test;/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;0;")
add_test("orbisview_plugin_host_test" "/Users/quandy/Workspace/github/autonomy/build-orbisview/orbisview_plugin_host_test")
set_tests_properties("orbisview_plugin_host_test" PROPERTIES  _BACKTRACE_TRIPLES "/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;162;add_test;/Users/quandy/Workspace/github/autonomy/autonomy/orbisview/CMakeLists.txt;0;")
subdirs("thirdparty/civetweb")
subdirs("backend")
subdirs("proto")
