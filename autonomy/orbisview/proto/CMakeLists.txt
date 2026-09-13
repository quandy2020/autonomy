# OrbisView .proto definitions (Dreamview proto/ counterpart).
# Runtime WS currently uses JSON + backend/common/render_schemas.h schema IDs.
# Do not place C++ sources here — only *.proto.

file(GLOB _ORBISVIEW_PROTOS "${CMAKE_CURRENT_SOURCE_DIR}/*.proto")
message(STATUS "orbisview proto: ${_ORBISVIEW_PROTOS}")
