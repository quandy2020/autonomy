# autonomy.tools.god_viewer
google_binary(autonomy.tools.god_viewer
  SRCS
    god_viewer_main.cpp
)

# install
install(TARGETS
  autonomy.tools.god_viewer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)