# =============================================================================
# Install rules: binaries, config, Ogre media, scripts, Linux desktop entry/icons
# =============================================================================

include(GNUInstallDirs)

install(TARGETS autoviz autoviz_bicmap_publisher RUNTIME DESTINATION bin)

install(FILES config/default.autoviz config/turtlebot3_burger.urdf
        DESTINATION share/autonomy/autoviz)
install(DIRECTORY resources/ogre_media DESTINATION share/autonomy/autoviz)
install(PROGRAMS
  scripts/mcap_to_record.py
  scripts/publish_test_sensors.py
  scripts/publish_bicmap_example.py
  scripts/run_bicmap_example.sh
  scripts/run_autoviz.sh
  scripts/check_gui_env.sh
  scripts/autoviz_bicmap_publisher
  scripts/install_linux_desktop.sh
  DESTINATION share/autonomy/autoviz/scripts)

# .desktop / AppStream (template vars from Config.cmake AUTOVIZ_*)
# .desktop / AppStream (template vars from Config.cmake AUTOVIZ_*)
string(TIMESTAMP AUTOVIZ_BUILD_DATE "%Y-%m-%d" UTC)
configure_file(${AUTOVIZ_ROOT}/deploy/linux/org.autonomy.autoviz.desktop.in
  ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.desktop @ONLY)
configure_file(${AUTOVIZ_ROOT}/deploy/linux/org.autonomy.autoviz.appdata.xml.in
  ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.appdata.xml @ONLY)

install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.desktop
  DESTINATION share/applications)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/org.autonomy.autoviz.appdata.xml
  DESTINATION share/metainfo)

# App / desktop icon (RViz package icon)
set(_icon_png ${AUTOVIZ_ROOT}/resources/icons/aviz.png)
set(_icon_svg ${AUTOVIZ_ROOT}/resources/icons/aviz.svg)
if(EXISTS ${_icon_svg})
  install(FILES ${_icon_svg}
    DESTINATION share/icons/hicolor/scalable/apps RENAME aviz.svg)
endif()

# Optional: multi-size PNG icons via rsvg-convert (SVG) or copy master PNG
find_program(RSVG_CONVERT rsvg-convert)
if(EXISTS ${_icon_svg} AND RSVG_CONVERT)
  set(_png_master ${_icon_svg})
  set(_use_rsvg TRUE)
elseif(EXISTS ${_icon_png})
  set(_png_master ${_icon_png})
  set(_use_rsvg FALSE)
else()
  set(_png_master "")
endif()

if(_png_master)
  set(_png_outputs "")
  foreach(_size 48 64 128 256)
    set(_dir ${CMAKE_CURRENT_BINARY_DIR}/icons/hicolor/${_size}x${_size}/apps)
    set(_png ${_dir}/aviz.png)
    if(_use_rsvg)
      add_custom_command(OUTPUT ${_png}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${_dir}
        COMMAND ${RSVG_CONVERT} -w ${_size} -h ${_size} ${_png_master} -o ${_png}
        DEPENDS ${_png_master} COMMENT "Generate aviz ${_size}x${_size} icon")
    else()
      add_custom_command(OUTPUT ${_png}
        COMMAND ${CMAKE_COMMAND} -E make_directory ${_dir}
        COMMAND ${CMAKE_COMMAND} -E copy ${_png_master} ${_png}
        DEPENDS ${_png_master} COMMENT "Install aviz ${_size}x${_size} icon")
    endif()
    list(APPEND _png_outputs ${_png})
  endforeach()
  add_custom_target(autoviz_icons ALL DEPENDS ${_png_outputs})
  install(DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/icons/hicolor/ DESTINATION share/icons)
  if(NOT _use_rsvg)
    install(FILES ${_icon_png} DESTINATION share/icons/hicolor/48x48/apps RENAME aviz.png)
  endif()
endif()

# Test binaries (only if Tests.cmake created the targets)
foreach(_t autoviz_ogre_pick_verify autoviz_mesh_loader_test autoviz_rendering_objects_test
         autoviz_bicmap_examples_test)
  if(TARGET ${_t})
    install(TARGETS ${_t} RUNTIME DESTINATION bin)
  endif()
endforeach()
