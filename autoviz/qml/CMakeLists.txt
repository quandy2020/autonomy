# Autoviz.Vehicle3D — QGC Viewer3D-style Qt Quick 3D module (ground + aerial).

set(_AUTOVIZ_QML_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(_AUTOVIZ_QML_MESH_DIR ${_AUTOVIZ_QML_ROOT}/meshes/Djif450)
set(_AUTOVIZ_QML_MESH_RES "")

if(EXISTS "${_AUTOVIZ_QML_MESH_DIR}/DroneModel_arm_1/node.mesh")
  set(_AUTOVIZ_QML_MESH_NAMES
    DroneModel_arm_1/node.mesh
    DroneModel_arm_2/node.mesh
    DroneModel_arm_3/node.mesh
    DroneModel_arm_4/node.mesh
    DroneModel_BLDC_1/node.mesh
    DroneModel_BLDC_2/node.mesh
    DroneModel_BLDC_3/node.mesh
    DroneModel_BLDC_4/node.mesh
    DroneModel_propeller2_2/node.mesh
    DroneModel_propeller2_7/node.mesh
    DroneModel_propeller22_1/node.mesh
    DroneModel_propeller22_2/node.mesh
    DroneModel_Base_bottom_1/node.mesh
    DroneModel_Base_Top_1/node.mesh
  )
  foreach(_mesh_name ${_AUTOVIZ_QML_MESH_NAMES})
    set(_mesh_file "${_AUTOVIZ_QML_MESH_DIR}/${_mesh_name}")
    set_source_files_properties(${_mesh_file} PROPERTIES
      QT_RESOURCE_ALIAS "Djif450/${_mesh_name}")
    list(APPEND _AUTOVIZ_QML_MESH_RES ${_mesh_file})
  endforeach()
  message(STATUS "Autoviz.Vehicle3D: DJI F450 meshes bundled")
else()
  message(STATUS "Autoviz.Vehicle3D: F450 meshes missing — using procedural models")
  message(STATUS "  Run: src/autonomy/autoviz/tools/setup/copy_qgc_drone_meshes.sh [QGC_ROOT]")
endif()

qt_add_library(autoviz_vehicle_qml STATIC)

qt_add_qml_module(autoviz_vehicle_qml
  URI Autoviz.Vehicle3D
  VERSION 1.0
  RESOURCE_PREFIX /qml
  OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/Autoviz/Vehicle3D
  QML_FILES
    Models3D/VehicleViewer3D.qml
    Models3D/MissionPathOverlay.qml
    Models3D/CameraLightModel.qml
    Vehicles/VehiclePoseNode.qml
    Vehicles/VehiclePart.qml
    Ground/GroundPlane.qml
    Ground/GroundModelDiffDrive.qml
    Ground/GroundModelAckermann.qml
    Drones/DronePart.qml
    Drones/AnimatedPropeller.qml
    Drones/DroneModelSimple.qml
    Drones/DroneModelDjiF450.qml
  RESOURCES
    ${_AUTOVIZ_QML_MESH_RES}
  IMPORTS
    QtQuick
    QtQuick.Controls
    QtQuick3D
)

target_link_libraries(autoviz_vehicle_qml
  PUBLIC
    Qt6::Core
    Qt6::Gui
    Qt6::Qml
    Qt6::Quick
    Qt6::QuickWidgets
    Qt6::Quick3D
)

target_include_directories(autoviz_vehicle_qml PUBLIC ${_AUTOVIZ_QML_ROOT})
