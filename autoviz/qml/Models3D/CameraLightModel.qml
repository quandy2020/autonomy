import QtQuick
import QtQuick3D

// Lightweight camera rig inspired by QGroundControl CameraLightModel.
Node {
    id: root

    property alias camera: cameraNode
    property real viewDistance: 5000

    DirectionalLight {
        brightness: 0.55
        eulerRotation: Qt.vector3d(-35, -55, 0)
        castsShadow: false
    }

    DirectionalLight {
        brightness: 0.35
        eulerRotation: Qt.vector3d(-20, 120, 0)
    }

    DirectionalLight {
        brightness: 0.2
        eulerRotation: Qt.vector3d(90, 0, 0)
    }

    PerspectiveCamera {
        id: cameraNode
        clipFar: root.viewDistance
        clipNear: 0.5
        fieldOfView: 45
    }
}
