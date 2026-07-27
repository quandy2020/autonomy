import QtQuick
import QtQuick3D

import Autoviz.Vehicle3D

// Ackermann / car-like ground platform.
VehiclePoseNode {
    id: root

    clampGroundZ: true
    scale: Qt.vector3d(8, 8, 8)

    readonly property real wheelSpinSign: root.linearSpeed >= 0 ? 1 : -1
    readonly property real wheelDuration: {
        if (root.motionMode <= 0)
            return 1500
        const speed = Math.max(0.05, Math.abs(root.linearSpeed))
        return Math.max(200, Math.min(1600, 500 / speed))
    }

    Node {
        parent: bodyNode

        Model {
            source: "#Cube"
            scale: Qt.vector3d(0.85, 0.38, 0.22)
            position: Qt.vector3d(0, 0, 0.14)
            materials: PrincipledMaterial {
                baseColor: "#3d6b4f"
                metalness: 0.3
                roughness: 0.45
            }
        }

        Model {
            source: "#Cube"
            scale: Qt.vector3d(0.25, 0.34, 0.18)
            position: Qt.vector3d(0.32, 0, 0.32)
            materials: PrincipledMaterial { baseColor: "#88bb99"; roughness: 0.5 }
        }

        Repeater3D {
            model: 4
            Node {
                id: wheelNode
                property real fx: (index === 0 || index === 1) ? 0.28 : -0.28
                property real fy: (index === 0 || index === 2) ? -0.2 : 0.2
                property bool isFront: index === 0 || index === 1
                position: Qt.vector3d(fx, fy, 0.06)
                eulerRotation.y: isFront
                    ? Math.max(-35, Math.min(35, root.angularSpeed * 0.6))
                    : 0

                Model {
                    source: "#Cylinder"
                    scale: Qt.vector3d(0.1, 0.05, 0.1)
                    eulerRotation.x: 90
                    materials: PrincipledMaterial { baseColor: "#1a1a1a"; metalness: 0.4 }
                }

                PropertyAnimation on eulerRotation.z {
                    from: 0
                    to: 360 * root.wheelSpinSign
                    duration: root.wheelDuration
                    loops: Animation.Infinite
                    running: root.motionMode > 0
                }
            }
        }

        Model {
            source: "#Cone"
            scale: Qt.vector3d(0.05, 0.12, 0.05)
            position: Qt.vector3d(0.48, 0, 0.14)
            eulerRotation.z: -90
            materials: PrincipledMaterial { baseColor: "#ffcc00" }
        }
    }
}
