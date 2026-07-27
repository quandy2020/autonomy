import QtQuick
import QtQuick3D

import Autoviz.Vehicle3D

// Differential-drive ground robot (TurtleBot-class proportions).
VehiclePoseNode {
    id: root

    clampGroundZ: true
    scale: Qt.vector3d(8, 8, 8)

    readonly property real wheelSpinSign: root.linearSpeed >= 0 ? 1 : -1
    readonly property real wheelDuration: {
        if (root.motionMode <= 0)
            return 1200
        const speed = Math.max(0.05, Math.abs(root.linearSpeed))
        return Math.max(180, Math.min(1400, 400 / speed))
    }

    Node {
        parent: bodyNode

        Model {
            source: "#Cube"
            scale: Qt.vector3d(0.45, 0.32, 0.18)
            position: Qt.vector3d(0, 0, 0.12)
            materials: PrincipledMaterial {
                baseColor: "#2a5a8a"
                metalness: 0.35
                roughness: 0.4
            }
        }

        Model {
            source: "#Cube"
            scale: Qt.vector3d(0.45, 0.34, 0.04)
            position: Qt.vector3d(0, 0, 0.02)
            materials: PrincipledMaterial { baseColor: "#1a1a1a"; metalness: 0.15 }
        }

        Model {
            source: "#Cube"
            scale: Qt.vector3d(0.08, 0.28, 0.06)
            position: Qt.vector3d(0.24, 0, 0.22)
            materials: PrincipledMaterial { baseColor: "#111111"; metalness: 0.2 }
        }

        Repeater3D {
            model: 2
            Node {
                property real side: index === 0 ? -1 : 1
                position: Qt.vector3d(0, side * 0.22, 0.08)
                Model {
                    source: "#Cylinder"
                    scale: Qt.vector3d(0.12, 0.06, 0.12)
                    eulerRotation.x: 90
                    materials: PrincipledMaterial {
                        baseColor: "#222222"
                        metalness: 0.5
                        roughness: 0.35
                    }
                }
                PropertyAnimation on eulerRotation.z {
                    from: 0
                    to: side * 360 * root.wheelSpinSign
                    duration: root.wheelDuration
                    loops: Animation.Infinite
                    running: root.motionMode > 0
                }
            }
        }

        Model {
            source: "#Sphere"
            scale: Qt.vector3d(0.06, 0.06, 0.06)
            position: Qt.vector3d(-0.2, 0, 0.04)
            materials: PrincipledMaterial { baseColor: "#666666" }
        }

        Model {
            source: "#Cylinder"
            scale: Qt.vector3d(0.14, 0.04, 0.14)
            position: Qt.vector3d(0, 0, 0.32)
            materials: PrincipledMaterial {
                baseColor: "#444444"
                metalness: 0.6
            }
        }

        Model {
            source: "#Cone"
            scale: Qt.vector3d(0.06, 0.14, 0.06)
            position: Qt.vector3d(0.28, 0, 0.12)
            eulerRotation.z: -90
            materials: PrincipledMaterial {
                baseColor: "#ffcc00"
                metalness: 0.1
                roughness: 0.5
            }
        }
    }
}
