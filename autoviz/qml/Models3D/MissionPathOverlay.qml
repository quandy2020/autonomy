import QtQuick
import QtQuick3D

Node {
    id: root

    property var missionPlan: null

    Repeater3D {
        id: waypointRepeater
        model: root.missionPlan ? root.missionPlan.count : 0

        Node {
            property real yawRad: root.missionPlan.waypointYaw(index)
            property var pos: root.missionPlan.waypointPosition(index)
            property bool isNext: root.missionPlan.isNextGoal(index)

            position: pos
            eulerRotation.z: yawRad * 180 / Math.PI

            Model {
                source: "#Cone"
                scale: isNext ? Qt.vector3d(0.2, 0.32, 0.2) : Qt.vector3d(0.16, 0.24, 0.16)
                position: Qt.vector3d(isNext ? 0.36 : 0.32, 0, isNext ? 0.14 : 0.12)
                eulerRotation.x: -90
                materials: [
                    DefaultMaterial {
                        diffuseColor: isNext ? "#66ff99" : "#ffcc00"
                        specularAmount: isNext ? 0.35 : 0.25
                    }
                ]
            }

            Model {
                source: "#Cylinder"
                scale: Qt.vector3d(0.03, isNext ? 0.48 : 0.42, 0.03)
                position: Qt.vector3d(isNext ? 0.24 : 0.21, 0, 0.05)
                eulerRotation.x: 90
                materials: [
                    DefaultMaterial {
                        diffuseColor: isNext ? "#99ffcc" : "#66c8ff"
                        specularAmount: 0.15
                    }
                ]
            }
        }
    }

    Repeater3D {
        id: segmentRepeater
        model: root.missionPlan && root.missionPlan.count > 1
               ? root.missionPlan.count - 1
               : 0

        Node {
            property var p0: root.missionPlan.waypointPosition(index)
            property var p1: root.missionPlan.waypointPosition(index + 1)
            property real dx: p1.x - p0.x
            property real dy: p1.y - p0.y
            property real dz: p1.z - p0.z
            property real len: Math.sqrt(dx * dx + dy * dy + dz * dz)

            position: Qt.vector3d(
                (p0.x + p1.x) * 0.5,
                (p0.y + p1.y) * 0.5,
                (p0.z + p1.z) * 0.5 + 0.05)

            eulerRotation.z: Math.atan2(dy, dx) * 180 / Math.PI
            eulerRotation.x: len > 1e-6 ? -Math.asin(dz / len) * 180 / Math.PI : 0

            Model {
                source: "#Cylinder"
                scale: Qt.vector3d(0.04, len > 1e-6 ? len : 0.01, 0.04)
                materials: [
                    DefaultMaterial {
                        diffuseColor: "#ff8c00"
                        specularAmount: 0.1
                    }
                ]
            }
        }
    }
}
