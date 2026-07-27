import QtQuick
import QtQuick3D

// Procedural quadcopter when QGC F450 .mesh assets are unavailable.
Node {
    id: body

    property var droneState

    function finiteOr(value, fallback) {
        return Number.isFinite(value) ? value : fallback
    }

    property int _angleAnimationDuration: 100
    property int _poseAnimationDuration: 200
    property double heading: droneState ? finiteOr(droneState.yaw, 0) : 0
    property double pitch: droneState ? finiteOr(droneState.pitch, 0) : 0
    property double roll: droneState ? finiteOr(droneState.roll, 0) : 0
    property double pose_x: droneState ? finiteOr(droneState.x, 0) : 0
    property double pose_y: droneState ? finiteOr(droneState.y, 0) : 0
    property double pose_z: droneState ? finiteOr(droneState.z, 0) : 0
    property int flightMode: droneState ? droneState.flightMode : 0

    rotation: Quaternion.fromEulerAngles(Qt.vector3d(0, 0, (90 - body.heading)))

    Behavior on rotation {
        QuaternionAnimation {
            duration: _angleAnimationDuration
            easing.type: Easing.Linear
        }
    }

    position: Qt.vector3d(body.pose_x, body.pose_y, body.pose_z)

    Behavior on position {
        Vector3dAnimation {
            duration: _poseAnimationDuration
            easing.type: Easing.Linear
        }
    }

    Node {
        id: rollPitchRotationNode

        rotation: Quaternion.fromEulerAngles(Qt.vector3d(body.roll, -body.pitch, 0))

        Behavior on rotation {
            QuaternionAnimation {
                duration: _angleAnimationDuration
                easing.type: Easing.Linear
            }
        }

        Node {
            scale: Qt.vector3d(0.08, 0.08, 0.08)

            Model {
                source: "#Cube"
                scale: Qt.vector3d(1.2, 1.2, 0.35)
                materials: PrincipledMaterial {
                    baseColor: "#333333"
                    metalness: 0.6
                    roughness: 0.25
                }
            }

            Repeater3D {
                model: 4
                Node {
                    property real angle: index * 90
                    eulerRotation.z: angle
                    position: Qt.vector3d(18, 0, 0)

                    Model {
                        source: "#Cylinder"
                        scale: Qt.vector3d(0.15, 3.5, 0.15)
                        eulerRotation.z: 90
                        materials: PrincipledMaterial {
                            baseColor: index === 0 ? "#ffffff" : "#cc3333"
                            metalness: 0.2
                            roughness: 0.3
                        }
                    }

                    Model {
                        source: "#Cylinder"
                        position: Qt.vector3d(14, 0, 0)
                        scale: Qt.vector3d(0.35, 0.08, 0.35)
                        materials: PrincipledMaterial {
                            baseColor: "#111111"
                            metalness: 0.8
                        }
                    }

                    Node {
                        position: Qt.vector3d(14, 0, 0)
                        PropertyAnimation on eulerRotation.y {
                            from: 0
                            to: (index % 2 === 0) ? 360 : -360
                            duration: flightMode > 1 ? 300 : 1000
                            loops: Animation.Infinite
                            running: flightMode > 0
                        }
                        Model {
                            source: "#Cylinder"
                            scale: Qt.vector3d(1.2, 0.03, 0.12)
                            materials: PrincipledMaterial {
                                baseColor: "#888888"
                                metalness: 0.1
                                roughness: 0.5
                            }
                        }
                    }
                }
            }
        }
    }
}
