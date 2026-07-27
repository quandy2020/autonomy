import QtQuick
import QtQuick3D

// Shared TF pose animation (QGC DroneModel / aviz ground models).
Node {
    id: root

    property var vehicleState: null

    function finiteOr(value, fallback) {
        return Number.isFinite(value) ? value : fallback
    }

    readonly property var state: vehicleState

    property alias bodyNode: body
    property int angleAnimationDuration: 100
    property int poseAnimationDuration: 200
    property bool clampGroundZ: false
    property double heading: state ? finiteOr(state.yaw, 0) : 0
    property double pitch: state ? finiteOr(state.pitch, 0) : 0
    property double roll: state ? finiteOr(state.roll, 0) : 0
    property double pose_x: state ? finiteOr(state.x, 0) : 0
    property double pose_y: state ? finiteOr(state.y, 0) : 0
    property double pose_z: {
        const z = state ? finiteOr(state.z, 0) : 0
        return root.clampGroundZ ? Math.max(0, z) : z
    }
    property int motionMode: state ? state.motionMode : 0
    property double linearSpeed: state ? state.linearSpeed : 0
    property double angularSpeed: state ? state.angularSpeed : 0

    default property alias contents: body.defaultProperty

    rotation: Quaternion.fromEulerAngles(Qt.vector3d(0, 0, (90 - root.heading)))

    Behavior on rotation {
        QuaternionAnimation {
            duration: root.angleAnimationDuration
            easing.type: Easing.Linear
        }
    }

    position: Qt.vector3d(root.pose_x, root.pose_y, root.pose_z)

    Behavior on position {
        Vector3dAnimation {
            duration: root.poseAnimationDuration
            easing.type: Easing.Linear
        }
    }

    Node {
        id: body
        rotation: Quaternion.fromEulerAngles(Qt.vector3d(root.roll, -root.pitch, 0))

        Behavior on rotation {
            QuaternionAnimation {
                duration: root.angleAnimationDuration
                easing.type: Easing.Linear
            }
        }
    }
}
