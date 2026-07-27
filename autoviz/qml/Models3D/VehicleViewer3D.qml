import QtQuick
import QtQuick.Controls
import QtQuick3D

import Autoviz.Vehicle3D

Item {
    id: root

    signal sourceFrameChanged(string frame)
    signal modelTypeChanged(int modelType)
    signal resetOriginRequested()

    property var vehicleState: autovizVehicleState
    property bool followVehicle: false

    readonly property bool isGroundModel: vehicleState
        ? (vehicleState.modelType === 0 || vehicleState.modelType === 1)
        : true
    readonly property bool isDroneModel: vehicleState
        ? (vehicleState.modelType === 2 || vehicleState.modelType === 3)
        : false

    View3D {
        id: view3d

        anchors.fill: parent
        focus: true

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#141824"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
        }

        Node {
            id: cameraRigRoot

            position: root.followVehicle && root.vehicleState && root.vehicleState.valid
                      ? Qt.vector3d(root.vehicleState.x, root.vehicleState.y, root.vehicleState.z)
                      : Qt.vector3d(0, 0, 0)

            Behavior on position {
                Vector3dAnimation {
                    duration: 180
                    easing.type: Easing.Linear
                }
            }

            CameraLightModel {
                id: rig
                viewDistance: root.isGroundModel ? 8000 : 5000
            }
        }

        GroundPlane {
            visible: root.isGroundModel
            extent: 120
        }

        Node {
            id: vehicleRoot

            MissionPathOverlay {
                missionPlan: autovizMissionPlan
            }

            Loader3D {
                id: modelLoader
                sourceComponent: {
                    if (!root.vehicleState)
                        return diffDriveComponent
                    switch (root.vehicleState.modelType) {
                    case 0: return diffDriveComponent
                    case 1: return ackermannComponent
                    case 3:
                        return root.vehicleState.useF450Mesh ? f450Component : simpleDroneComponent
                    case 2:
                    default:
                        return simpleDroneComponent
                    }
                }
            }

            Component {
                id: diffDriveComponent
                GroundModelDiffDrive { vehicleState: root.vehicleState }
            }

            Component {
                id: ackermannComponent
                GroundModelAckermann { vehicleState: root.vehicleState }
            }

            Component {
                id: simpleDroneComponent
                DroneModelSimple { vehicleState: root.vehicleState }
            }

            Component {
                id: f450Component
                DroneModelDjiF450 { vehicleState: root.vehicleState }
            }
        }

        property bool keyW: false
        property bool keyS: false
        property bool keyA: false
        property bool keyD: false
        property bool keyQ: false
        property bool keyE: false
        property bool keyShift: false

        Keys.onPressed: function(event) {
            switch (event.key) {
            case Qt.Key_W: keyW = true; break
            case Qt.Key_S: keyS = true; break
            case Qt.Key_A: keyA = true; break
            case Qt.Key_D: keyD = true; break
            case Qt.Key_Q: keyQ = true; break
            case Qt.Key_E: keyE = true; break
            }
            if (event.modifiers & Qt.ShiftModifier)
                keyShift = true
            event.accepted = true
        }

        Keys.onReleased: function(event) {
            switch (event.key) {
            case Qt.Key_W: keyW = false; break
            case Qt.Key_S: keyS = false; break
            case Qt.Key_A: keyA = false; break
            case Qt.Key_D: keyD = false; break
            case Qt.Key_Q: keyQ = false; break
            case Qt.Key_E: keyE = false; break
            }
            if (!(event.modifiers & Qt.ShiftModifier))
                keyShift = false
        }

        Timer {
            interval: 16
            repeat: true
            running: view3d.keyW || view3d.keyS || view3d.keyA || view3d.keyD
                    || view3d.keyQ || view3d.keyE
            onTriggered: {
                const spd = (view3d.keyShift
                             ? (root.isGroundModel ? 240 : 150)
                             : (root.isGroundModel ? 80 : 50)) * 0.016
                let delta = Qt.vector3d(0, 0, 0)
                if (view3d.keyW)
                    delta.y -= spd
                if (view3d.keyS)
                    delta.y += spd
                if (view3d.keyA)
                    delta.x -= spd
                if (view3d.keyD)
                    delta.x += spd
                if (view3d.keyQ)
                    delta.z -= spd
                if (view3d.keyE)
                    delta.z += spd
                rig.camera.position = rig.camera.position.plus(delta)
            }
        }

        DragHandler {
            target: null
            onActiveChanged: if (active) rig.camera.eulerRotation.z -= translation.x * 0.15
            onTranslationChanged: {
                rig.camera.eulerRotation.x = Math.max(-80, Math.min(-5,
                    rig.camera.eulerRotation.x - translation.y * 0.08))
            }
        }

        WheelHandler {
            onWheel: {
                const factor = 1.0 + (wheel.angleDelta.y / 1200.0)
                rig.camera.position = rig.camera.position.times(factor)
            }
        }
    }

    onIsGroundModelChanged: resetCamera()

    function resetCamera() {
        rig.camera.position = root.isGroundModel
                          ? Qt.vector3d(0, -180, 70)
                          : Qt.vector3d(0, -120, 55)
        rig.camera.eulerRotation.x = root.isGroundModel ? -28 : -22
    }

    Component.onCompleted: resetCamera()

    Rectangle {
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.top: parent.top
        height: 36
        color: "#80000000"

        Row {
            anchors.fill: parent
            anchors.margins: 6
            spacing: 8

            Text {
                color: "#cccccc"
                text: qsTr("Model:")
                anchors.verticalCenter: parent.verticalCenter
            }

            ComboBox {
                id: modelTypeCombo
                anchors.verticalCenter: parent.verticalCenter
                implicitWidth: 150
                model: [
                    qsTr("Ground (diff-drive)"),
                    qsTr("Ground (Ackermann)"),
                    qsTr("Drone (simple)"),
                    qsTr("Drone (F450 mesh)")
                ]
                currentIndex: root.vehicleState ? root.vehicleState.modelType : 0
                onActivated: root.modelTypeChanged(currentIndex)
            }

            Text {
                color: "#cccccc"
                text: qsTr("TF frame:")
                anchors.verticalCenter: parent.verticalCenter
            }

            TextField {
                id: frameField
                text: "base_link"
                color: "#eeeeee"
                selectByMouse: true
                anchors.verticalCenter: parent.verticalCenter
                implicitWidth: 120
                onEditingFinished: root.sourceFrameChanged(text)
            }

            CheckBox {
                text: qsTr("Follow")
                checked: root.followVehicle
                onCheckedChanged: root.followVehicle = checked
                anchors.verticalCenter: parent.verticalCenter
            }

            ToolButton {
                text: qsTr("Origin")
                anchors.verticalCenter: parent.verticalCenter
                onClicked: root.resetOriginRequested()
            }

            Text {
                color: root.vehicleState && root.vehicleState.valid ? "#66dd66" : "#dd6666"
                text: root.vehicleState && root.vehicleState.valid
                      ? qsTr("OK %1").arg(root.vehicleState.label)
                      : qsTr("No TF")
                anchors.verticalCenter: parent.verticalCenter
            }

            Text {
                color: "#999999"
                text: {
                    if (!root.vehicleState || !root.vehicleState.valid)
                        return ""
                    const v = root.vehicleState.linearSpeed.toFixed(2)
                    const w = root.vehicleState.angularSpeed.toFixed(0)
                    return qsTr("%1 m/s · %2°/s").arg(v).arg(w)
                }
                anchors.verticalCenter: parent.verticalCenter
            }
        }
    }
}
