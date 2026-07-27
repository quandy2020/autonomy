import QtQuick
import QtQuick3D

Node {
    id: root

    property real extent: 120
    property int divisions: 12
    property color lineColor: "#334455"
    property color fillColor: "#1a2030"

    Model {
        source: "#Plane"
        scale: Qt.vector3d(root.extent / 100, root.extent / 100, 1)
        eulerRotation.x: -90
        materials: PrincipledMaterial {
            baseColor: root.fillColor
            metalness: 0.0
            roughness: 0.95
        }
    }

    Repeater3D {
        model: root.divisions * 2 + 1
        Model {
            property real t: (index - root.divisions) * (root.extent / root.divisions)
            source: "#Cube"
            scale: Qt.vector3d(root.extent / 50, 0.02, 0.02)
            position: Qt.vector3d(0, t, 0.01)
            materials: PrincipledMaterial {
                baseColor: root.lineColor
                metalness: 0.0
            }
        }
    }

    Repeater3D {
        model: root.divisions * 2 + 1
        Model {
            property real t: (index - root.divisions) * (root.extent / root.divisions)
            source: "#Cube"
            scale: Qt.vector3d(0.02, root.extent / 50, 0.02)
            position: Qt.vector3d(t, 0, 0.01)
            materials: PrincipledMaterial {
                baseColor: root.lineColor
                metalness: 0.0
            }
        }
    }
}
