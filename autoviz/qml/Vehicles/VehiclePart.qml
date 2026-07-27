import QtQuick
import QtQuick3D

Node {
    id: root

    required property url meshSource
    property color baseColor: "gray"
    property real metalness: 0.1
    property real roughness: 0.25

    Model {
        source: root.meshSource
        materials: PrincipledMaterial {
            baseColor: root.baseColor
            metalness: root.metalness
            opacity: 1.0
            roughness: root.roughness
            specularAmount: 0.8
        }
    }
}
