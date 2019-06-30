import QtQuick 2.4
import QtQuick.Controls 2.2
import QtQuick.Extras 1.4
import QtQuick.Controls.Material 2.0
import QtQuick.XmlListModel 2.0

Item {
    id: element
    width: 498
    height: 400

    property alias fwdKinematicsComboBox: fwdKinematicsComboBox

    ComboBox {
        id: fwdKinematicsComboBox
        width: 241
        height: 30
        anchors.left: parent.left
        anchors.leftMargin: 160
        anchors.top: parent.top
        anchors.topMargin: 30
        model: fwdKinematicsComboBoxModel
        textRole: "display"
    }

    ComboBox {
        id: invKinematicsComboBox
        width: 241
        height: 30
        anchors.left: fwdKinematicsComboBox.left
        anchors.leftMargin: 0
        anchors.top: fwdKinematicsComboBox.bottom
        anchors.topMargin: 30
    }

    ComboBox {
        id: discreteCollisionComboBox
        width: 241
        height: 30
        anchors.left: fwdKinematicsComboBox.left
        anchors.leftMargin: 0
        anchors.top: invKinematicsComboBox.bottom
        anchors.topMargin: 30
    }

    ComboBox {
        id: continuousCollisionComboBox
        width: 241
        height: 30
        anchors.left: fwdKinematicsComboBox.left
        anchors.leftMargin: 0
        anchors.top: discreteCollisionComboBox.bottom
        anchors.topMargin: 30
    }

    Label {
        id: fwdKinLabel
        height: 17
        text: qsTr("Forward Kinematics:")
        anchors.verticalCenter: fwdKinematicsComboBox.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 10
    }

    Label {
        id: invKinLabel
        text: qsTr("Inverse Kinematics:")
        anchors.verticalCenter: invKinematicsComboBox.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 10
    }

    Label {
        id: discreteCollisionLabel
        text: qsTr("Discrete Collision:")
        anchors.verticalCenter: discreteCollisionComboBox.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 10
    }

    Label {
        id: continuousCollisionLabel
        text: qsTr("Continuous Collision:")
        anchors.verticalCenter: continuousCollisionComboBox.verticalCenter
        anchors.left: parent.left
        anchors.leftMargin: 10
    }
}
