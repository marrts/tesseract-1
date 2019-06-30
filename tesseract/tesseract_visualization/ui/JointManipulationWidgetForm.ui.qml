import QtQuick 2.4
import QtQuick.Controls 2.2
import QtQuick.Controls.Styles.Breeze 1.0
import QtQuick.Extras 1.4

Item {
    id: element
    width: 500
    height: 400

    property alias jointSpinBox: jointSpinBox
    property alias jointLabelText: jointLabel.text
    property alias jointSlider: jointSlider
    property alias jointSliderFrom: jointSlider.from
    property alias jointSliderTo: jointSlider.to
    property alias jointSliderStepSize: jointSlider.stepSize

    Slider {
        id: jointSlider
        y: 8
        to: 99
        stepSize: 1.00
        antialiasing: false
        anchors.verticalCenter: jointLabel.verticalCenter
        anchors.left: jointLabel.right
        anchors.leftMargin: 20
        value: 0.5
    }

    SpinBox {
        id: jointSpinBox
        y: 8
        stepSize: jointSlider.stepSize
        to: jointSlider.to
        from: jointSlider.from
        editable: true
        anchors.verticalCenter: jointSlider.verticalCenter
        anchors.left: jointSlider.right
        anchors.leftMargin: 10
    }

    Label {
        id: jointLabel
        x: 8
        y: 26
        width: 92
        height: 27
        text: qsTr("Joint:")
        font.pointSize: 15
        anchors.right: parent.left
        anchors.rightMargin: -100
    }
}






/*##^## Designer {
    D{i:1;anchors_x:130}D{i:2;anchors_x:352}D{i:3;anchors_x:39}
}
 ##^##*/
