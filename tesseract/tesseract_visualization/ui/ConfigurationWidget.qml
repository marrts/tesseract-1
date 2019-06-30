import QtQuick 2.4

ConfigurationWidgetForm {
    signal fwdKinematicsIndexChanged(int index)
    signal fwdKinematicsTextChanged(string txt)

    fwdKinematicsComboBox.onCurrentIndexChanged: { fwdKinematicsIndexChanged(fwdKinematicsComboBox.currentIndex)}
    fwdKinematicsComboBox.onCurrentTextChanged: { fwdKinematicsTextChanged(fwdKinematicsComboBox.currentText) }
}
