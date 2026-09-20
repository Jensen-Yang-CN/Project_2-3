import QtQuick 2.0

Rectangle {
    id:btnRoot
    width: 120
    height: 50
    color: mouseArea.pressed ? "#ddd" : "#eee"
    property alias text: btntext.text
    //property alias color:btntext.color
    radius: 20
    signal clicked(string context);

    Text {
        id: btntext
        text: qsTr(" 按钮")
        color:"white"
        anchors.centerIn: parent
    }
    MouseArea {
            id: mouseArea
            anchors.fill: parent

            // 2. 在点击时触发这个信号
            onClicked: {
                //btnRoot.clicked()
                myDialog.showWithParam(btntext.text)
            }
        }

}
