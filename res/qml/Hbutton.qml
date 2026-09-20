import QtQuick 2.0
import QtQuick.Controls 2.0

Item {
    id: root
    width: 80
    height: 32

    // 自定义属性：当前状态是否在右侧
    property bool isRight: false

    Rectangle {
        id: background
        anchors.fill: parent
        color: root.isRight ? "#4CD964" : "#E5E5EA" // 状态颜色切换
        radius: height / 2
        border.color: "#D1D1D6"
        border.width: 1

        // 滑块
        Rectangle {
            id: handle
            width: parent.height - 4
            height: width
            radius: width / 2
            color: "white"
            y: 2
            x: 2 // 初始位置（左侧）

            layer.enabled: true // 开启阴影效果更具质感
        }

        MouseArea {
            anchors.fill: parent
            // 点击切换状态
            onClicked: root.isRight = !root.isRight

            // 支持左右滑动的逻辑
            drag.target: handle
            drag.axis: Drag.XAxis
            drag.minimumX: 2
            drag.maximumX: background.width - handle.width - 2

            // 松开手指时，根据滑块位置自动吸附
            onReleased: {
                if (handle.x > background.width / 2 - handle.width / 2) {
                    root.isRight = true;
                    myDialog.showWithParam("isRight")
                } else {
                    root.isRight = false;
                    myDialog.showWithParam("isLeft")
                }
            }
        }
    }

    // 状态定义
    states: [
        State {
            name: "left"
            when: !root.isRight
            PropertyChanges { target: handle; x: 2 }
        },
        State {
            name: "right"
            when: root.isRight
            PropertyChanges { target: handle; x: background.width - handle.width - 2 }
        }
    ]

    // 动画效果
    transitions: Transition {
        NumberAnimation {
            properties: "x"
            duration: 200
            easing.type: Easing.InOutQuad
        }
        // 背景颜色平滑过渡
        ColorAnimation { duration: 200 }
    }
}
