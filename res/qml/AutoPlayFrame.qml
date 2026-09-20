import QtQuick 2.12
import QtQuick.Window 2.12
import QtGraphicalEffects 1.0    //qt5.12
import QtQuick.Controls 2.12
//import QtMultimedia 5.12         //qt5.12

import QtMultimedia 5.14

Window {
    visible: true
    id:root
    width: 1080
    height: 700
    title: qsTr("Hello World")

    Rectangle{
        id:frameTop
        width:root.width
        height: 150
        border.color: "#04AA99"
        color:"#04AA99"
        radius: 20
        property int spacing1 : 20
        //spacging: 40
        ButtonUser{
            id:anchorsbtn
            text: "在线"
            color: "#156082"
            x:20
            y:15
            onClicked: {
                myDialog.open()
            }
        }
        ButtonUser{
            id:btn2
            text: "加载离线数据回放"
            color: "#156082"
            x: anchorsbtn.x + anchorsbtn.width +  frameTop.spacing1
            y:anchorsbtn.y
            onClicked: {
                //console.log("spacing = " + spacing)
                //myDialog.open()

                //saveData(); // 调用你的函数
            }
        }
        ButtonUser{
            id:btn3
            text: "检测开关"
            color: "#156082"
            x: btn2.x + btn2.width +  frameTop.spacing1
            y:anchorsbtn.y
            onClicked: {
                //myDialog.open()
                //saveData(); // 调用你的函数
            }
        }
        ButtonUser{
            id:btn4
            text: "左视图"
            color: "#156082"
            x: btn3.x + btn3.width + frameTop.spacing1
            y:anchorsbtn.y
            onClicked: {
                //myDialog.open()
                //saveData(); // 调用你的函数
            }
        }
        ButtonUser{
            id:btn5
            text: "右视图"
            color: "#156082"
            x: btn4.x + btn4.width +  frameTop.spacing1
            y:anchorsbtn.y
            onClicked: {
                //myDialog.open()
                //saveData(); // 调用你的函数
            }
        }
        ButtonUser{
            id:btn6
            text: "前视图"
            color: "#156082"
            x: btn5.x + btn5.width +  frameTop.spacing1
            y:anchorsbtn.y
            onClicked: {
                //myDialog.open()
                //saveData(); // 调用你的函数
            }
        }
        ButtonUser{
            id:btn7
            text: "俯视图"
            color: "#156082"
            x: btn6.x + btn6.width +  frameTop.spacing1
            y:anchorsbtn.y
            onClicked: {
                //myDialog.open(btn7.text)
                //saveData(); // 调用你的函数
            }
        }
        Rectangle{
            id:text1trect
            width:32
            height: 20
            color: "transparent"
            border.color: "transparent"
            Text{
                anchors.fill: parent
                text:"泊位"
                color: "white"
                font.pixelSize: 16
            }
            anchors.margins: 2
            anchors.right:hbtn1.left
            anchors.bottom: hbtn1.bottom
        }

        Hbutton{
            id:hbtn1
            anchors.horizontalCenter:  anchorsbtn.horizontalCenter
            y:anchorsbtn.y +  anchorsbtn.height + frameTop.spacing1
        }
        Rectangle{
            id:text2trect
            width:32
            height: 20
            color: "transparent"
            border.color: "transparent"
            Text{
                anchors.fill: parent
                text:"桥"
                color: "white"
                font.pixelSize: 16
            }
            anchors.margins: 2
            anchors.right:hbtn2.left
            anchors.bottom: hbtn2.bottom
        }

        Hbutton{
            id:hbtn2
            anchors.horizontalCenter:  btn2.horizontalCenter
            y:anchorsbtn.y +  btn2.height + frameTop.spacing1
        }
        Rectangle{
            id:text3trect
            width:32
            height: 20
            color: "transparent"
            border.color: "transparent"
            Text{
                anchors.fill: parent
                text:"船只"
                color: "white"
                font.pixelSize: 16
            }
            anchors.margins: 2
            anchors.right:hbtn3.left
            anchors.bottom: hbtn3.bottom
        }
        Hbutton{
            id:hbtn3
            anchors.horizontalCenter:  btn3.horizontalCenter
            y:anchorsbtn.y +  btn3.height + frameTop.spacing1
        }
        Rectangle{
            id:text4trect
            width:32
            height: 20
            color: "transparent"
            border.color: "transparent"
            Text{
                anchors.fill: parent
                text:"航标"
                color: "white"
                font.pixelSize: 16
            }
            anchors.margins: 2
            anchors.right:hbtn4.left
            anchors.bottom: hbtn4.bottom
        }
        Hbutton{
            id:hbtn4
            anchors.horizontalCenter:  btn4.horizontalCenter
            y:anchorsbtn.y +  btn4.height + frameTop.spacing1
        }
        Rectangle{
            id:text5trect
            width:32
            height: 20
            color: "transparent"
            border.color: "transparent"
            Text{
                anchors.fill: parent
                text:"浮筒"
                color: "white"
                font.pixelSize: 16
            }
            anchors.margins: 2
            anchors.right:hbtn5.left
            anchors.bottom: hbtn5.bottom
        }
        Hbutton{
            id:hbtn5
            anchors.horizontalCenter:  btn5.horizontalCenter
            y:anchorsbtn.y +  btn5.height + frameTop.spacing1
        }
        Rectangle{
            id:text6trect
            width:50
            height: 20
            color: "transparent"
            border.color: "transparent"
            Text{
                anchors.fill: parent
                text:"障碍物"
                color: "white"
                font.pixelSize: 16
            }
            anchors.margins: 2
            anchors.right:hbtn6.left
            anchors.bottom: hbtn6.bottom
        }
        Hbutton{
            id:hbtn6
            anchors.horizontalCenter:  btn6.horizontalCenter
            y:anchorsbtn.y +  btn6.height + frameTop.spacing1
        }
    }

    Rectangle{
        id:frameLeft
        width: root.width*0.65
        height: (root.height-frameTop.height)*.65
        color: "black"

        anchors.left:root.left
        anchors.top:frameTop.bottom
        radius: 40
        Image {
            id: img
            source: "qrc:/res/pic/g1.png"
            anchors.fill: parent
            fillMode: Image.PreserveAspectCrop
            clip: true

        }


    }

    Rectangle{
        id:videoleft
        width: frameLeft.width/2
        height: root.height-frameTop.height-frameLeft.height
        color: "lightgray"
        anchors.left: root.left
        anchors.top:frameLeft.bottom
        radius: 30
        Image {
            id: v1
            source: "qrc:/res/pic/v1.png"
            anchors.fill: parent
        }
        MediaPlayer {
            id: player1
            source: "file:///e:/qtapp/AutoPlayFrame/1.mp4"   // 注意：Windows路径需用正斜杠
            // 状态监听：当媒体加载完成时自动播放
            loops: MediaPlayer.Infinite
            onStatusChanged: console.log("videoleft Current Status: " + status)
            onError: console.log("videoleft Error: " + errorString)

        }
        VideoOutput {
            id: videoOutput1
            source: player1
            anchors.fill: parent
            fillMode: VideoOutput.PreserveAspectCrop
            // 确保它没有被隐藏
            visible: true
        }
        // 重点：组件加载完成后强制播放
        Component.onCompleted: {
            //console.log("QML Completed, calling play...")
            player1.play()
        }


    }
    Rectangle{
        id:videoright
        width: frameLeft.width/2
        height: root.height-frameTop.height-frameLeft.height
        color: "lightgray"
        anchors.left: videoleft.right
        anchors.top:frameLeft.bottom
        radius: 30
        MediaPlayer {
            id: player
            source: "file:///e:/qtapp/AutoPlayFrame/3c.mp4"   // 注意：Windows路径需用正斜杠
            // 状态监听：当媒体加载完成时自动播放
            loops: MediaPlayer.Infinite
            onStatusChanged: console.log("videoright Current Status: " + status)
            onError: console.log("videoright Error: " + errorString)

        }
        VideoOutput {
            id: videoOutput
            source: player
            anchors.fill: parent
            fillMode: VideoOutput.PreserveAspectCrop
            // 确保它没有被隐藏
            visible: true
        }

        // 重点：组件加载完成后强制播放
        Component.onCompleted: {
            //console.log("QML Completed, calling play...")
            player.play()
        }

    }
    Rectangle{
        id:frameRight
        width: root.width-frameLeft.width
        height:root.height-frameTop.height
        color:"#04A898"
        anchors.left: frameLeft.right
        anchors.top: frameTop.bottom
        Rectangle{
            id:right1
            width:root.width-frameLeft.width
            height: (root.height-frameTop.height)*0.25
            anchors.left: parent.left
            anchors.top: parent.top
            color: "#156082"
            radius: 40


        }
        Rectangle{
            id:right2
            width:root.width-frameLeft.width
            height: (root.height-frameTop.height)*0.25
            anchors.top: right1.bottom
            border.color: "#04A898"
            color: "#156082"
            radius: 40

        }
        Rectangle{
            id:right3
            width:root.width-frameLeft.width
            height: (root.height-frameTop.height)*0.25
            anchors.top: right2.bottom
            border.color: "#04A898"
            color: "#156082"
            radius: 40

        }
        Rectangle{
            id:right4
            width:root.width-frameLeft.width
            height: (root.height-frameTop.height)*0.25
            anchors.top: right3.bottom
            border.color: "#04A898"
            color: "#156082"
            radius: 40

        }

    }

    // 定义对话框
    Dialog {
        id: myDialog
        title: "系统确认"

        // 定义一个内部属性来保存传入的文字
        property string currentAction: ""

        x: (parent.width - width) / 2
        y: (parent.height - height) / 2
        modal: true
        standardButtons: Dialog.Ok | Dialog.Cancel

        // --- 核心修改：定义一个带参数的方法 ---
        function showWithParam(btnText) {
            currentAction = btnText ; // 将按钮传来的参数存起来
            myDialog.open();         // 然后打开对话框
        }

        Column {
            spacing: 10
            width: parent.width
            Text {
                // 使用传入的参数动态显示内容 + + frameTop.spacing1
                text: "您点击了：" + myDialog.currentAction +"\n确定要执行此操作吗？"
                font.pixelSize: 14
            }
        }

        onAccepted: {
            console.log("正在执行与 " + currentAction + " 相关的逻辑");
            // 这里可以根据 currentAction 执行不同的 saveData() 逻辑
        }
    }
}
