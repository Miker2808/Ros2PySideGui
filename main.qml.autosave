import QtQuick 2.15
import QtQuick.Window 2.15
import QtQuick.Controls 2.15
import QtMultimedia 5.15

Rectangle {
    id: root
    width: 1920
    height: 1080
    visible: true
    color: "#373737"


    property int screen_height: Screen.height
    property int screen_width: Screen.width
    property int speed_value: 20
    property int power_value: 0
    property int imu_speed_value: backend.prop_imu_speed
    FontLoader{
        id: font_digital
        source: "fonts/digital-7.ttf"
    }

    focus: true // required for keypressed to be registered
    Keys.onPressed: { //& !event.isAutoRepeat
        if (event.key === Qt.Key_W & !event.isAutoRepeat){
            //console.log("W Pressed")
            event.accepted = true
            backend.w_key_pressed()
            root.power_value = speed_value

        }
        if(event.key === Qt.Key_A & !event.isAutoRepeat){
            //console.log("A Pressed")
            event.accepted = true
            backend.a_key_pressed()
            textLeftSteerAction.opacity = 1.0

        }
        if(event.key === Qt.Key_S & !event.isAutoRepeat){
            //console.log("S Pressed")
            event.accepted = true
            backend.s_key_pressed()
            root.power_value = -1 * speed_value
        }
        if(event.key === Qt.Key_D & !event.isAutoRepeat){
            //console.log("D Pressed")
            event.accepted = true
            backend.d_key_pressed()
            textRightSteerAction.opacity = 1.0
        }

        if(event.key === Qt.Key_Shift & !event.isAutoRepeat){
            event.accepted = true
            backend.shift_key_pressed()
            if(speed_value < 100){
                root.speed_value += 20
            }
        }

        if(event.key === Qt.Key_Control & !event.isAutoRepeat){
            event.accepted = true
            backend.ctrl_key_pressed()
            if(speed_value > 0){
                root.speed_value -= 20
            }
        }

    }



    Keys.onReleased: { //& !event.isAutoRepeat
        if (event.key === Qt.Key_W & !event.isAutoRepeat){
            //console.log("W Released")
            event.accepted = true
            backend.w_key_released()
            root.power_value = 0

        }
        else if(event.key === Qt.Key_A & !event.isAutoRepeat){
            //console.log("A Released")
            event.accepted = true
            backend.a_key_released()
            textLeftSteerAction.opacity = 0

        }
        else if(event.key === Qt.Key_S & !event.isAutoRepeat){
            //console.log("S Released")
            event.accepted = true
            backend.s_key_released()
            root.power_value = 0

        }
        else if(event.key === Qt.Key_D & !event.isAutoRepeat){
            //console.log("D Released")
            event.accepted = true
            backend.d_key_released()
            textRightSteerAction.opacity = 0
        }
    }


    Item {
        enabled: false
        id: itemCameraFront
        width: root.width * 0.5
        height: root.height * 0.8
        anchors.left: root.left
        anchors.top: root.top
        anchors.leftMargin: 0
        anchors.topMargin: 0

        Rectangle {
            id: rectWebWindowFront
            anchors.fill: itemCameraFront
            color: "#4a4a4a"


            Video{
                id: videoCameraFront
                anchors.fill: parent
                autoLoad: true
                autoPlay: true
                source: "rtsp://10.0.40.202/axis-media/media.amp?camera=2"
            }
        }
    }


    Item {
        id: itemCameraRear
        width: root.width * 0.5
        height: root.height * 0.8
        anchors.right: root.right
        anchors.top: root.top
        anchors.rightMargin: 0
        anchors.topMargin: 0

        Rectangle {
            id: rectWebWindowRear
            color: "#4a4a4a"
            anchors.fill: itemCameraRear

            Video{
                id: videoCameraRear
                anchors.fill: parent
                autoLoad: true
                autoPlay: true
                source: "rtsp://10.0.40.202/axis-media/media.amp?camera=3"
            }
        }
    }

    Rectangle {
        id: rectDualViewButton
        width: root.width * 0.06
        height: root.height * 0.04
        color: "#00000000"
        border.color: "#ffffff"
        border.width: 2
        anchors.top: parent.top
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.topMargin: root.height * 0.015

        Text {
            id: textDualViewButton
            color: "#ffffff"
            text: qsTr("DUAL")
            anchors.verticalCenter: parent.verticalCenter
            font.pixelSize: 21 * (root.width / 1280)
            anchors.horizontalCenter: parent.horizontalCenter
        }

        MouseArea {
            id: mouseDualViewButton
            anchors.fill: parent

            onPressed: {
                // enable Front Camera and set positions
                videoCameraFront.source = "rtsp://10.0.40.202/axis-media/media.amp?camera=2"
                itemCameraFront.anchors.horizontalCenter = undefined
                itemCameraFront.visible = true
                itemCameraFront.enabled = true
                itemCameraFront.width = root.width * 0.5
                itemCameraFront.height = root.height * 0.8
                itemCameraFront.anchors.left =  root.left
                itemCameraFront.anchors.top = root.top
                itemCameraFront.anchors.leftMargin = 0
                itemCameraFront.anchors.topMargin = 0
                // enable rear camera and set positions
                videoCameraRear.source = "rtsp://10.0.40.202/axis-media/media.amp?camera=3"
                itemCameraRear.anchors.horizontalCenter = undefined
                itemCameraRear.visible = true
                itemCameraRear.enabled = true
                itemCameraRear.width = root.width * 0.5
                itemCameraRear.height = root.height * 0.8
                itemCameraRear.anchors.right =  root.right
                itemCameraRear.anchors.top = root.top
                itemCameraRear.anchors.rightMargin = 0
                itemCameraRear.anchors.topMargin = 0

                rectDualViewButton.color = "#7b7b7b"
            }
            onReleased: {
                rectDualViewButton.color = "#00000000"
            }
        }
    }

    Rectangle {
        id: rectRearViewButton
        x: 1036
        y: 934
        width: root.width * 0.06
        height: root.height * 0.04
        color: "#00000000"
        border.color: "#ffffff"
        border.width: 2
        anchors.top: parent.top
        anchors.horizontalCenterOffset: root.width * 0.07
        anchors.topMargin: root.height * 0.015
        anchors.horizontalCenter: parent.horizontalCenter
        Text {
            id: textRearViewButton
            color: "#ffffff"
            text: qsTr("REAR")
            anchors.verticalCenter: parent.verticalCenter
            font.pixelSize: 21 * (root.width / 1280)
            anchors.horizontalCenter: parent.horizontalCenter
        }

        MouseArea {
            id: mouseRearViewButton
            anchors.fill: parent
            onPressed: {
                // disable Front Camera
                videoCameraFront.source = ""
                itemCameraFront.visible = false
                itemCameraFront.enabled = false

                // enable rear camera and set positions
                videoCameraRear.source = "rtsp://10.0.40.202/axis-media/media.amp?camera=3"
                itemCameraRear.visible = true
                itemCameraRear.enabled = true
                itemCameraRear.width = root.width * 1
                itemCameraRear.height = root.height * 1
                itemCameraRear.anchors.horizontalCenter = root.horizontalCenter
                itemCameraRear.anchors.top = root.top
                itemCameraRear.anchors.topMargin = 0
                rectRearViewButton.color = "#7b7b7b"

            }
            onReleased: {
                rectRearViewButton.color = "#00000000"
            }
        }
    }

    Rectangle {
        id: rectFrontViewButton
        width: root.width * 0.06
        height: root.height * 0.04
        color: "#00000000"
        border.color: "#ffffff"
        border.width: 2
        anchors.top: parent.top
        anchors.horizontalCenterOffset: -(root.width * 0.07)
        anchors.topMargin: root.height * 0.015
        anchors.horizontalCenter: parent.horizontalCenter
        Text {
            id: textFrontViewButton
            color: "#ffffff"
            text: qsTr("FRONT")
            anchors.verticalCenter: parent.verticalCenter
            font.pixelSize: 21 * (root.width / 1280)
            anchors.horizontalCenter: parent.horizontalCenter
        }

        MouseArea {
            id: mouseFrontViewButton
            anchors.fill: parent
            onPressed: {
                // enable Front Camera and set positions
                videoCameraFront.source = "rtsp://10.0.40.202/axis-media/media.amp?camera=2"
                itemCameraFront.visible = true
                itemCameraFront.enabled = true
                itemCameraFront.width = root.width * 1
                itemCameraFront.height = root.height * 1
                itemCameraFront.anchors.horizontalCenter = root.horizontalCenter
                itemCameraFront.anchors.top = root.top
                itemCameraFront.anchors.topMargin = 0
                // enable rear camera and set positions
                videoCameraRear.source = ""
                itemCameraRear.visible = false
                itemCameraRear.enabled = false
                rectFrontViewButton.color = "#7b7b7b"

            }
            onReleased: {
                rectFrontViewButton.color = "#00000000"
            }
        }
    }
    /*
    Components.ArcItem {
        id: arcImuSpeedUnderlay
        x: 878
        y: -103
        width: root.height * 0.40
        height: root.height * 0.40
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.bottomMargin: -(root.height * 0.25)
        strokeColor: "#282828"
        begin: -90
        fillColor: "#282828"
        strokeWidth: 35
        end: 90
        dashOffset: 0
        antialiasing: true
    }
    */
    Text {

        property int textImuSpeed_value: root.imu_speed_value
        opacity: 1
        Behavior on textImuSpeed_value{
            NumberAnimation{
                duration: 200
            }
        }

        id: textPowerValue
        color: "#ffffff"
        text: textImuSpeed_value
        anchors.bottom: parent.bottom
        font.pixelSize: 51 * (root.width / 1280)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        anchors.bottomMargin: root.height * 0.02
        style: Text.Normal
        font.styleName: "Regular"
        font.weight: Font.Normal
        anchors.horizontalCenter: parent.horizontalCenter
        font.family: "Digital-7"
    }

    Text {
        property int textSpeedLimit_value: root.speed_value
        Behavior on textSpeedLimit_value{
            NumberAnimation{
                duration: 200
            }
        }

        id: textSpeedLimit
        color: "#2828e0"
        text: textSpeedLimit_value
        anchors.bottom: parent.bottom
        font.pixelSize: 31 * (root.width / 1280)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        anchors.horizontalCenter: parent.horizontalCenter
        font.family: "Digital-7"
        anchors.horizontalCenterOffset: -(root.width * 0.04)
        anchors.bottomMargin: root.height * 0.02
        font.styleName: "Regular"
        font.weight: Font.Normal
        style: Text.Normal
    }

    Text {
        property int textPower_value: root.power_value
        Behavior on textPower_value{
            NumberAnimation{
                duration: 100
            }
        }

        id: textImuValue
        color: "#e02828"
        text: textPower_value
        anchors.bottom: parent.bottom
        font.pixelSize: 31 * (root.width / 1280)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        anchors.horizontalCenterOffset: root.width * 0.04
        anchors.horizontalCenter: parent.horizontalCenter
        font.family: "Digital-7"
        anchors.bottomMargin: root.height * 0.02
        font.styleName: "Regular"
        style: Text.Normal
        font.weight: Font.Normal
    }

    /*
    Components.ArcItem {
        property int arc_value_speed: (root.imu_speed_value * 9) - 90

        Behavior on arc_value_speed {
            NumberAnimation{
                duration: 500
            }
        }

        id: arcImuSpeed
        width: root.height * 0.40
        height: root.height * 0.40
        anchors.bottom: parent.bottom
        dashOffset: 0
        antialiasing: true
        anchors.bottomMargin: -(root.height * 0.25)
        anchors.horizontalCenter: parent.horizontalCenter
        strokeWidth: 30
        begin: -90
        end: arc_value_speed
        strokeColor: "#ffffff"
        fillColor: "#00000000"
    }
    */

    Rectangle {
        id: rectPowerSpeedFrame
        width: root.width * 0.197
        height: root.height * 0.05
        color: "#00000000"
        border.color: "#e02828"
        border.width: 2
        anchors.bottom: parent.bottom
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.horizontalCenterOffset: root.width * 0.25
        anchors.bottomMargin: root.height * 0.01
        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectPower1
            width: root.width * 0.035
            opacity: (root.power_value >= 100 || root.power_value <= -20) ? 1.0 : 0.0
            color: "#e02828"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: root.width * 0.005
            anchors.bottomMargin: root.width * 0.005
            anchors.topMargin: root.width * 0.005
        }

        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectPower2
            width: root.width * 0.035
            opacity: (root.power_value >= 80 || root.power_value <= -40) ? 1.0 : 0.0
            color: "#e02828"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.bottomMargin: root.width * 0.005
            anchors.leftMargin: root.width * 0.043
            anchors.topMargin: root.width * 0.005
        }

        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectPower3
            width: root.width * 0.035
            opacity: (root.power_value >= 60 || root.power_value <= -60) ? 1.0 : 0.0
            color: "#e02828"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.bottomMargin: root.width * 0.005
            anchors.leftMargin: root.width * 0.081
            anchors.topMargin: root.width * 0.005
        }

        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectPower4
            width: root.width * 0.035
            opacity: (root.power_value >= 40 || root.power_value <= -80) ? 1.0 : 0.0
            color: "#e02828"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.bottomMargin: root.width * 0.005
            anchors.leftMargin: root.width * 0.119
            anchors.topMargin: root.width * 0.005
        }

        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectPower5
            width: root.width * 0.035
            opacity: (root.power_value >= 20 || root.power_value <= -100) ? 1.0 : 0.0
            color: "#e02828"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.bottomMargin: root.width * 0.005
            anchors.leftMargin: root.width * 0.157
            anchors.topMargin: root.width * 0.005
        }

        Text {
            id: textPowerName
            x: -960
            y: -40
            color: "#e02828"
            text: "POWER"
            anchors.right: parent.right
            anchors.bottom: parent.bottom
            font.pixelSize: 31 * (root.width / 1280)
            horizontalAlignment: Text.AlignRight
            verticalAlignment: Text.AlignVCenter
            anchors.rightMargin: 0
            font.weight: Font.Normal
            font.styleName: "Regular"
            style: Text.Normal
            anchors.bottomMargin: root.height * 0.055
            font.family: "Digital-7"
        }
    }

    Rectangle {
        id: rectSpeedLimitFrame
        width: root.width * 0.197
        height: root.height * 0.05
        color: "#00000000"
        border.color: "#2828e0"
        border.width: 4
        anchors.bottom: parent.bottom
        antialiasing: true
        anchors.bottomMargin: root.height * 0.01
        anchors.horizontalCenterOffset: -root.width * 0.25
        anchors.horizontalCenter: parent.horizontalCenter

        Rectangle {

            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }

            id: rectSpeedLimit1
            width: root.width * 0.035
            color: "#2828e0"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: root.width * 0.005
            anchors.topMargin: root.width * 0.005
            anchors.bottomMargin: root.width * 0.005
            opacity: (root.speed_value >= 20) ? 1.0 : 0.0
        }

        Rectangle {

            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectSpeedLimit2
            width: root.width * 0.035
            color: "#2828e0"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: root.width * 0.043
            anchors.bottomMargin: root.width * 0.005
            anchors.topMargin: root.width * 0.005
            opacity: (root.speed_value >= 40) ? 1.0 : 0.0
        }

        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectSpeedLimit3
            width: root.width * 0.035
            color: "#2828e0"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: root.width * 0.081
            anchors.bottomMargin: root.width * 0.005
            anchors.topMargin: root.width * 0.005
            opacity: (root.speed_value >= 60) ? 1.0 : 0.0
        }

        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectSpeedLimit4
            width: root.width * 0.035
            color: "#2828e0"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: root.width * 0.119
            anchors.bottomMargin: root.width * 0.005
            anchors.topMargin: root.width * 0.005
            opacity: (root.speed_value >= 80) ? 1.0 : 0.0
        }

        Rectangle {
            Behavior on opacity {
                NumberAnimation{
                    duration: 200
                }
            }
            id: rectSpeedLimit5
            width: root.width * 0.035
            color: "#2828e0"
            anchors.left: parent.left
            anchors.top: parent.top
            anchors.bottom: parent.bottom
            anchors.leftMargin: root.width * 0.157
            anchors.bottomMargin: root.width * 0.005
            anchors.topMargin: root.width * 0.005
            opacity: (root.speed_value >= 100) ? 1.0 : 0.0
        }

        Text {
            id: textSpeedLimitName
            color: "#2828e0"
            text: "LIMIT"
            anchors.left: parent.left
            anchors.bottom: parent.bottom
            font.pixelSize: 31 * (root.width / 1280)
            horizontalAlignment: Text.AlignLeft
            verticalAlignment: Text.AlignVCenter
            anchors.leftMargin: 0
            anchors.bottomMargin: root.height * 0.055
            font.styleName: "Regular"
            font.weight: Font.Normal
            style: Text.Normal
            font.family: "Digital-7"
        }
    }

    Text {
        Behavior on opacity {
            NumberAnimation{
                duration: 200
            }
        }

        id: textLeftSteerAction
        opacity: 0
        color: "#ffffff"
        text: "<-"
        anchors.bottom: parent.bottom
        font.pixelSize: 51 * (root.width / 1280)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.horizontalCenterOffset: -1 * root.width * 0.1
        font.family: "Digital-7"
        anchors.bottomMargin: root.height * 0.02
        style: Text.Normal
        font.styleName: "Regular"
        font.weight: Font.Normal
    }

    Text {
        Behavior on opacity {
            NumberAnimation{
                duration: 200
            }
        }

        id: textRightSteerAction
        opacity: 0
        color: "#ffffff"
        text: "->"
        anchors.bottom: parent.bottom
        font.pixelSize: 51 * (root.width / 1280)
        horizontalAlignment: Text.AlignHCenter
        verticalAlignment: Text.AlignVCenter
        anchors.horizontalCenterOffset: root.width * 0.1
        anchors.horizontalCenter: parent.horizontalCenter
        font.family: "Digital-7"
        anchors.bottomMargin: root.height * 0.02
        style: Text.Normal
        font.styleName: "Regular"
        font.weight: Font.Normal
    }


    Image {
        id: pngUserGuide
        visible: false
        anchors.left: parent.left
        anchors.right: parent.right
        anchors.bottom: parent.bottom
        source: "images/user-guide.png"
        anchors.leftMargin: 0
        anchors.rightMargin: 0
        anchors.bottomMargin: 0
        fillMode: Image.PreserveAspectFit
    }

    MouseArea {
        id: mouseUserGuide
        width: root.width * 0.03
        height: root.width * 0.03
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.bottomMargin: -10
        anchors.leftMargin: -10
        hoverEnabled: true
        onEntered: {
            pngUserGuide.visible = true
        }
        onExited: {
            pngUserGuide.visible = false
        }


        Text {
            id: textQuestionSymbol
            color: "#ffffff"
            text: qsTr("?")
            anchors.left: parent.left
            anchors.bottom: parent.bottom
            font.pixelSize: 40
            horizontalAlignment: Text.AlignHCenter
            verticalAlignment: Text.AlignVCenter
            font.underline: false
            font.bold: true
            anchors.leftMargin: 20
            anchors.bottomMargin: 20
            font.family: "Arial"
        }
    }









}





/*##^##
Designer {
    D{i:0;formeditorZoom:0.75}D{i:34}D{i:35}D{i:36}D{i:37}
}
##^##*/
