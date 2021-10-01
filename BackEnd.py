# This Python file uses the following encoding: utf-8
from PySide2 import QtCore
from PySide2 import QtQuick

from PySide2.QtCore import Slot, Property, Signal, QTimer
from PySide2.QtQuick import QQuickImageProvider
from PySide2.QtQml import QQmlImageProviderBase

from QNode import QNode

class ImageProvider(QQuickImageProvider):
    def __init__(self):
        QQuickImageProvider.__init__(self, QQuickImageProvider.Pixmap)

    def request

class BackEnd(QtCore.QObject):

    imu_speedChanged = Signal()  # related to prop_imu_speed

    @Slot()
    def update_qml_gui(self):
        self.set_imu_speed(self.qnode.ros_node.imu_speed_value)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.speed_value = 20
        self.qnode = QNode()
        self.qnode.start()
        self.update_qml_timer = QTimer(self)
        self.update_qml_timer.timeout.connect(self.update_qml_gui)
        self.update_qml_timer.start(200)

        self._imu_speed = 0

    def __del__(self):
        pass  # for now

    def get_imu_speed(self):  # related to prop_imu_speed
        return self._imu_speed

    def set_imu_speed(self, value):  # related to pro_imu_speed
        if self._imu_speed == value:
            return
        self._imu_speed = value
        self.imu_speedChanged.emit()

    prop_imu_speed = Property(int, fget=get_imu_speed, fset=set_imu_speed, notify=imu_speedChanged)  # nopep8

    @Slot()
    def w_key_pressed(self):
        # print("Pressed w")
        self.qnode.ros_node.esc_pub_value = 1500 + self.speed_value

    @Slot()
    def a_key_pressed(self):
        # print("Pressed a")
        self.qnode.ros_node.servo_pub_value = 1000

    @Slot()
    def s_key_pressed(self):
        # print("Pressed s")
        self.qnode.ros_node.esc_pub_value = 1500 - self.speed_value

    @Slot()
    def d_key_pressed(self):
        # print("Pressed d")
        self.qnode.ros_node.servo_pub_value = 1800

    @Slot()
    def shift_key_pressed(self):
        # print("Pressed Shift")
        if(self.speed_value < 100):
            self.speed_value += 20

    @Slot()
    def ctrl_key_pressed(self):
        # print("Pressed Ctrl")
        if(self.speed_value > 0):
            self.speed_value -= 20

    @Slot()
    def w_key_released(self):
        # print("released w")
        self.qnode.ros_node.esc_pub_value = 1500

    @Slot()
    def a_key_released(self):
        # print("released a")
        self.qnode.ros_node.servo_pub_value = 1400

    @Slot()
    def s_key_released(self):
        # print("released s")
        self.qnode.ros_node.esc_pub_value = 1500

    @Slot()
    def d_key_released(self):
        # print("released d")
        self.qnode.ros_node.servo_pub_value = 1400


