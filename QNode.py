# This Python file uses the following encoding: utf-8
from PySide2.QtCore import QThread
# from PySide2 import QtCore, QObject

import rclpy
from RosNode import RosNode


class QNode(QThread):
    def __init__(self, parent=None):
        super(QNode, self).__init__(parent)

    def run(self):
        print("Thread Running")
        rclpy.init(args=None)
        self.ros_node = RosNode()
        rclpy.spin(self.ros_node)
        # self.ros_node.destroy_node()
        rclpy.shutdown()
