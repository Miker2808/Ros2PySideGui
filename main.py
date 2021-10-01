# This Python file uses the following encoding: utf-8
import os
from pathlib import Path
import sys

from PySide2.QtCore import QUrl, QSize
from PySide2.QtGui import QGuiApplication, QFontDatabase
from PySide2.QtQuick import QQuickView
from PySide2 import QtMultimedia  # Dirty little secret
from PySide2.QtQuick import QQuickImageProvider


from BackEnd import BackEnd


if __name__ == "__main__":
    app = QGuiApplication(sys.argv)
    view = QQuickView()
    view.setResizeMode(QQuickView.SizeRootObjectToView)
    backend = BackEnd()
    windowsize = QSize(1024, 576)
    view.setMinimumSize(windowsize)
    QFontDatabase.addApplicationFont("fonts/digital-7.ttf")
    view.engine().addImageProvider("imageprovider")
    view.engine().addImportPath("imports")
    view.rootContext().setContextProperty("backend", backend)
    qml_file = Path(__file__).parent / "main.qml"
    view.setSource(QUrl.fromLocalFile(os.fspath(qml_file.resolve())))
    view.Visibility(view.FullScreen)
    if view.status() == QQuickView.Error:
        sys.exit(-1)
    view.show()
    app.exec_()
    del view
