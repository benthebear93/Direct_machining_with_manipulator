# This Python file uses the following encoding: utf-8
import os
import sys

from PySide2 import QtCore
from PySide2.QtWidgets import QApplication, QWidget
from PySide2.QtCore import QFile
from PySide2.QtUiTools import QUiLoader
import basicform

class Widget(QWidget, basicform.Ui_Widget):
    def __init__(self):
        super(Widget, self).__init__()
        self.load_ui()
        #self.btn1.clicked.connect(self.send_pose_click)

    def load_ui(self):
        loader = QUiLoader()
        #path = os.fspath(Path(__file__).resolve().parent / "form.ui")
        path = os.path.abspath("form.ui")
        ui_file = QFile(path)
        ui_file.open(QFile.ReadOnly)
        loader.load(ui_file, self)
        ui_file.close()

    def send_pose_click(self):
        print("send_pose")

if __name__ == "__main__":
    QtCore.QCoreApplication.setAttribute(QtCore.Qt.AA_ShareOpenGLContexts)
    app = QApplication([])
    widget = Widget()
    widget.show()
    sys.exit(app.exec_())
