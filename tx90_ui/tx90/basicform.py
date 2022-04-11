# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'form.ui'
#
# Created by: PyQt4 UI code generator 4.12.1
#
# WARNING! All changes made in this file will be lost!

from PyQt4 import QtCore, QtGui

try:
    _fromUtf8 = QtCore.QString.fromUtf8
except AttributeError:
    def _fromUtf8(s):
        return s

try:
    _encoding = QtGui.QApplication.UnicodeUTF8
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig, _encoding)
except AttributeError:
    def _translate(context, text, disambig):
        return QtGui.QApplication.translate(context, text, disambig)

class Ui_Widget(object):
    def setupUi(self, Widget):
        Widget.setObjectName(_fromUtf8("Widget"))
        Widget.resize(800, 600)
        self.btn1 = QtGui.QPushButton(Widget)
        self.btn1.setGeometry(QtCore.QRect(330, 80, 89, 25))
        self.btn1.setObjectName(_fromUtf8("btn1"))
        self.label = QtGui.QLabel(Widget)
        self.label.setGeometry(QtCore.QRect(10, 10, 231, 41))
        font = QtGui.QFont()
        font.setPointSize(15)
        self.label.setFont(font)
        self.label.setObjectName(_fromUtf8("label"))

        self.retranslateUi(Widget)
        QtCore.QObject.connect(self.btn1, QtCore.SIGNAL(_fromUtf8("clicked()")), Widget.send_pose_click)
        QtCore.QMetaObject.connectSlotsByName(Widget)

    def retranslateUi(self, Widget):
        Widget.setWindowTitle(_translate("Widget", "Widget", None))
        self.btn1.setText(_translate("Widget", "Send", None))
        self.label.setText(_translate("Widget", "Normal vector calculation", None))

