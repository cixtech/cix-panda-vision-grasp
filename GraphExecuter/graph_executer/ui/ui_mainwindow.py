# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'mainwindow.ui'
##
## Created by: Qt User Interface Compiler version 6.8.3
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import QCoreApplication, QMetaObject, Qt, QRect
from PySide6.QtGui import QAction, QFont
from PySide6.QtWidgets import QApplication, QMainWindow, QMenu, QMenuBar, QToolBar, QWidget


class Ui_MainWindow(object):
    def setupUi(self, MainWindow):
        if not MainWindow.objectName():
            MainWindow.setObjectName(u"MainWindow")
        MainWindow.resize(800, 622)
        self.actionOpen_log_dir = QAction(MainWindow)
        self.actionOpen_log_dir.setObjectName(u"actionOpen_log_dir")
        font = QFont()
        self.actionOpen_log_dir.setFont(font)
        self.actionUpdate_log = QAction(MainWindow)
        self.actionUpdate_log.setObjectName(u"actionUpdate_log")
        self.actionUpdate_log.setFont(font)
        self.actionexecute_graph = QAction(MainWindow)
        self.actionexecute_graph.setObjectName(u"actionexecute_graph")
        self.actionexetute_from_goal_node = QAction(MainWindow)
        self.actionexetute_from_goal_node.setObjectName(u"actionexetute_from_goal_node")
        self.centralwidget = QWidget(MainWindow)
        self.centralwidget.setObjectName(u"centralwidget")
        MainWindow.setCentralWidget(self.centralwidget)
        self.menuBar = QMenuBar(MainWindow)
        self.menuBar.setObjectName(u"menuBar")
        self.menuBar.setGeometry(QRect(0, 0, 800, 33))
        font1 = QFont()
        font1.setBold(False)
        self.menuBar.setFont(font1)
        self.menuHelp = QMenu(self.menuBar)
        self.menuHelp.setObjectName(u"menuHelp")
        self.menuHelp.setFont(font1)
        self.menuTools = QMenu(self.menuBar)
        self.menuTools.setObjectName(u"menuTools")
        MainWindow.setMenuBar(self.menuBar)
        self.toolBar = QToolBar(MainWindow)
        self.toolBar.setObjectName(u"toolBar")
        MainWindow.addToolBar(Qt.ToolBarArea.TopToolBarArea, self.toolBar)

        self.menuBar.addAction(self.menuTools.menuAction())
        self.menuBar.addAction(self.menuHelp.menuAction())
        self.menuHelp.addAction(self.actionOpen_log_dir)
        self.menuHelp.addAction(self.actionUpdate_log)
        self.menuTools.addAction(self.actionexetute_from_goal_node)
        self.menuTools.addAction(self.actionexecute_graph)
        self.toolBar.addAction(self.actionexetute_from_goal_node)

        self.retranslateUi(MainWindow)

        QMetaObject.connectSlotsByName(MainWindow)

    def retranslateUi(self, MainWindow):
        MainWindow.setWindowTitle(QCoreApplication.translate("MainWindow", u"GraphExecuter", None))
        self.actionOpen_log_dir.setText(
            QCoreApplication.translate("MainWindow", u"\u6253\u5f00\u65e5\u5fd7\u6587\u4ef6\u5939", None)
        )
        self.actionUpdate_log.setText(
            QCoreApplication.translate("MainWindow", u"\u66f4\u65b0\u8bb0\u5f55", None)
        )
        self.actionexecute_graph.setText(
            QCoreApplication.translate("MainWindow", u"\u8fd0\u884c\u6574\u56fe", None)
        )
        self.actionexecute_graph.setIconText(
            QCoreApplication.translate(
                "MainWindow",
                u"\u4ece\u6240\u6709\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u521b\u5efa\u591a\u7ebf\u7a0b\u8fd0\u884c\u6574\u4e2a\u56fe",
                None,
            )
        )
        self.actionexecute_graph.setToolTip(
            QCoreApplication.translate(
                "MainWindow",
                u"\u4ece\u6240\u6709\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u521b\u5efa\u591a\u7ebf\u7a0b\u8fd0\u884c\u6574\u4e2a\u56fe",
                None,
            )
        )
        self.actionexetute_from_goal_node.setText(
            QCoreApplication.translate("MainWindow", u"\u9009\u4e2d\u76ee\u6807\u8282\u70b9\u8fd0\u884c", None)
        )
        self.actionexetute_from_goal_node.setIconText(
            QCoreApplication.translate(
                "MainWindow",
                u"\u4ece\u9009\u4e2d\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u8fd0\u884c",
                None,
            )
        )
        self.actionexetute_from_goal_node.setToolTip(
            QCoreApplication.translate(
                "MainWindow",
                u"\u4ece\u9009\u4e2d\u7684\u76ee\u6807\u8282\u70b9\u5f00\u59cb\u8fd0\u884c",
                None,
            )
        )
        self.menuHelp.setTitle(QCoreApplication.translate("MainWindow", u"\u5e2e\u52a9", None))
        self.menuTools.setTitle(QCoreApplication.translate("MainWindow", u"\u5de5\u5177", None))
        self.toolBar.setWindowTitle(QCoreApplication.translate("MainWindow", u"toolBar", None))
