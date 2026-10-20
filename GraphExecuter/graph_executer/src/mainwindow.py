# coding=utf-8
import datetime
import json
import os
import subprocess
import sys

from PySide6.QtCore import Qt, Signal, QSize
from PySide6.QtGui import QIcon
from PySide6.QtWidgets import (
    QDockWidget,
    QMainWindow,
    QSizePolicy,
    QWidget,
)

from src.GraphFlow import GraphFlow
from src.messageconsole import MessageConsole
from src.updatelog import UpdateLog
from ui.ui_mainwindow import Ui_MainWindow

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


class MainWindow(QMainWindow):
    messageSignal = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.setDockNestingEnabled(True)
        self.ui.centralwidget.setMaximumWidth(0.1)
        self.settings_dir = os.path.join(BASE_DIR, "settings")

        self.setWindowIcon(QIcon(os.path.join(BASE_DIR, "settings", "myicon.png")))
        self.setIconSize(QSize(22, 22))

        self.updatelog = UpdateLog(self)

        self.messageconsole = MessageConsole()
        self.messageSignal.connect(self.messageconsole.showMessage)
        self.messageSignal.emit("Start time: {}".format(datetime.datetime.now()))

        self.dockConsole = QDockWidget("Message Console", self)
        self.dockConsole.setObjectName("Message Console")
        self.dockConsole.setFloating(False)
        self.dockConsole.setVisible(True)
        self.dockConsole.setWidget(self.messageconsole)
        self.messageconsole.setVisible(True)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.dockConsole)
        self.dockConsole.setTitleBarWidget(QWidget())
        self.dockConsole.setMinimumWidth(300)

        self.initGui()

    def initGui(self):
        self.graph = GraphFlow(self.messageSignal, self)

        self.graph_path = os.path.join(BASE_DIR, "res", "graphs", "process_graph.json")
        try:
            self.graph.graph.load_session(self.graph_path)
            for node in self.graph.graph.all_nodes():
                if hasattr(node, 'sync_open_window_from_property'):
                    node.sync_open_window_from_property()
        except Exception as err:
            self.messageSignal.emit(str(err))
            try:
                os.remove(self.graph_path)
                self.graph.graph._model.session = self.graph_path
            except OSError:
                pass

        self.dock_graph = QDockWidget("Graph", self)
        self.dock_graph.setObjectName("Graph")
        self.dock_graph.setStyleSheet("""
            QDockWidget {
                font-size: 12px;
                font-family: "Microsoft YaHei", "Arial";
                font-weight: normal;
            }
        """)
        size_policy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.dock_graph.setSizePolicy(size_policy)
        self.dock_graph.setVisible(True)
        self.dock_graph.setWidget(self.graph.graph_widget)
        self.graph.graph_widget.setVisible(True)
        self.dock_graph.setAllowedAreas(
            Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea
        )
        self.addDockWidget(
            Qt.DockWidgetArea.RightDockWidgetArea, self.dock_graph, Qt.Orientation.Horizontal
        )
        self.dock_graph.setMinimumWidth(300)

        self.ui.actionexecute_graph.triggered.connect(self.graph.execute_all_nodes)
        self.ui.actionexetute_from_goal_node.triggered.connect(
            self.graph.execute_selected_nodes
        )
        self.ui.actionexetute_from_goal_node.setIcon(
            QIcon(os.path.join(BASE_DIR, "settings", "BtnIcon", "from_obj_node.png"))
        )
        self.ui.actionexecute_graph.setIcon(
            QIcon(os.path.join(BASE_DIR, "settings", "BtnIcon", "all_graph.png"))
        )

        self.ui.actionOpen_log_dir.triggered.connect(self._open_log_dir)
        self.ui.actionUpdate_log.triggered.connect(self.updatelog.show)

        with open(
            os.path.join(BASE_DIR, "settings", "languages", "en.json"),
            "r",
            encoding="utf-8",
        ) as f:
            en_data = json.load(f)
            mw = en_data["main_window"]
            self.ui.menuHelp.setTitle(mw["menuHelp"])
            self.ui.actionUpdate_log.setText(mw["actionUpdate_log"])
            self.ui.actionOpen_log_dir.setText(mw["actionOpen_log_dir"])
            self.ui.menuTools.setTitle(mw["menuTools"])
            self.ui.actionexetute_from_goal_node.setText(
                mw["actionexetute_from_goal_node"]
            )
            self.ui.actionexecute_graph.setText(mw["actionexecute_graph"])

    def _open_log_dir(self):
        log_dir = os.path.dirname(self.messageconsole.logPath)
        if sys.platform == "win32":
            os.startfile(log_dir)
        else:
            subprocess.run(["xdg-open", log_dir], check=False)

    def closeEvent(self, event):
        self.messageSignal.emit("Close time: {}".format(datetime.datetime.now()))
        self.graph.graph.save_session(self.graph_path)
        self.graph.close_event()
        super().closeEvent(event)
