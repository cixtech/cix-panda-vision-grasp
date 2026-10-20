import sys

from PySide6.QtGui import QFont, QPalette, QColor
from PySide6.QtWidgets import QApplication, QStyleFactory

from src.mainwindow import MainWindow


def style(q_app):
    q_app.setStyleSheet(
        "QMainWindow::separator {width: 1px} "
        "QMenu::separator {height: 1px; margin-left: 6px; margin-right: 6px; "
        "background: rgba(155, 155, 155, 255);}"
        "QMessageBox {color: white;}QMessageBox QLabel {color: yellow;}"
    )
    q_app.setStyle(QStyleFactory.create("Fusion"))
    dark = QPalette()
    dark.setColor(QPalette.ColorRole.Text, QColor(255, 255, 255))
    dark.setColor(QPalette.ColorRole.WindowText, QColor(255, 255, 255))
    dark.setColor(QPalette.ColorRole.Window, QColor(50, 50, 50))
    dark.setColor(QPalette.ColorRole.Button, QColor(50, 50, 50))
    dark.setColor(QPalette.ColorRole.Base, QColor(25, 25, 25))
    dark.setColor(QPalette.ColorRole.AlternateBase, QColor(50, 50, 50))
    dark.setColor(QPalette.ColorRole.ToolTipBase, QColor(200, 200, 200))
    dark.setColor(QPalette.ColorRole.ToolTipText, QColor(50, 50, 50))
    dark.setColor(QPalette.ColorRole.ButtonText, QColor(255, 255, 255))
    dark.setColor(QPalette.ColorRole.BrightText, QColor(255, 50, 50))
    dark.setColor(QPalette.ColorRole.Link, QColor(40, 130, 220))
    dark.setColor(
        QPalette.ColorGroup.Disabled, QPalette.ColorRole.Text, QColor(99, 99, 99)
    )
    dark.setColor(
        QPalette.ColorGroup.Disabled,
        QPalette.ColorRole.WindowText,
        QColor(99, 99, 99),
    )
    dark.setColor(
        QPalette.ColorGroup.Disabled,
        QPalette.ColorRole.ButtonText,
        QColor(99, 99, 99),
    )
    dark.setColor(
        QPalette.ColorGroup.Disabled,
        QPalette.ColorRole.Highlight,
        QColor(80, 80, 80),
    )
    dark.setColor(
        QPalette.ColorGroup.Disabled,
        QPalette.ColorRole.HighlightedText,
        QColor(99, 99, 99),
    )
    q_app.setPalette(dark)
    font = QFont()
    font.setPointSize(10)
    q_app.setFont(font)


if __name__ == '__main__':
    if sys.platform == "linux":
        from utils.ros_network_env import ensure_localhost_ros_dds

        ensure_localhost_ros_dds()

        import rclpy

        if not rclpy.ok():
            rclpy.init(args=None)

    app = QApplication.instance()
    if app is None:
        app = QApplication(sys.argv)
    style(app)

    mainwindow = MainWindow()
    mainwindow.show()
    app.exec()

    if sys.platform == "linux":
        import rclpy

        if rclpy.ok():
            rclpy.shutdown()
