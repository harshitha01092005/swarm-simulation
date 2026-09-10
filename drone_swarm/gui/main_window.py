"""PySide6 desktop host for the local Gazebo swarm workspace."""

import argparse
import sys

from PySide6.QtCore import QUrl
from PySide6.QtGui import QAction, QColor
from PySide6.QtWidgets import QApplication, QMainWindow, QMessageBox
from PySide6.QtWebEngineWidgets import QWebEngineView


class MainWindow(QMainWindow):
    """Embed the same local web client used by the browser, without simulation logic."""

    def __init__(self, url="http://127.0.0.1:8765"):
        super().__init__()
        self.setWindowTitle("AEROS — Swarm Studio")
        self.resize(1440, 900)
        self.setMinimumSize(1000, 700)
        self.view = QWebEngineView(self)
        self.view.page().setBackgroundColor(QColor("#0b121b"))
        self.setCentralWidget(self.view)
        self.url = QUrl(url)
        self.view.loadFinished.connect(self._loaded)
        reload_action = QAction("Reload workspace", self)
        reload_action.setShortcut("Ctrl+R")
        reload_action.triggered.connect(self.reload)
        self.addAction(reload_action)
        self.view.load(self.url)

    def reload(self):
        self.view.load(self.url)

    def _loaded(self, succeeded):
        if not succeeded:
            box = QMessageBox(self)
            box.setWindowTitle("Workspace unavailable")
            box.setIcon(QMessageBox.Icon.Warning)
            box.setText("The local swarm workspace could not be reached.")
            box.setInformativeText(
                f"Start the Gazebo swarm server at {self.url.toString()}, then reload the workspace."
            )
            box.setStandardButtons(QMessageBox.StandardButton.Retry | QMessageBox.StandardButton.Close)
            if box.exec() == QMessageBox.StandardButton.Retry:
                self.reload()


def main(args=None):
    parser = argparse.ArgumentParser(description="Open AEROS Swarm Studio")
    parser.add_argument("--url", default="http://127.0.0.1:8765", help="Local swarm HTTP server URL")
    options = parser.parse_args(args)
    url = QUrl(options.url)
    if url.scheme() != "http" or url.host() not in ("127.0.0.1", "localhost", "::1"):
        parser.error("--url must be an HTTP address on localhost")
    app = QApplication.instance() or QApplication([sys.argv[0]])
    app.setApplicationName("AEROS Swarm Studio")
    app.setOrganizationName("Drone Swarm Simulator")
    window = MainWindow(options.url)
    window.show()
    return app.exec()


if __name__ == "__main__":
    sys.exit(main())
