from PyQt6.QtWidgets import (
    QMainWindow, QApplication, QToolBar, QMenu, QToolButton
)
import sys


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        toolbar = QToolBar("Main Toolbar")
        self.addToolBar(toolbar)

        # --- File Menu ---
        file_menu = QMenu("File", self)

        # Import submenu
        import_menu = QMenu("Import", self)
        import_menu.addAction("Image")
        import_menu.addAction("Graph")

        file_menu.addMenu(import_menu)

        # ToolButton to show the File menu
        file_button = QToolButton()
        file_button.setText("File")
        file_button.setMenu(file_menu)
        file_button.setPopupMode(QToolButton.ToolButtonPopupMode.InstantPopup)

        # Add button to toolbar
        toolbar.addWidget(file_button)


app = QApplication(sys.argv)
window = MainWindow()
window.show()
app.exec()
