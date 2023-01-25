from PyQt5 import QtWidgets
from PyQt5.QtWidgets import QMainWindow
from Settings import Ui_Form
from PyQt5.QtWidgets import *
import sys

class Settings(Ui_Form, QWidget):
    choseObject=""
    def __init__(self):
        super().__init__()
        QWidget.__init__(self)
        self.left = 400
        self.top = 50
        self.move(self.left, self.top)  # set location for window
        self.setWindowTitle("Control Settings")  # change title
        self.ui = Ui_Form()
        self.ui.setupUi(self)




"""app = QtWidgets.QApplication(sys.argv)
MainWindow = QWidget()
ui = Settings()
ui.setupUi(MainWindow)
MainWindow.show()
app.exec_()"""
"""if __name__ == "__main__":
    app = QtWidgets.QApplication(sys.argv)
    MainWindow = QMainWindow()
    ui = Settings()
    ui.setupUi(MainWindow)
    MainWindow.show()
    sys.exit(app.exec_())"""