import sys

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import uic
from PyQt5.QtCore import *

from utils.custom_logger import Logger

log = Logger(__name__)

from_class = uic.loadUiType("/home/yoh/git_ws/ros-repo-1/nursing_home/src/ui_pkg/ui_pkg/monitoring.ui")[0]


class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

        # creating a timer object
        timer = QTimer(self)

        # adding action to timer
        timer.timeout.connect(self.time)
        timer.start(1000)

        # 날짜 표시하기
        # date = QDate.currentDate()
        # self.label_3.setText(self.date.toString('yyyy년 MM월 dd일')

        self.serve_btn.hide()
        self.task_combo.hide()
        self.location_combo.hide()
        self.call_btn.hide()

        self.serve_mode.clicked.connect(self.serve)
        self.normal_mode.clicked.connect(self.normal)

    def serve(self):
        self.serve_btn.show()
    
    def normal(self):
        self.task_combo.show()
        self.location_combo.show()
        self.call_btn.show()


    def time(self):
        current_time = QTime.currentTime()
        label_time = current_time.toString('hh:mm:ss')
        self.time_label.setText(label_time)


def main():
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())
        

if __name__ == "__main__":
    main()