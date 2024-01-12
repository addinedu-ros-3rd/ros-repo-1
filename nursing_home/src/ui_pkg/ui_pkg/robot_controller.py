import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import QtGui, uic
from PyQt5.QtCore import *
import cv2, imutils
import time
import datetime
import rclpy as rp
from std_msgs.msg import String

from_class = uic.loadUiType("robot_controller.ui")[0]

class WindowClass(QMainWindow, from_class) :

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.tableWidget.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.robot_table.setItem(0,0, QTableWidgetItem('대기 중'))
        self.robot_table.setItem(1,0, QTableWidgetItem('대기 중'))
        self.robot_table.setItem(2,0, QTableWidgetItem('대기 중'))

        # 시간 표시하기     
        timer = QTimer(self)
        timer.timeout.connect(self.time)
        timer.start(1000)

        # 날짜 표시하기
        # date = QDate.currentDate()
        # self.label_3.setText(self.date.toString('yyyy년 MM월 dd일')

        # 토픽 발행하기
        self.count = 0
        rp.init()
        self.node = rp.create_node('task_node')

        self.serve_btn.clicked.connect(self.pub)
        self.serve_stop.clicked.connect(self.stop)
        
        self.publisher = self.node.create_publisher(String, 'task_topic', 10)
        self.timer = self.node.create_timer(1000, self.pub)

        self.serve_btn.hide()
        self.task_combo.hide()
        self.location_combo.hide()
        self.call_btn.hide()
        self.serve_stop.hide()
        self.clear_btn.hide()

        self.serve_mode.clicked.connect(self.serve)
        self.normal_mode.clicked.connect(self.normal)

        # 다크모드 만들기
        self.dark_btn.clicked.connect(self.change_to_black)
        self.white_btn.clicked.connect(self.change_to_white)

        self.zeroto255 = [self.time_label, self.title_label, self.robot_table, self.queue_label,
                          self.serve_mode, self.serve_btn, self.serve_stop, self.normal_mode, self.clear_btn,
                          self.task_combo, self.location_combo, self.call_btn, self.tableWidget]
        
        self.labels = [self.map_group, self.cam_group, self.request_group]

        # 요청하기
        self.call_btn.clicked.connect(self.add)
        self.task_combo.currentIndexChanged.connect(self.update_table)
        self.location_combo.currentIndexChanged.connect(self.update_table)
    
    def init_ros(self):
        rp.init()
        self.node = rp.create_node('task_node')
        self.subscription = self.node.create_subscription(
            String,
            'my_topic',  # 토픽 이름을 여기에 입력
            self.message_callback,
            10  # QoS 프로파일
        )
        self.subscription  # 필요할 경우 추가 구독 설정

    def message_callback(self, msg):
        received_message = msg.data
        print(f'Received: {received_message}')

        # 메세지를 받으면 GUI 업데이트
        self.label.setText(f'Received message: {received_message}')

    def update_table(self,index):
        # ComboBox의 선택이 변경되면 테이블 위젯에 항목 추가
        selected_item1 = self.task_combo.currentText()
        selected_item2 = self.location_combo.currentText()

    def add(self):
        selected_item1 = self.task_combo.currentText()
        selected_item2 = self.location_combo.currentText()

        row_position = self.tableWidget.rowCount()

        self.tableWidget.insertRow(row_position)

        self.tableWidget.setItem(row_position, 0, QTableWidgetItem(selected_item1))
        self.tableWidget.setItem(row_position, 1, QTableWidgetItem(selected_item2))

        # 토픽 발행
        msg = String()
        msg.data = f"{selected_item1} {selected_item2}"
        self.count += 1
        self.node.get_logger().info(f"Publishing: {msg.data}")
        self.publisher.publish(msg)
        
    def pub(self):
        self.serve_stop.show()
        self.serve_btn.setText('진행 중..')
        msg = String()
        msg.data = f"배식 모드 ON! {self.count}"
        self.count += 1
        self.node.get_logger().info(f"Publishing: {msg.data}")
        self.publisher.publish(msg)

    def stop(self):
        self.count = 0
        self.timer.cancel()
        self.node.get_logger().info("Stop Button")   
        
    def change_colors(self, color_rgb):
        for target in self.zeroto255:
            target.setStyleSheet(f'color: {color_rgb};')
    
    def change_labels(self, color_rgb):
        for target in self.labels:
            target.setStyleSheet(f'background-color: {color_rgb};')

    def change_to_black(self):    
        self.change_colors('rgb(249, 240, 107)')
        self.setStyleSheet("background-color: rgb(21,32,43);")
        self.label.setPixmap(QtGui.QPixmap('')) 
        self.change_labels('rgb(34,48,60)')

    def change_to_white(self):    
        self.change_colors('rgb(0,0,0);')
        self.label.setPixmap(QtGui.QPixmap('./molang.png')) 
        self.change_labels('')
        self.setStyleSheet('')

    def serve(self):
        self.serve_btn.show()
    
    def normal(self):
        self.task_combo.show()
        self.location_combo.show()
        self.call_btn.show()
        self.clear_btn.show()

    def time(self):
        current_time = QTime.currentTime()
        label_time = current_time.toString('hh:mm:ss')
        self.time_label.setText(label_time)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    sys.exit(app.exec_())