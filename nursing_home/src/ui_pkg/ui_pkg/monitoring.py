import sys
from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import QtGui, uic
from PyQt5.QtCore import *
import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from threading import Thread

from interfaces_pkg.msg import *
from std_msgs.msg import String
from database.service_ui import DataManager


from_class = uic.loadUiType("/home/yoh/git_ws/ros-repo-1/nursing_home/src/ui_pkg/ui_pkg/monitoring.ui")[0]


class RobotStatusSubscriber(Node):
    def __init__(self, ui):
        
        super().__init__('robot_status_subscriber')
        
        self.ui = ui
        self.subscription = self.create_subscription(
            RobotStatusList,
            'robot_status',
            self.callback,
            1
        )

    def callback(self, msg):
        
        self.ui.robot_table.setItem(0, 0, QTableWidgetItem(msg.robot1.status))
        self.ui.robot_table.setItem(0, 1, QTableWidgetItem(msg.robot1.task))
        self.ui.robot_table.setItem(0, 2, QTableWidgetItem(msg.robot1.goal))
        
        self.ui.robot_table.setItem(1, 0, QTableWidgetItem(msg.robot2.status))
        self.ui.robot_table.setItem(1, 1, QTableWidgetItem(msg.robot2.task))
        self.ui.robot_table.setItem(1, 2, QTableWidgetItem(msg.robot2.goal))
        
        self.ui.robot_table.setItem(2, 0, QTableWidgetItem(msg.robot3.status))
        self.ui.robot_table.setItem(2, 1, QTableWidgetItem(msg.robot3.task))
        self.ui.robot_table.setItem(2, 2, QTableWidgetItem(msg.robot3.goal))
        
        
class TaskQueueSubscriber(Node):
    def __init__(self, ui):
        super().__init__('task_queue_subscriber')
        
        self.ui = ui
        self.subscription = self.create_subscription(
            TaskQueue,
            'task_queue',
            self.callback,
            1
        )
        
        
    def callback(self, msg):
        # print(len(msg.data))
        
        if len(msg.data) > 0:
            for i in range(len(msg.data)):
                # print(msg.data[i].task_type, msg.data[i].place)
                
                if self.ui.task_queue.rowCount() < len(msg.data):
                    self.ui.task_queue.insertRow(i)
                    
                    self.ui.task_queue.setItem(i, 0, QTableWidgetItem(msg.data[i].task_type))
                    self.ui.task_queue.setItem(i, 1, QTableWidgetItem(msg.data[i].place))


class WindowClass(QMainWindow, from_class):

    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.task_queue.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        # 시간 표시하기     
        timer = QTimer(self)
        timer.timeout.connect(self.time)
        timer.start(1000)

        # 날짜 표시하기
        # date = QDate.currentDate()
        # self.label_3.setText(self.date.toString('yyyy년 MM월 dd일')

        # 토픽 발행하기
        self.count = 0
        self.node = rp.create_node('task_node')

        self.serve_btn.clicked.connect(self.pub)
        self.serve_stop.clicked.connect(self.stop)
        
        self.publisher = self.node.create_publisher(TaskRequest, 'task_request', 10)
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
                          self.task_combo, self.location_combo, self.call_btn, self.task_queue]
        
        self.labels = [self.map_group, self.cam_group, self.request_group]

        # 요청하기
        self.call_btn.clicked.connect(self.add)
        
        # DB에서 콤보박스 가져오기
        self.dm = DataManager()
        self.set_combo()
        
        
    def set_combo(self):
        task_type_list = self.dm.select_all_task_type()
        for item in task_type_list:
            self.task_combo.addItem(item[0], item[1])  # 이름과 ID를 가져옴(이름만 표시)
            
        place_list = self.dm.select_all_place()
        for item in place_list:
            self.location_combo.addItem(item[0], (item[1], item[2], item[3]))  # 이름과 (x, y, z) 좌표를 가져옴(이름만 표시) 
            

    def add(self):
        task_type_id = self.task_combo.currentData()
        place = self.location_combo.currentText()
        pos = self.location_combo.currentData()

        # 토픽 발행
        req = TaskRequest()
        req.task_type_id = task_type_id
        req.place = place
        req.position.x = pos[0]
        req.position.y = pos[1]
        req.position.z = pos[2]
        
        self.count += 1
        self.publisher.publish(req)
        
        
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
        
        
def main():
    rp.init()
    executor = MultiThreadedExecutor()
    
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()
    
    robot_status_subscriber = RobotStatusSubscriber(myWindows)
    executor.add_node(robot_status_subscriber)
    
    task_queue_subscriber = TaskQueueSubscriber(myWindows)
    executor.add_node(task_queue_subscriber)
    
    thread = Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()