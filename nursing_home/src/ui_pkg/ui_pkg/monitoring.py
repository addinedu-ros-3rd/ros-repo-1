from threading import Thread

from PyQt5.QtWidgets import *
from PyQt5.QtGui import *
from PyQt5 import QtGui, uic
from PyQt5.QtCore import *

import rclpy as rp
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile, qos_profile_sensor_data

from interfaces_pkg.msg import *
from std_msgs.msg import String
from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import PoseWithCovarianceStamped 

from cv_bridge import CvBridge

from database.service_ui import DataManager

from ament_index_python.packages import get_package_share_directory

import numpy as np
import cv2
import sys
import os

ui_file = os.path.join(get_package_share_directory('ui_pkg'), 'ui', 'monitoring.ui')
from_class = uic.loadUiType(ui_file)[0]

class AmclSubscriber(Node):

    def __init__(self, ui):

        super().__init__('amcl_subscriber')
        self.ui = ui        

        self.pixmap = QPixmap('./src/main_pkg/map/home.pgm')
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        self.ui.map_label.setPixmap(self.pixmap.scaled(372, 498, Qt.KeepAspectRatio))
  
        amcl_pose_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)
        
        # 3개의 로봇이 전부 표시되어야 함
        self.pose1 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose_1', 
            self.amcl_callback1, 
            amcl_pose_qos)
        self.create_timer(self)
        
        self.pose2 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose_2', 
            self.amcl_callback1, 
            amcl_pose_qos)
        
        self.pose3 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose_3', 
            self.amcl_callback1, 
            amcl_pose_qos)
    
        self.now_x = 0
        self.now_y = 0

        self.map_resolution = 0.05
        self.map_origin = (-0.315, -2.76)

        self.painter = QPainter(self.ui.map_label.pixmap())
        self.painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))


    def amcl_callback1(self, amcl):
        self.now_x = amcl.pose.pose.position.x
        self.now_y = amcl.pose.pose.position.y
        self.get_logger().info(f"{self.now_x}, {self.now_y}")

        x, y = self.calc_grid_position(self.now_x, self.now_y)

        self.ui.map_label.setPixmap(self.pixmap.scaled(372,498, Qt.KeepAspectRatio))
        painter = QPainter(self.ui.map_label.pixmap())
        painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
        painter.drawPoint(int((self.width - x)* 6), int(y * 6))
        painter.end

    def calc_grid_position(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
        return pos_x, pos_y


class PiCamSubscriber(Node):

    def __init__(self, ui):

        super().__init__('pi_cam_subscriber')
        self.ui = ui

        self.sub1 = self.create_subscription(
            CompressedImage,
            'image_raw/compressed',
            lambda data: self.listener_callback(data, self.ui.cam_r3_1),
            10)

        self.sub2 = self.create_subscription(
            CompressedImage,
            'image_raw/compressed_2',
            lambda data: self.listener_callback(data, self.ui.cam_r3_2),
            10)

        self.sub3 = self.create_subscription(
            CompressedImage,
            'image_raw/compressed_3',
            lambda data: self.listener_callback(data, self.ui.cam_r3_3),
            10)

    def listener_callback(self, data, label_widget):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        height, width, channel = image_np.shape
        bytes_per_line = 3 * width
        q_image = QImage(image_np.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        label_widget.setPixmap(pixmap)

class CctvVideoSubscriber(Node):
    
    def __init__(self, ui):

        super().__init__('cctv_video_subscriber')
        self.ui = ui

        self.subscription = self.create_subscription(
            CompressedImage,
            'cctv_video',
            self.video_callback,
            qos_profile_sensor_data)
        
        self.bridge = CvBridge()
    
    def video_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        height, width, channel = image_np.shape
        self.get_logger().info(image_np.shape)
        bytes_per_line = 3 * width
        q_image = QImage(image_np.data, width, height, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        self.ui.cam_label.setPixmap(pixmap)


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


class EmergencySubscriber(Node):

    def __init__(self, ui):
        super().__init__('emergency_status_subscriber')
        self.ui = ui

        self.subscription = self.create_subscription(
            String,
            'action_rec',
            self.callback,
            1
        )

    def callback(self, msg):
        if msg.data == 'Collapsed':
            self.ui.cctv_label.setText("Emergency 🔴")
        else:
            self.ui.cctv_label.setText("CCTV 🟢")
            


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
        if len(msg.data) != self.ui.task_queue.rowCount():
            self.ui.task_queue.setRowCount(0)
        
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

        self.serve_mode.clicked.connect(self.serve)
        self.normal_mode.clicked.connect(self.normal)

        # 다크모드 만들기
        self.dark_btn.clicked.connect(self.change_to_black)
        self.white_btn.clicked.connect(self.change_to_white)

        self.zeroto255 = [self.time_label, self.title_label, self.robot_table, self.queue_label, self.cctv_label,
                          self.serve_mode, self.serve_btn, self.serve_stop, self.normal_mode, self.task_label,
                          self.task_combo, self.location_combo, self.call_btn, self.task_queue, self.view_label]
        
        self.labels = [self.map_group, self.cam_group, self.request_group, self.cctv_group]

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
        self.change_colors('rgb(255,255,255)') 
        self.setStyleSheet("background-color: rgb(21,32,43);")
        self.change_labels('rgb(34,48,60)')


    def change_to_white(self):    
        self.change_colors('rgb(0,0,0);')
        self.setStyleSheet('')
        self.change_labels('')


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
    rp.init()
    executor = MultiThreadedExecutor()
    
    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    pi_cam_subscriber = PiCamSubscriber(myWindows)
    executor.add_node(pi_cam_subscriber)

    emergency_status_subscriber = EmergencySubscriber(myWindows)
    executor.add_node(emergency_status_subscriber)

    robot_status_subscriber = RobotStatusSubscriber(myWindows)
    executor.add_node(robot_status_subscriber)
    
    task_queue_subscriber = TaskQueueSubscriber(myWindows)
    executor.add_node(task_queue_subscriber)

    cctv_video_subscriber = CctvVideoSubscriber(myWindows)
    executor.add_node(cctv_video_subscriber)

    amcl_subscriber = AmclSubscriber(myWindows)
    executor.add_node(amcl_subscriber)

    thread = Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()