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
import yaml


GOAL_REACHED_TOL_DEFAULT = 0.25

ui_file = os.path.join(get_package_share_directory('ui_pkg'), 'ui', 'monitoring.ui')
map_yaml_file = os.path.join(get_package_share_directory('main_pkg'), 'map', 'home.yaml')
from_class = uic.loadUiType(ui_file)[0]

global amcl_1, amcl_2, amcl_3
amcl_1 = PoseWithCovarianceStamped()
amcl_2 = PoseWithCovarianceStamped()
amcl_3 = PoseWithCovarianceStamped()

global path_1, path_2, path_3, path_before_1, path_before_2, path_before_3
path_1 = AstarMsg()
path_2 = AstarMsg()
path_3 = AstarMsg()
path_before_1 = AstarMsg()
path_before_2 = AstarMsg()
path_before_3 = AstarMsg()

global start_point_1, start_point_2, start_point_3
start_point_1, start_point_2, start_point_3 = None, None, None


class AmclSubscriber(Node):

    def __init__(self):

        super().__init__('amcl_subscriber')
  
        amcl_pose_qos = QoSProfile(
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=1)
        
        # 3개의 로봇 위치 표시
        self.pose1 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose_1', 
            self.amcl_callback1, 
            amcl_pose_qos)
        
        self.pose2 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose_2', 
            self.amcl_callback2, 
            amcl_pose_qos)
        
        self.pose3 = self.create_subscription(
            PoseWithCovarianceStamped, 
            '/amcl_pose_3', 
            self.amcl_callback3, 
            amcl_pose_qos)

    def amcl_callback1(self, amcl):
        global amcl_1
        amcl_1 = amcl
        
    def amcl_callback2(self, amcl):
        global amcl_2
        amcl_2 = amcl
        
    def amcl_callback3(self, amcl):
        global amcl_3
        amcl_3 = amcl
        
class PathSubscriber(Node):
    
    def __init__(self):
        super().__init__('path_subscriber')
        
        self.sub1 = self.create_subscription(
            AstarMsg,
            '/astar_paths_1',
            self.path_callback1,
            10
        )
        
        self.sub2 = self.create_subscription(
            AstarMsg,
            '/astar_paths_2',
            self.path_callback2,
            10
        )
        
        self.sub3 = self.create_subscription(
            AstarMsg,
            '/astar_paths_3',
            self.path_callback3,
            10
        )
        
    def path_callback1(self, path):
        global path_1, amcl_1, start_point_1
        path_1 = path
        start_point_1 = amcl_1
        
        
    def path_callback2(self, path):
        global path_2, amcl_2, start_point_2
        path_2 = path
        start_point_2 = amcl_2
        
        
    def path_callback3(self, path):
        global path_3, amcl_3, start_point_3
        path_3 = path
        start_point_3 = amcl_3
        
class DoneTaskSubscriber(Node):
    def __init__(self):
        super().__init__('done_task_subscriber')
        
        self.done_task_1 = self.create_subscription(String, '/done_task_1', self.done_task_callback_1, 10)
        self.done_task_2 = self.create_subscription(String, '/done_task_2', self.done_task_callback_2, 10)
        self.done_task_3 = self.create_subscription(String, '/done_task_3', self.done_task_callback_3, 10)

    def done_task_callback_1(self, msg):
        if msg.data == 'OK':
            global path_1, path_before_1
            path_before_1 = path_1
            path_1 = AstarMsg()

    def done_task_callback_2(self, msg):
        if msg.data == 'OK':
            global path_2, path_before_2
            path_before_2 = path_2
            path_2 = AstarMsg()

    def done_task_callback_3(self, msg):
        if msg.data == 'OK':
            global path_3, path_before_3
            path_before_3 = path_3
            path_3 = AstarMsg()
        

class PiCamSubscriber(Node):

    def __init__(self, ui):

        super().__init__('pi_cam_subscriber')
        self.ui = ui

        self.sub1 = self.create_subscription(
            CompressedImage,
            'image_raw/compressed_1',
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
        self.robot_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.setWindowTitle('Monitoring')

        # 시간 표시하기     
        timer = QTimer(self)
        timer.timeout.connect(self.time)
        timer.timeout.connect(self.updateMap)
        timer.start(200)

        # 토픽 발행하기
        self.count = 0
        self.node = rp.create_node('task_node')
        
        self.publisher = self.node.create_publisher(TaskRequest, 'task_request', 10)

        # 다크모드 만들기
        self.dark_btn.clicked.connect(self.change_to_black)
        self.white_btn.clicked.connect(self.change_to_white)

        self.zeroto255 = [self.time_label, self.robot_table, self.queue_label, self.cctv_label, self.task_label, self.map_label,
                          self.task_combo, self.location_combo, self.call_btn, self.task_queue, self.view_label]
        
        self.labels = [self.map_group, self.cam_group, self.request_group, self.cctv_group]

        # 요청하기
        self.call_btn.clicked.connect(self.add)
        
        # DB에서 콤보박스 가져오기
        self.dm = DataManager()
        self.set_combo()

        with open(map_yaml_file) as f:
            map_yaml_data = yaml.full_load(f)

        # map 관련
        self.pixmap = QPixmap(os.path.join(get_package_share_directory('main_pkg'), 'map', map_yaml_data['image']))
        self.height = self.pixmap.size().height()
        self.width = self.pixmap.size().width()
        self.image_scale = 6
        self.pixmap = self.pixmap.transformed(QTransform().scale(-1, -1))
        self.map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))
    
        self.now_x = 0
        self.now_y = 0

        self.map_resolution = map_yaml_data['resolution']
        self.map_origin = map_yaml_data['origin'][:2]
        
        
    def updateMap(self):
        self.map.setPixmap(self.pixmap.scaled(self.width * self.image_scale, self.height * self.image_scale, Qt.KeepAspectRatio))

        painter = QPainter(self.map.pixmap())
        
        # 로봇 번호 표시
        self.font = QFont()
        self.font.setBold(True)
        self.font.setPointSize(15)
        painter.setFont(self.font)
        
        # 1번 로봇 좌표
        x, y = self.calc_grid_position(amcl_1.pose.pose.position.x, amcl_1.pose.pose.position.y)

        painter.setPen(QPen(Qt.red, 20, Qt.SolidLine))
        painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
        painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), '1')
        
        # 1번 로봇 경로
        
        # 현황: 경로를 그려준 채로 로봇이 그 위에서 움직임
        # todo: 로봇이 지나간 좌표의 선은 지워주기
        
        # 참고: 현재 좌표를 x_before, y_before로 주고 시작하면 로봇이 움직이면서 기존에 그렸던 선이 예쁘게 지워짐
        # 근데 waypoint를 지나가면서 기존 waypoint도 남아있어서 선이 이상하게 그려짐
        if start_point_1 is not None:
            x_start, y_start = self.calc_grid_position(start_point_1.pose.pose.position.x, start_point_1.pose.pose.position.y)
            x_start, y_start = int((self.width - x_start) * self.image_scale), int(y_start * self.image_scale)
        
        x_before_recorded = False
        
        # 완료된 waypoint 지우기
        if path_1.length == 0 and path_before_1.length != 0:    
            for i in range(path_before_1.length):
                x, y = self.calc_grid_position(path_before_1.poses[i].position.x, path_before_1.poses[i].position.y)
                x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
                painter.setPen(QPen(QColor(0, 0, 0, 0)))
                
                if x_before_recorded:
                    painter.drawLine(x_before, y_before, x, y)
                else:
                    painter.drawLine(x_start, y_start, x, y)
                
                painter.drawPoint(x, y)
                x_before, y_before = x, y
                x_before_recorded = True
        
        # 신규 waypoint 그리기
        for i in range(len(path_1.poses)):
            x, y = self.calc_grid_position(path_1.poses[i].position.x, path_1.poses[i].position.y)
            x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
            painter.setPen(QPen(Qt.darkRed, 5))
            
            if x_before_recorded:
                painter.drawLine(x_before, y_before, x, y)
            else:
                painter.drawLine(x_start, y_start, x, y)
            
            painter.setPen(QPen(Qt.darkRed, 10))
            painter.drawPoint(x, y)
            x_before, y_before = x, y
            x_before_recorded = True
        
        # 2번 로봇 좌표
        x, y = self.calc_grid_position(amcl_2.pose.pose.position.x, amcl_2.pose.pose.position.y)

        painter.setPen(QPen(Qt.blue, 20, Qt.SolidLine))
        painter.drawPoint(int((self.width - x) * self.image_scale), int(y * self.image_scale))
        painter.drawText(int((self.width - x) * self.image_scale + 13), int(y * self.image_scale + 5), '2')
        
        # 2번 로봇 경로
        if start_point_2 is not None:
            x_start, y_start = self.calc_grid_position(start_point_2.pose.pose.position.x, start_point_2.pose.pose.position.y)
            x_start, y_start = int((self.width - x_start) * self.image_scale), int(y_start * self.image_scale)
        
        x_before_recorded = False
        
        # 완료된 waypoint 지우기
        if path_2.length == 0 and path_before_2.length != 0:    
            for i in range(path_before_2.length):
                x, y = self.calc_grid_position(path_before_2.poses[i].position.x, path_before_2.poses[i].position.y)
                x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
                painter.setPen(QPen(QColor(0, 0, 0, 0)))
                
                if x_before_recorded:
                    painter.drawLine(x_before, y_before, x, y)
                else:
                    painter.drawLine(x_start, y_start, x, y)
                
                painter.drawPoint(x, y)
                x_before, y_before = x, y
                x_before_recorded = True
        
        # 신규 waypoint 그리기
        for i in range(len(path_2.poses)):
            x, y = self.calc_grid_position(path_2.poses[i].position.x, path_2.poses[i].position.y)
            x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
            painter.setPen(QPen(Qt.darkBlue, 5))
            
            if x_before_recorded:
                painter.drawLine(x_before, y_before, x, y)
            else:
                painter.drawLine(x_start, y_start, x, y)
            
            painter.setPen(QPen(Qt.darkBlue, 10))
            painter.drawPoint(x, y)
            x_before, y_before = x, y
            x_before_recorded = True

        # 3번 로봇 좌표
        x_now, y_now = self.calc_grid_position(amcl_3.pose.pose.position.x, amcl_3.pose.pose.position.y)

        painter.setPen(QPen(Qt.green, 20, Qt.SolidLine))
        painter.drawPoint(int((self.width - x_now) * self.image_scale), int(y_now * self.image_scale))
        painter.drawText(int((self.width - x_now) * self.image_scale + 13), int(y_now * self.image_scale + 5), '3')
        
        # 3번 로봇 경로
        if start_point_3 is not None:
            x_start, y_start = self.calc_grid_position(start_point_3.pose.pose.position.x, start_point_3.pose.pose.position.y)
            x_start, y_start = int((self.width - x_start) * self.image_scale), int(y_start * self.image_scale)
        
        x_before_recorded = False
        
        # 완료된 waypoint 지우기
        if path_3.length == 0 and path_before_3.length != 0:    
            for i in range(path_before_3.length):
                x, y = self.calc_grid_position(path_before_3.poses[i].position.x, path_before_3.poses[i].position.y)
                x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
                painter.setPen(QPen(QColor(0, 0, 0, 0)))
                
                if x_before_recorded:
                    painter.drawLine(x_before, y_before, x, y)
                else:
                    painter.drawLine(x_start, y_start, x, y)
                
                painter.drawPoint(x, y)
                x_before, y_before = x, y
                x_before_recorded = True
        
        # 신규 waypoint 그리기
        for i in range(len(path_3.poses)):
            x, y = self.calc_grid_position(path_3.poses[i].position.x, path_3.poses[i].position.y)
            x, y = int((self.width - x) * self.image_scale), int(y * self.image_scale)
            painter.setPen(QPen(Qt.darkGreen, 5))
            
            if x_before_recorded:
                painter.drawLine(x_before, y_before, x, y)
            else:
                painter.drawLine(x_start, y_start, x, y)
            
            painter.setPen(QPen(Qt.darkGreen, 10))
            painter.drawPoint(x, y)
            x_before, y_before = x, y
            x_before_recorded = True

        painter.end()


    def calc_grid_position(self, x, y):
        pos_x = (x - self.map_origin[0]) / self.map_resolution
        pos_y = (y - self.map_origin[1]) / self.map_resolution
        return pos_x, pos_y
    
    # 현재 좌표가 주어진 경로를 지나갔는지 체크하기 위한 함수
    # 정확하지 않음
    def is_nearby(x, y, x1, y1):
        distance = ((x - x1) ** 2 + (y - y1) ** 2) ** 0.5
        return distance <= GOAL_REACHED_TOL_DEFAULT  # goal_reached_tol default
    
    
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

    amcl_subscriber = AmclSubscriber()
    executor.add_node(amcl_subscriber)
    
    path_subscriber = PathSubscriber()
    executor.add_node(path_subscriber)
    
    done_task_subscriber = DoneTaskSubscriber()
    executor.add_node(done_task_subscriber)

    thread = Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()