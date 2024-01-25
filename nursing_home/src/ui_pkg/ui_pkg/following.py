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

from ament_index_python.packages import get_package_share_directory

import numpy as np
import signal
import cv2
import sys
import os
import yaml
import time

ui_file = os.path.join(get_package_share_directory('ui_pkg'), 'ui', 'following.ui')
from_class = uic.loadUiType(ui_file)[0]


class PiCamSubscriber(Node):
    def __init__(self, ui):
        super().__init__('pi_cam_subscriber')
        self.ui = ui

        self.sub1 = self.create_subscription(
            CompressedImage,
            '/yolo_video',
            self.listener_callback,
            qos_profile_sensor_data)
        
        self.bridge = CvBridge()
    
    def listener_callback(self, data):
        np_arr = np.frombuffer(data.data, np.uint8)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        image_np = cv2.cvtColor(image_np, cv2.COLOR_BGR2RGB)
        height, width, channel = image_np.shape
        bytes_per_line = 3 * width
        q_image = QImage(image_np.data, width, height, bytes_per_line, QImage.Format_RGB888)
        self.pixmap = QPixmap.fromImage(q_image)
        self.pixmap = self.pixmap.scaled(self.ui.cam_label.width(), self.ui.cam_label.width())
        self.ui.cam_label.setPixmap(self.pixmap)


class HandSubscriber(Node):
    def __init__(self, ui):
        super().__init__('hand_status_subscriber')
        self.ui = ui

        self.subscription = self.create_subscription(
            String,
            '/hand_gesture',
            self.callback,
            10
        )
        self.follow_publisher = self.create_publisher(String, '/follow', 10)
        self.prev_hand = 'wait'
        self.cur_data = None
        self.start_time = None
        self.last_times = 0

    def callback(self, msg):
        hand = String()
        
        if self.cur_data != msg.data:
            self.cur_data = msg.data
            self.start_time = time.time()
        
        else:
            if time.time() - self.start_time >= 2.5:
                
                self.hand = msg.data
                
                if msg.data == 'start':
                    self.ui.follow_label.setText("Follow 🟢")
                    hand.data = 'follow'
                    self.prev_hand = hand.data
                elif msg.data == 'stop':
                    self.ui.follow_label.setText("Wait 🔴")
                    hand.data = 'wait'
                    self.prev_hand = hand.data
                else:
                    hand.data = self.prev_hand
                
                self.follow_publisher.publish(hand)

        # if msg.data == 'start':
        #     self.ui.follow_label.setText("Follow 🟢")
        #     hand.data = 'follow'
        #     self.prev_hand = hand.data

        # elif msg.data == 'stop':
        #     self.ui.follow_label.setText("Wait 🔴")
        #     hand.data = 'wait'
        #     self.prev_hand = hand.data
        
        # else:
        #     hand.data = self.prev_hand

        # self.follow_publisher.publish(hand)


class WindowClass(QMainWindow, from_class):
    def __init__(self):
        super().__init__()
        self.setupUi(self)
        self.setWindowTitle('Following')

        self.follow_label.setText('Wait 🔴')

        self.follow_node = rp.create_node('following_mode')
        self.follow_publisher = self.follow_node.create_publisher(String, '/follow', 10)


        # 사람 선택
        self.isCaptureOn = False
        self.capture_btn.setText('Capture Person')
        self.capture_label.hide()
        self.capture_btn.clicked.connect(self.clickCapture)

        self.capture_node = rp.create_node('capture_mode')
        self.capture_publisher = self.capture_node.create_publisher(String, '/capturing', 10)
        

    def clickCapture(self):
        msg = String()
        if self.isCaptureOn == False:
            self.capture_btn.setText('Stop')
            self.isCaptureOn = True
            self.capture_label.show()
            self.capture_label.setText('Capturing 🟢')

            msg.data = 'capture_start'
            
        else:
            self.capture_btn.setText('Capture Person')
            self.isCaptureOn = False
            self.capture_label.show()
            self.capture_label.setText('Stop 🔴')

            msg.data = 'capture_stop'

        self.capture_publisher.publish(msg)

def main():
    rp.init()
    executor = MultiThreadedExecutor()

    app = QApplication(sys.argv)
    myWindows = WindowClass()
    myWindows.show()

    pi_cam_subscriber = PiCamSubscriber(myWindows)
    executor.add_node(pi_cam_subscriber)

    hand_status_subscriber = HandSubscriber(myWindows)
    executor.add_node(hand_status_subscriber)

    thread = Thread(target=executor.spin)
    thread.start()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()