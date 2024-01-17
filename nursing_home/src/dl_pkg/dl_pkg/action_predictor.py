import os
import cv2
import torch
import torch.nn as nn
from torch.utils.data import Dataset, DataLoader
import mediapipe as mp
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from models.action import ActionCam
from models.action_lstm import ActionLSTM
from ament_index_python.packages import get_package_share_directory

TIMER_PERIOD = 0.1

class ActionPublisher(Node):
    def __init__(self):
            super().__init__(node_name='action_publisher')
            self.publisher = self.create_publisher(String, '/action_rec' , 10)
            self.timer = self.create_timer(TIMER_PERIOD, self.action_timer_callback)
            
            self.action_output_data = None
            self.action_output_frame = None

            cv2.destroyAllWindows()
            self.cap = cv2.VideoCapture(0)

            
            self.attention_dot = [11, 12, 13, 14, 15, 16, 23, 24, 25, 26, 27, 28]
            self.draw_line = [[11, 13], [13, 15], [12, 14], [14, 16], [23, 25], [25, 27], 
                              [24, 26], [26, 28], [11, 12], [11, 23], [23, 24], [12, 24]]
            
            self.length = 20    
            self.input_size = 24
            self.hidden_size = 128
            self.num_layers = 2
            self.action_num_class = 4

            if torch.cuda.is_available() == True:
                self.device = 'cuda:0'
                print("GPU is available")
            else:
                self.device = 'cpu'
                print('GPU is unavailable')
 
            self.mp_pose = mp.solutions.pose
            self.pose = self.mp_pose.Pose(static_image_mode=True, model_complexity=1, enable_segmentation=False, min_detection_confidence = 0.5)

            self.lstm = ActionLSTM(self.input_size, self.hidden_size, self.num_layers, self.action_num_class, self.device).to(self.device)
            self.lstm.load_state_dict(torch.load(os.path.join(get_package_share_directory('dl_pkg'), 'models', 'action_state_dict.pt')))
            self.lstm.eval()

            self.Action = ActionCam(self.attention_dot, self.draw_line, self.length, self.pose, self.lstm, self.device)

    def action_timer_callback(self):
        msg = String()

        if self.cap.isOpened():
            ret, img = self.cap.read()

            if ret:
                img = img[:, :400, :]

                action_output_frame, self.action_output_data = self.Action.predict(img)

                msg.data = self.action_output_data
                self.publisher.publish(msg)

                print(self.action_output_data)
                
                cv2.imshow("Action Cam", action_output_frame)
                cv2.waitKey(1)





def main(args=None):
    
    rclpy.init(args=args)
    
    action_pulisher = ActionPublisher()
    
    try :
        rclpy.spin(action_pulisher)
    
    except KeyboardInterrupt :
        action_pulisher.get_logger().info('Publish Stopped')
    
    finally :
        action_pulisher.destroy_node()
        rclpy.shutdown()




if __name__ == '__main__' :
    main()