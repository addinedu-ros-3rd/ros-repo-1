import numpy as np
import cv2
import torch
from torch.utils.data import Dataset, DataLoader


class MyDataset(Dataset):
    def __init__(self, seq_list):
        self.X = []
        self.y = []
        for dic in seq_list:
            self.y.append(dic['key'])
            self.X.append(dic['value'])
        
    def __getitem__(self, index):
        data = self.X[index]
        label = self.y[index]
        return torch.Tensor(np.array(data)), torch.tensor(np.array(int(label)))
    
    def __len__(self):
        return len(self.X)
    

class ActionCam():
    def __init__(self, attention_dot, draw_line, length, pose, lstm_model, device):
        self.attention_dot = attention_dot
        self.draw_line = draw_line
        self.length = length
        self.device = device

        self.pose = pose
        self.lstm_model = lstm_model
        self.input_list = []
        self.status = 'None'
        self.color = (0, 0, 0)
        

    def draw_pose(self, draw_line, draw_line_dic, img, color):
        for line in draw_line:
            x1, y1 = draw_line_dic[line[0]][0], draw_line_dic[line[0]][1]
            x2, y2 = draw_line_dic[line[1]][0], draw_line_dic[line[1]][1]
            img = cv2.line(img, (x1, y1), (x2, y2), color, 2)
        
        return img

    
    def predict(self, img):
        width, height = img.shape[1], img.shape[0]
        
        results = self.pose.process(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))

        if not results.pose_landmarks: 
            return img, self.status
        else:
            xy_list = []
            idx = 0
            draw_line_dic = {}
            
            for x_and_y in results.pose_landmarks.landmark:
                if idx in self.attention_dot:
                    xy_list.append(x_and_y.x)
                    xy_list.append(x_and_y.y)
                    x, y = int(x_and_y.x * width), int(x_and_y.y * height)
                    draw_line_dic[idx] = [x, y]
                idx += 1

            self.input_list.append(xy_list)

            if len(self.input_list) == self.length:
                dataset = []
                dataset.append({'key' : 0, 'value' : self.input_list})
                dataset = MyDataset(dataset)
                dataset = DataLoader(dataset)
                self.input_list = []
                for data, _ in dataset:
                    data = data.to(self.device)
                    with torch.no_grad():
                        result = self.lstm_model(data)
                        _, act_out = torch.max(result, 1)
                        
                        if act_out.item() == 0:
                            # self.output_data = 0
                            action_text = 'Collapsed'
                            self.status = action_text
                            self.color = (0, 0, 255)

                        elif act_out.item() == 1:
                            # self.output_data = 1
                            action_text = 'Fell Down'
                            self.status = action_text
                            self.color = (255, 0, 0)

                        elif act_out.item() == 2:
                            # self.output_data = 2
                            action_text = 'Standing Up'
                            self.status = action_text
                            self.color = (0, 255, 0)

                        else:
                            # self.output_data = 3
                            action_text = 'Walking'
                            self.status = action_text
                            self.color = (0, 255, 0)

            self.draw_pose(self.draw_line, draw_line_dic, img, self.color)
            cv2.putText(img, self.status, (0, 25), cv2.FONT_HERSHEY_SIMPLEX, 1, self.color, 2, cv2.LINE_AA)

            return img, self.status
