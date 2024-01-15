from utils.custom_logger import Logger
from tools.task_planning import TaskPlanning

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from interfaces_pkg.msg import *


log = Logger(__name__)
task_planner = TaskPlanning()

TIMER_PERIOD = 0.5

class TaskSubscriber(Node):

    def __init__(self):
        log.info("TaskSubscriber started.")
        self.received_task = None
        
        super().__init__('task_subscriber')
        self.subscription = self.create_subscription(
            TaskRequest,
            'task_request',
            self.callback,
            10)


    def callback(self, msg):
        # log.info(msg)
        task_planner.add_task(msg)
        
        
class RobotStatusPublisher(Node):
    
    def __init__(self):
        log.info("SendRobotStatusPublisher started.")
        
        super().__init__('send_robot_status_publisher')
        self.publisher = self.create_publisher(RobotStatusList, '/robot_status', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        
        
    def timer_callback(self):
        msg = RobotStatusList()
        
        task_planner.robot_status_list = task_planner.show_robot_status()
        
        msg.robot1.id = task_planner.robot_status_list[0][0]
        msg.robot1.status = task_planner.robot_status_list[0][1]
        msg.robot1.task = task_planner.robot_status_list[0][2]
        msg.robot1.goal = task_planner.robot_status_list[0][3]
        
        msg.robot2.id = task_planner.robot_status_list[1][0]
        msg.robot2.status = task_planner.robot_status_list[1][1]
        msg.robot2.task = task_planner.robot_status_list[1][2]
        msg.robot2.goal = task_planner.robot_status_list[1][3]
        
        msg.robot3.id = task_planner.robot_status_list[2][0]
        msg.robot3.status = task_planner.robot_status_list[2][1]
        msg.robot3.task = task_planner.robot_status_list[2][2]
        msg.robot3.goal = task_planner.robot_status_list[2][3]
        
        self.publisher.publish(msg)
        
        
class TaskPublisher1(Node):
    def __init__(self):
        log.info("TaskPublisher started.")
        
        super().__init__('send_task_publisher')
        self.publisher = self.create_publisher(Task, '/task_1', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        
        
    def timer_callback(self):
        task_planner.robot, task_planner.item, task_planner.q, task_planner.robot_status_list = task_planner.give_robot_task()
        
        # if (task_planner.item != None) and (task_planner.robot == 1):
        if task_planner.item != None:
            log.info(task_planner.item.waypoints)
            
            msg = Task()
            msg.header.frame_id = "map"
            msg.header.stamp = self.get_clock().now().to_msg()
            
            x = task_planner.item.waypoints.split(",")[0].replace("[", "")
            log.info(x)
            
            y = task_planner.item.waypoints.split(",")[1]
            log.info(y)
            
            z = task_planner.item.waypoints.split(",")[2].replace("]", "")
            log.info(z)
            
            msg.position.x = float(x)
            msg.position.y = float(y)
            msg.position.z = float(z)
            
            # path planning을 여기에서 해야 할 것 같다
            
            self.publisher.publish(msg)
        
        
class TaskQueuePublisher(Node):
    def __init__(self):
        super().__init__('send_queue_publisher')
        self.publisher = self.create_publisher(TaskQueue, '/task_queue', 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.timer_callback)
        
    def timer_callback(self):
        msg = TaskQueue()
        
        task_planner.q = task_planner.add_task()
        
        # log.info(len(task_planner.q.queue))
        
        if len(task_planner.q.queue) > 0:
            
            for i in range(len(task_planner.q.queue)):
                item = TaskQueueItem()
                item.task_type = task_planner.q.queue[i].task_type
                item.place = task_planner.q.queue[i].place
                
                # log.info((item.task_type, item.place))
                
                msg.data.append(item)
        
        for v in msg.data:
            log.info(v)
            
        self.publisher.publish(msg)
        
        
class DoneTaskSubscriber1(Node):
    def __init__(self):
        super().__init__('done_task_subscriber_1')
        self.subscription = self.create_subscription(
            String,
            'robot1/done_task',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        log.info("listening..." + msg.data)
        
        if msg.data == 'OK':
            task_planner = TaskPlanning()
            task_planner.get_done(1)
            
            
class DoneTaskSubscriber2(Node):
    def __init__(self):
        super().__init__('done_task_subscriber_2')
        self.subscription = self.create_subscription(
            String,
            'robot2/done_task',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        log.info("listening..." + msg.data)
        
        if msg.data == 'OK':
            task_planner = TaskPlanning()
            task_planner.get_done(2)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    task_subscriber = TaskSubscriber()  # UI에서 요청 받음
    task_queue_publisher = TaskQueuePublisher()  # UI로 로봇 할당 안된 업무 목록 보내기
    robot_status_publisher = RobotStatusPublisher()  # UI로 로봇 상태 보내기
    task_publisher = TaskPublisher1()  # 로봇 1에 좌표 보내기
    done_task_1 = DoneTaskSubscriber1()  # 로봇 1에서 업무완료여부 받기
    
    executor.add_node(task_subscriber)
    executor.add_node(task_queue_publisher)
    executor.add_node(robot_status_publisher)
    executor.add_node(task_publisher)
    executor.add_node(done_task_1)
    
    executor.spin()
    

if __name__ == '__main__':
    main()