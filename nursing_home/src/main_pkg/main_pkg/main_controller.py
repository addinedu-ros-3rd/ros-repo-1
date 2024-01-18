from utils.custom_logger import Logger
from tools.task_planning import TaskPlanning
from tools.astar_planning import AStarPlanner

import math
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from interfaces_pkg.msg import *


log = Logger(__name__)
task_planner = TaskPlanning()

global amcl_1
global amcl_2
global amcl_3
amcl_1 = PoseWithCovarianceStamped()
amcl_2 = PoseWithCovarianceStamped()
amcl_3 = PoseWithCovarianceStamped()

TIMER_PERIOD = 0.5

class TaskRequestSubscriber(Node):

    def __init__(self):
        self.received_task = None
        
        super().__init__("task_request_subscriber")
        self.subscription = self.create_subscription(
            TaskRequest,
            "task_request",
            self.task_request_callback,
            1)


    def task_request_callback(self, msg):
        # log.info('task_request subscribed')
        task_planner.add_task(msg)


class AMCLSubscriber(Node):
    def __init__(self):
        super().__init__('amcl_subscriber')

        amcl_pose_qos = QoSProfile(
          durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
          reliability=QoSReliabilityPolicy.RELIABLE,
          history=QoSHistoryPolicy.KEEP_LAST,
          depth=1)
        
        self.amcl_subscriber_1 = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_1', self.amcl_callback_1, amcl_pose_qos) # 아직 어떤 로봇 불러올지 안맞춤
        self.amcl_subscriber_2 = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_2', self.amcl_callback_2, amcl_pose_qos) # 아직 어떤 로봇 불러올지 안맞춤
        self.amcl_subscriber_3 = self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose_3', self.amcl_callback_3, amcl_pose_qos) # 아직 어떤 로봇 불러올지 안맞춤


    def amcl_callback_1(self, msg):
        global amcl_1
        amcl_1 = msg

    def amcl_callback_2(self, msg):
        global amcl_2
        amcl_2 = msg

    def amcl_callback_3(self, msg):
        global amcl_3
        amcl_3 = msg


class AStarPublisher1(Node):
    def __init__(self) :
        super().__init__('astar_publisher_1')

        self.astar_publisher = self.create_publisher(AstarMsg, '/astar_paths_1', 10)
        self.goal_subscriber = self.create_subscription(TaskRequest, '/task_1', self.task_callback, 10)
        self.astar_planner = AStarPlanner(resolution=0.7, rr=0.4, padding=4)
            

    def task_callback(self, msg):
        self.now_x = amcl_1.pose.pose.position.x
        self.now_y = amcl_1.pose.pose.position.y

        pose_stamp = PoseWithCovarianceStamped()
        pose_stamp.header.frame_id = 'map'

        rx, ry, tpx, tpy, tvec_x, tvec_y = self.astar_planner.planning(self.now_x, self.now_y, msg.position.x, msg.position.y)

        astar_paths = []
        astar_paths_msg = AstarMsg()
        astar_paths_msg.length = len(tpx)

        for i in range(len(tpx)):
            tmp = Pose()
            tmp.position.x = tpx[i]
            tmp.position.y = tpy[i]

            q = quaternion_from_euler(0, 0, math.atan2(tvec_y[i], tvec_x[i]), 'rxyz')
            tmp.orientation.z = q[2]
            tmp.orientation.w = q[3]

            astar_paths.append(tmp)

        astar_paths_msg.poses = astar_paths

        self.astar_publisher.publish(astar_paths_msg)
        
        
class AStarPublisher2(Node):
    def __init__(self) :
        super().__init__('astar_publisher_2')

        self.astar_publisher = self.create_publisher(AstarMsg, '/astar_paths_2', 10)
        self.goal_subscriber = self.create_subscription(TaskRequest, '/task_2', self.task_callback, 10)
        self.astar_planner = AStarPlanner(resolution=0.7, rr=0.4, padding=4)
            

    def task_callback(self, msg):
        self.now_x = amcl_1.pose.pose.position.x
        self.now_y = amcl_1.pose.pose.position.y

        pose_stamp = PoseWithCovarianceStamped()
        pose_stamp.header.frame_id = 'map'

        rx, ry, tpx, tpy, tvec_x, tvec_y = self.astar_planner.planning(self.now_x, self.now_y, msg.position.x, msg.position.y)

        astar_paths = []
        astar_paths_msg = AstarMsg()
        astar_paths_msg.length = len(tpx)

        for i in range(len(tpx)):
            tmp = Pose()
            tmp.position.x = tpx[i]
            tmp.position.y = tpy[i]

            q = quaternion_from_euler(0, 0, math.atan2(tvec_y[i], tvec_x[i]), 'rxyz')
            tmp.orientation.z = q[2]
            tmp.orientation.w = q[3]

            astar_paths.append(tmp)

        astar_paths_msg.poses = astar_paths

        self.astar_publisher.publish(astar_paths_msg)
        
        
class AStarPublisher3(Node):
    def __init__(self) :
        super().__init__('astar_publisher_3')

        self.astar_publisher = self.create_publisher(AstarMsg, '/astar_paths_3', 10)
        self.goal_subscriber = self.create_subscription(TaskRequest, '/task_3', self.task_callback, 10)
        self.astar_planner = AStarPlanner(resolution=0.7, rr=0.4, padding=4)
            

    def task_callback(self, msg):
        self.now_x = amcl_1.pose.pose.position.x
        self.now_y = amcl_1.pose.pose.position.y

        pose_stamp = PoseWithCovarianceStamped()
        pose_stamp.header.frame_id = 'map'

        rx, ry, tpx, tpy, tvec_x, tvec_y = self.astar_planner.planning(self.now_x, self.now_y, msg.position.x, msg.position.y)

        astar_paths = []
        astar_paths_msg = AstarMsg()
        astar_paths_msg.length = len(tpx)

        for i in range(len(tpx)):
            tmp = Pose()
            tmp.position.x = tpx[i]
            tmp.position.y = tpy[i]

            q = quaternion_from_euler(0, 0, math.atan2(tvec_y[i], tvec_x[i]), 'rxyz')
            tmp.orientation.z = q[2]
            tmp.orientation.w = q[3]

            astar_paths.append(tmp)

        astar_paths_msg.poses = astar_paths

        self.astar_publisher.publish(astar_paths_msg)
        
        
class RobotStatusPublisher(Node):
    
    def __init__(self):
        super().__init__("send_robot_status_publisher")
        self.publisher = self.create_publisher(RobotStatusList, "/robot_status", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.robot_status_timer_callback)
        
        
    def robot_status_timer_callback(self):
        msg = RobotStatusList()
        
        task_planner.robot_status_list = task_planner.show_robot_status()
        
        if task_planner.robot_status_list is not None:
        
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
        
        
class TaskPublisher(Node):
    def __init__(self):
        super().__init__("task_publisher_1")
        self.task_1_publisher = self.create_publisher(TaskRequest, "/task_1", 10)
        self.task_2_publisher = self.create_publisher(TaskRequest, "/task_2", 10)
        self.task_3_publisher = self.create_publisher(TaskRequest, "/task_3", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.task_timer_callback)
        
        
    def task_timer_callback(self):
        task_planner.robot, task_planner.item, task_planner.q, task_planner.robot_status_list = task_planner.give_robot_task()
        log.info(("robot, item in task_timer_callback: ", task_planner.robot, task_planner.item))
        
        if task_planner.item is not None:
            log.info(task_planner.item.goal_point)
            
            msg = TaskRequest()
            
            x = task_planner.item.goal_point.split(",")[0].replace("[", "")
            log.info(x)
            
            y = task_planner.item.goal_point.split(",")[1]
            log.info(y)
            
            z = task_planner.item.goal_point.split(",")[2].replace("]", "")
            log.info(z)
            
            msg.position.x = float(x)
            msg.position.y = float(y)
            msg.position.z = float(z)
            
            if task_planner.robot == 1:
                self.task_1_publisher.publish(msg)
                task_planner.item = None
                task_planner.robot = None
            elif task_planner.robot == 2:
                self.task_2_publisher.publish(msg)
                task_planner.item = None
                task_planner.robot = None
            elif task_planner.robot == 3:
                self.task_3_publisher.publish(msg)
                task_planner.item = None
                task_planner.robot = None
        
        
class TaskQueuePublisher(Node):
    def __init__(self):
        super().__init__('task_queue_publisher')
        self.publisher = self.create_publisher(TaskQueue, '/task_queue' , 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.task_queue_timer_callback)
        
    def task_queue_timer_callback(self):
        msg = TaskQueue()
        
        task_queue = task_planner.select_task()

        if len(task_queue) > 0:
            
            for i in range(len(task_queue)):
                item = TaskQueueItem()
                item.task_type = task_queue[i].task_type
                item.place = task_queue[i].place
                
                # log.info((item.task_type, item.place))
                
                msg.data.append(item)
        
        for v in msg.data:
            log.info(v)
            
        self.publisher.publish(msg)


class DoneTaskSubscriber(Node):
    def __init__(self):
        super().__init__('done_task_subscriber')
        
        self.done_task_1 = self.create_subscription(String, '/done_task_1', self.done_task_callback_1, 10)
        self.done_task_2 = self.create_subscription(String, '/done_task_2', self.done_task_callback_2, 10)
        self.done_task_3 = self.create_subscription(String, '/done_task_3', self.done_task_callback_3, 10)

    def done_task_callback_1(self, msg):
        if msg.data == 'OK':
            task_planner = TaskPlanning()
            task_planner.get_done(1)

    def done_task_callback_2(self, msg):
        if msg.data == 'OK':
            task_planner = TaskPlanning()
            task_planner.get_done(2)

    def done_task_callback_3(self, msg):
        if msg.data == 'OK':
            task_planner = TaskPlanning()
            task_planner.get_done(3)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    task_request_subscriber = TaskRequestSubscriber()  # UI에서 요청 받음
    task_queue_publisher = TaskQueuePublisher()  # UI로 로봇 할당 안된 업무 목록 보내기
    robot_status_publisher = RobotStatusPublisher()  # UI로 로봇 상태 보내기
    
    task_publisher = TaskPublisher()  # 로봇에 좌표 보내기
    astar_publisher_1 = AStarPublisher1()  # 로봇 이동 경로 publish
    astar_publisher_2 = AStarPublisher2()
    astar_publisher_3 = AStarPublisher3()
    executor.add_node(task_publisher)
    executor.add_node(astar_publisher_1)
    executor.add_node(astar_publisher_2)
    executor.add_node(astar_publisher_3)
        
    amcl_subscriber = AMCLSubscriber()  # 로봇들 amcl_pose 갱신
    done_task_subscriber = DoneTaskSubscriber()  # 로봇에서 업무완료여부 받기
    executor.add_node(amcl_subscriber)
    executor.add_node(task_request_subscriber)
    executor.add_node(task_queue_publisher)
    executor.add_node(robot_status_publisher)
    executor.add_node(done_task_subscriber)
    
    executor.spin()
    

if __name__ == '__main__':
    main()