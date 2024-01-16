from utils.custom_logger import Logger
from tools.task_planning import TaskPlanning
from tools.astar_planning import AStarPlanner

import math
from tf_transformations import quaternion_from_euler

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSReliabilityPolicy, QoSProfile
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose
from interfaces_pkg.msg import *


log = Logger(__name__)
task_planner = TaskPlanning()

amcl_1 = PoseWithCovarianceStamped()
amcl_2 = PoseWithCovarianceStamped()
amcl_3 = PoseWithCovarianceStamped()

TIMER_PERIOD = 0.5

class TaskRequestSubscriber(Node):

    def __init__(self):
        log.info("TaskRequestSubscriber started.")
        self.received_task = None
        
        super().__init__("task_request_subscriber")
        self.subscription = self.create_subscription(
            TaskRequest,
            "task_request",
            self.task_request_callback,
            10)


    def task_request_callback(self, msg):
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
        amcl_1 = msg

    def amcl_callback_2(self, msg):
        amcl_2 = msg

    def amcl_callback_3(self, msg):
        amcl_3 = msg


class AStarPublisher(Node):
    def __init__(self, robot) :
        super().__init__('astar_publisher_' + str(robot))

        self.astar_publisher = self.create_publisher(AstarMsg, '/astar_paths_' + str(robot), 10)
        self.goal_subscriber = self.create_subscription(TaskRequest, '/task_' + str(robot), self.task_callback, 10)

        self.astar_planner = AStarPlanner(resolution=0.7, rr=0.3, padding=5)
            

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
        log.info("SendRobotStatusPublisher started.")
        
        super().__init__("send_robot_status_publisher")
        self.publisher = self.create_publisher(RobotStatusList, "/robot_status", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.robot_status_timer_callback)
        
        
    def robot_status_timer_callback(self):
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
        
        super().__init__("task_publisher_1")
        self.publisher = self.create_publisher(TaskRequest, "/task_1", 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.task_timer_callback)
        
        
    def task_timer_callback(self):
        task_planner.robot, task_planner.item, task_planner.q, task_planner.robot_status_list = task_planner.give_robot_task()
        
        if task_planner.item != None:
        # if (task_planner.item != None) and (task_planner.robot == 1):
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
            
            # path planning을 여기에서 해야 할 것 같다
            
            self.publisher.publish(msg)
            task_planner.item = None
        
        
class TaskQueuePublisher(Node):
    def __init__(self):
        super().__init__('task_queue_publisher')
        self.publisher = self.create_publisher(TaskQueue, '/task_queue' , 10)
        self.timer = self.create_timer(TIMER_PERIOD, self.task_queue_timer_callback)
        
    def task_queue_timer_callback(self):
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
            'done_task_1',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        log.info("done_task_1 : " + msg.data)
        
        if msg.data == 'OK':
            task_planner = TaskPlanning()
            task_planner.get_done(1)


def main():
    rclpy.init()
    executor = MultiThreadedExecutor()
    
    task_request_subscriber = TaskRequestSubscriber()  # UI에서 요청 받음
    task_queue_publisher = TaskQueuePublisher()  # UI로 로봇 할당 안된 업무 목록 보내기
    robot_status_publisher = RobotStatusPublisher()  # UI로 로봇 상태 보내기
    task_publisher = TaskPublisher1()  # 로봇 1에 좌표 보내기
    amcl_subscriber = AMCLSubscriber()  # 로봇들 amcl_pose 갱신
    astar_publisher_1 = AStarPublisher(robot=1)  # 로봇 이동 경로 publish
    done_task_1 = DoneTaskSubscriber1()  # 로봇 1에서 업무완료여부 받기
    
    executor.add_node(task_request_subscriber)
    executor.add_node(task_queue_publisher)
    executor.add_node(robot_status_publisher)
    executor.add_node(task_publisher)
    executor.add_node(amcl_subscriber)
    executor.add_node(astar_publisher_1)
    executor.add_node(done_task_1)
    
    executor.spin()
    

if __name__ == '__main__':
    main()