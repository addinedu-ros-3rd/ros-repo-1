from queue import Queue

from utils.custom_logger import Logger
from database.service import DataManager
from database.dto import Task

from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped


log = Logger(__name__)
dm = DataManager()

q = Queue()

class TaskPlanning():
    def __init__(self):
        log.info("TaskPlanning started.")
    
    
    def add_task(self, req):
        
        # req에 따라 DB에서 좌표(x, y, z) 가져오기
        
        # Task 객체 생성
        t1 = Task(task_type_id = 2,  # 휴지
                  waypoints = [
                               [-1.7216078042984009, 0.6827247142791748, 0.030487060546875],
                               [-0.36505940556526184, 0.6492809653282166, 0.002471923828125],
                               [0.7526519298553467, 0.6099578142166138, 0.002471923828125]
                            ]
                  )
        
        # t2 = Task(task_type_id = 1,  # 식사
        #           waypoints = [
        #                         [1.9273842573165894, 0.6419123411178589, -0.001434326171875], 
        #                         [1.8944379091262817, -0.4697618782520294, -0.001434326171875]
        #                     ]
        #           )
        
        # DB에 추가
        dm.insert_task(t1)
        # dm.insert_task(t2)

        # DB task 테이블에 현재 들어있는 값 가져오기
        task_list = dm.select_task_not_started()

        # 큐에 추가
        for task in task_list:
            q.put(Task(id = task[0], task_type_id = task[1], waypoints = task[2]))
            
        log.info(("현재 큐 사이즈: ", q.qsize()))
        
        
    def task_start(self, robot, item):
        dm.update_task_robot_id(robot, item)
        dm.update_robot_working(robot)
        q.get()
        
        
    def make_poses(self, nav, item):
        goal_poses = []
        
        log.info(("item.waypoints: ", item.waypoints))
        
        waypoints_length = item.waypoints.count("],") + 1
        
        for i in range(waypoints_length):
            
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = nav.get_clock().now().to_msg()
            
            x = item.waypoints.split("],")[i].split(",")[0].replace("[[", "").replace(" [", "")
            log.info(x)
            
            y = item.waypoints.split("],")[i].split(",")[1].replace(" ", "")
            log.info(y)
            
            z = item.waypoints.split("],")[i].split(",")[2].replace(" ", "").replace("]]", "")
            log.info(z)
            
            goal_pose.pose.position.x = float(x)
            goal_pose.pose.position.y = float(y)
            goal_pose.pose.position.z = float(z)
            
            goal_pose.pose.orientation.x = 0.0
            goal_pose.pose.orientation.y = 0.0
            goal_pose.pose.orientation.z = 0.0
            goal_pose.pose.orientation.w = 0.0
            
            goal_poses.append(goal_pose)
            
        log.info(("goal_poses: ", goal_poses))
        
        return goal_poses


    def main(self, req=None):
        
        self.add_task(req)

        # 로봇에 업무 추가 + 큐에서 삭제
        while not q.empty():
            d = q.queue
            first_item = d[0]
            
            try:
                robot = dm.select_waiting_robot()
                log.info(("now waiting robot: ", robot))
                
                if robot == 1:
                    self.task_start(robot, first_item)
                    nav_1 = BasicNavigator(namespace="minibot_hongki")
                    
                    goal_poses = self.make_poses(nav_1, first_item)
                    
                    nav_1.goThroughPoses(goal_poses)
                        
                    if nav_1.isTaskComplete():
                        log.info("nav_1 Task Complete")
                        dm.update_task_done(first_item)
                        dm.update_robot_waiting(robot)
                        
                elif robot == 2:
                    self.task_start(robot, first_item)
                    nav_2 = BasicNavigator(namespace="minibot_haneol")
                    
                    goal_poses = self.make_poses(nav_2, first_item)
                    
                    nav_2.goThroughPoses(goal_poses)
                        
                    if nav_2.isTaskComplete():
                        log.info("nav_2 Task Complete")
                        dm.update_task_done(first_item)
                        dm.update_robot_waiting(robot)
                        
                elif robot == 3:
                    self.task_start(robot, first_item)
                    nav_3 = BasicNavigator(namespace="minibot_taesang")
                    
                    goal_poses = self.make_poses(nav_3, first_item)
                    
                    nav_3.goThroughPoses(goal_poses)
                        
                    if nav_3.isTaskComplete():
                        log.info("nav_3 Task Complete")
                        dm.update_task_done(first_item)
                        dm.update_robot_waiting(robot)
                        
                else:
                    log.error(f"NO ROBOT")
            
            except Exception as e:
                log.error(f"task_planning main: {e}")


if __name__ == "__main__":
    tp = TaskPlanning()
    tp.main()