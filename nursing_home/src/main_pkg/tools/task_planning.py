from queue import Queue

from utils.custom_logger import Logger
from database.service import DataManager
from database.dto import Task


log = Logger(__name__)
dm = DataManager()


class TaskPlanning():
    def __init__(self):
        log.info("TaskPlanning started.")
        self.q = Queue()
        self.robot = None
        self.item = None
        self.robot_status_list = dm.select_all_robot_status()
    
    
    def add_task(self, req=None):
        
        if req != None:
            t1 = Task(task_type_id = req.task_type_id,
                      waypoints = [req.position.x, req.position.y, req.position.z],
                      place = req.place)
            dm.insert_task(t1)

        # DB task 테이블에 현재 들어있는 값 가져오기
        task_list = dm.select_task_not_started()

        # 큐에 추가
        for task in task_list:
            self.q.put(Task(id = task[0], task_type_id = task[1], waypoints = task[2]))
            
        log.info('-----------add_task----------------')
        log.info(self.q.qsize())
        
        
    def task_start(self, robot, item):
        dm.update_task_robot_id(robot, item)
        dm.update_robot_working(robot)
        self.q.get()
        
        self.robot_status_list = dm.select_all_robot_status()
    
    
    def get_done(self, robot):
        # 로봇이 배정되어서 시작 후 종료되지 않은 업무 조회
        item = dm.select_task(robot)
        # 업무 완료 처리
        dm.update_task_done(item)
        
        # 업무중이던 로봇을 대기 상태로 변경
        dm.update_robot_waiting(robot)


    def main(self, req=None):
        
        self.add_task(req)

        # 로봇에 업무 추가 + 큐에서 삭제
        while not self.q.empty():
            d = self.q.queue
            self.item = d[0]
            
            try:
                self.robot = dm.select_waiting_robot()
                
                log.info(("now waiting robot: ", self.robot))
                
                if self.robot != None:
                    self.task_start(self.robot, self.item)
            
            except Exception as e:
                log.error(f"task_planning main: {e}")
                
                
        return self.robot, self.item, self.q, self.robot_status_list


if __name__ == "__main__":
    tp = TaskPlanning()
    tp.main()