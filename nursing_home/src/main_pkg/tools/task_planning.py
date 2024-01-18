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
        
        
    def select_task(self):
        # DB task 테이블에 현재 들어있는 값 가져오기
        task_list = dm.select_task_not_started()
        
        # 현재 큐에 있는 태스크
        id_list = []
        for item in self.q.queue:
            id_list.append(item.id)
            
        for id in id_list:
            log.info(id)

        # (현재 큐에 없는 것만) 큐에 추가
        
        if task_list != None:
            for task in task_list:
                if task[0] not in id_list:
                    self.q.put(Task(id = task[0], task_type_id = task[1], goal_point = task[2], task_type = task[3], place = task[4]))
                    
        return self.q.queue
    
    
    def add_task(self, req):        
        task = Task(task_type_id = req.task_type_id,
                    goal_point = [req.position.x, req.position.y, req.position.z],
                    place = req.place)
        dm.insert_task(task)
        
        self.q.queue = self.select_task()
            
        return self.q.queue
    
    
    def get_done(self, robot):
        # 로봇이 배정되어서 시작 후 종료되지 않은 업무 조회
        done_task = dm.select_task_done(robot)
        # 업무 완료 처리
        dm.update_task_done(done_task)
        
        # 업무중이던 로봇을 대기 상태로 변경
        dm.update_robot_waiting(robot)
        
        
    def show_robot_status(self):
        self.robot_status_list = dm.select_all_robot_status()
        
        return self.robot_status_list


    def give_robot_task(self):
        # 로봇에 업무 추가 + 큐에서 삭제
        while not self.q.empty():
            d = self.q.queue
            self.item = d[0]
            
            log.info(("self.item : ", self.item))
            
            try:
                self.robot = dm.select_waiting_robot()
                
                if self.robot != 0:
                    dm.give_robot_task(self.robot, self.item)
                    self.q.get()
                    
                    return self.robot, self.item, self.q, self.robot_status_list
            
            except Exception as e:
                log.error(f"task_planning main: {e}")
                
        return self.robot, self.item, self.q, self.robot_status_list


if __name__ == "__main__":
    tp = TaskPlanning()
    tp.give_robot_task()