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
        
        # todo: ui 실행 후 태스크 큐 하나 추가 후 다음부터 req를 못 받음
        # log.info(req)
        
        if req != None:
            t1 = Task(task_type_id = req.task_type_id,
                      goal_point = [req.position.x, req.position.y, req.position.z],
                      place = req.place)
            dm.insert_task(t1)

        # DB task 테이블에 현재 들어있는 값 가져오기
        task_list = dm.select_task_not_started()
        
        # 현재 큐에 있는 태스크
        id_list = []
        for item in self.q.queue:
            id_list.append(item.id)
            
        for id in id_list:
            log.info(id)

        # (현재 큐에 없는 것만) 큐에 추가
        for task in task_list:
            if task[0] not in id_list:
                self.q.put(Task(id = task[0], task_type_id = task[1], goal_point = task[2], task_type = task[3], place = task[4]))
            
        return self.q
        
        
    def task_start(self, robot, item):
        dm.update_task_robot_id(robot, item)
        dm.update_robot_working(robot)
        self.q.get()
    
    
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
            
            try:
                self.robot = dm.select_waiting_robot()
                
                if self.robot != 0:
                    self.task_start(self.robot, self.item)
                    
                    return self.robot, self.item, self.q, self.robot_status_list
            
            except Exception as e:
                log.error(f"task_planning main: {e}")
                
        return self.robot, self.item, self.q, self.robot_status_list


if __name__ == "__main__":
    tp = TaskPlanning()
    tp.give_robot_task()