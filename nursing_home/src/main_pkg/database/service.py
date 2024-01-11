from rclpy.logging import get_logger as log
from database.dao import DB


db = DB()


class DataManager:
    
    def insert_task(self, task):
        try:
            query = "INSERT INTO task (task_type_id, waypoints) values (%s, %s)"
            db.execute(query, (task.task_type_id, str(task.waypoints)))
            
        except Exception as e:
            log.error(f"insert_task : {e}")
            
            
    def select_task_not_started(self):
        try:
            query = "SELECT id, task_type_id, waypoints \
                     FROM task \
                     WHERE robot_id IS NULL"
            db.execute(query)
            task_list = db.fetchAll()
            
            return task_list
            
        except Exception as e:
            log.error(f"select_task_not_started : {e}")
            
            
    def select_waiting_robot(self):
        try:
            query = "SELECT id FROM robot WHERE robot_status_id = 2"
            db.execute(query)
            waiting_robot = db.fetchOne()
            
            return waiting_robot
        
        except Exception as e:
            log.error(f"select_not_working_robot : {e}")
            
            
    def update_task_robot_id(self, robot, task):
        try:
            query = "UPDATE task SET robot_id = (%s), started_at = now() WHERE id = (%s)"
            db.execute(query, (robot, task.id))
            
        except Exception as e:
            log.error(f"update_task_robot_id : {e}")
            
            
    def update_robot_working(self, robot):
        try:
            # 상태를 WORK로, 모드를 AUTO로 업데이트
            query = "UPDATE robot SET robot_status_id = 3, robot_work_mode_id = 2 WHERE id = (%s)"
            db.execute(query, (robot,))
            
        except Exception as e:
            log.error(f"update_robot_status : {e}")
            
            
    def update_task_done(self, task):
        try:
            query = "UPDATE task SET finished_at = now() WHERE id = (%s)"
            db.execute(query, (task.id,))
            
        except Exception as e:
            log.error(f"update_task_done : {e}")
            
            
    def update_robot_waiting(self, robot):
        try:
            # 상태를 WAIT으로, 모드를 NONE으로 업데이트
            query = "UPDATE robot SET robot_status_id = 2, robot_work_mode_id = 1 WHERE id = (%s)"
            db.execute(query, (robot,))
            
        except Exception as e:
            log.error(f"update_robot_waiting : {e}")