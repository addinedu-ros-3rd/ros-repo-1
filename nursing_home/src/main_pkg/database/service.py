from utils.custom_logger import Logger
from database.dao import DB

log = Logger(__name__)
db = DB()


class DataManager:
    
    def insert_task(self, task):
        try:
            query = "INSERT INTO task (task_type_id, goal_point, place) values (%s, %s, %s)"
            db.execute(query, (task.task_type_id, str(task.goal_point), task.place))
            
        except Exception as e:
            log.error(f"insert_task : {e}")
            
            
    def select_task_not_started(self):
        try:
            query = """
                    SELECT t.id, t.robot_id, t.goal_point, tt.meaning, t.place
                    FROM task t
                    JOIN task_type tt
                    ON t.task_type_id = tt.id
                    WHERE t.robot_id IS NULL
                    order by t.id
                    """
            task_list = db.executeAndFetchAll(query)
            
            return task_list
            
        except Exception as e:
            log.error(f"select_task_not_started : {e}")
            
            
    def select_waiting_robot(self):
        try:
            query = "SELECT id FROM robot WHERE robot_status_id = 2 order by id"
            waiting_robot = db.executeAndFetchOne(query)
            
            return waiting_robot
        
        except Exception as e:
            log.error(f"select_not_working_robot : {e}")
            
            
    def select_task_done(self, robot):
        try:
            query = """
                    SELECT id FROM task
                    WHERE robot_id = (%s)
                    AND started_at IS NOT NULL
                    AND finished_at IS NULL
                    """
            done_task = db.executeAndFetchOne(query, (robot,))
            log.info(("done_task: ", done_task))
            
            return done_task
        
        except Exception as e:
            log.error(f"select_task : {e}")
            
            
    def give_robot_task(self, robot, task):
        try:
            db.callProc('give_robot_task', (robot, task.id))
        
        except Exception as e:
            log.error(f" give_robot_task : {e}")
            
            
    def update_task_done(self, task_id):
        try:
            query = "UPDATE task SET finished_at = now() WHERE id = (%s)"
            db.execute(query, (task_id,))
            
        except Exception as e:
            log.error(f"update_task_done : {e}")
            
            
    def update_robot_waiting(self, robot):
        try:
            # 상태를 WAIT으로, 모드를 NONE으로 업데이트
            query = "UPDATE robot SET robot_status_id = 2, robot_work_mode_id = 1 WHERE id = (%s)"
            db.execute(query, (robot,))
            
        except Exception as e:
            log.error(f"update_robot_waiting : {e}")
            
            
    def select_all_robot_status(self):
        try:
            query = """
                    SELECT t2.id,
                        rs.meaning as robot_status, 
                        IFNULL(t2.meaning, '') as task_type,
                        IFNULL(t2.place, '') as goal
                    FROM (
                        SELECT t1.id, t1.robot_status_id, tt.meaning, t1.place
                        FROM (
                            SELECT r.id, r.robot_status_id, t.task_type_id, t.place
                            FROM robot r
                            LEFT JOIN task t ON r.id = t.robot_id AND t.finished_at IS NULL
                            LEFT JOIN (
                                SELECT robot_id, MAX(id) AS max_task_id
                                FROM task
                                GROUP BY robot_id
                            ) latest_task ON t.robot_id = latest_task.robot_id AND t.id = latest_task.max_task_id
                        ) t1
                        LEFT JOIN task_type tt ON t1.task_type_id = tt.id
                    ) t2
                    LEFT JOIN robot_status rs ON t2.robot_status_id = rs.id
                    order by id
                    """
            robot_status_list = db.executeAndFetchAll(query)
            
            return robot_status_list
            
        except Exception as e:
            log.error(f"select_all_robot_status : {e}")