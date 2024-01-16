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
            query = "SELECT id FROM robot WHERE robot_status_id = 2"
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
                    select ttt1.robot_id as final_robot_id,
                        ttt1.status as status,
                        CASE status WHEN '대기중' THEN '' ELSE tt.meaning END as task,
                        CASE status WHEN '대기중' THEN '' ELSE ttt1.place END as place
                    from (select tt1.id as robot_id, tt1.status, t.task_type_id, t.place, t.id
                        from (SELECT t1.id, t1.battery, t1.status, rwm.meaning
                            FROM (SELECT r.id, r.battery, rs.meaning as status, r.robot_work_mode_id
                                    FROM robot r
                                    join robot_status rs
                                    on r.robot_status_id = rs.id) t1
                            join robot_work_mode rwm
                            on t1.robot_work_mode_id = rwm.id) tt1
                        left join task t
                        on tt1.id = t.robot_id
                        order by t.id desc) ttt1
                    left join task_type tt
                    on ttt1.task_type_id = tt.id
                    group by final_robot_id
                    """
            robot_status_list = db.executeAndFetchAll(query)
            
            return robot_status_list
            
        except Exception as e:
            log.error(f"select_all_robot_status : {e}")