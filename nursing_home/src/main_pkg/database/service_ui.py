from utils.custom_logger import Logger
from database.dao import DB

log = Logger(__name__)
db = DB()


class DataManager:        
            
    def select_all_task_type(self):
        try:
            query = "SELECT meaning, id FROM task_type order by id"
            db.execute(query)
            task_type_list = db.fetchAll()
            
            log.info(task_type_list)
            
            return task_type_list
        
        except Exception as e:
            log.error(f"select_all_task_type : {e}")
            
            
    def select_all_place(self):
        try:
            query = "SELECT ko_name, x, y, z FROM place order by id"
            db.execute(query)
            place_list = db.fetchAll()
            
            return place_list
        
        except Exception as e:
            log.error(f"select_all_place : {e}")