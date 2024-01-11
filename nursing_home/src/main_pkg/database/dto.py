class Task:
    def __init__(self, id=None, task_type_id=None, waypoints=None):
        self._id = id
        self._task_type_id = task_type_id
        self._waypoints = waypoints
    
    @property
    def id(self):
        return self._id
    
    @property
    def task_type_id(self):
        return self._task_type_id
    
    @property
    def waypoints(self):
        return self._waypoints