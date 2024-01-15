class Task:
    def __init__(self, id=None, task_type_id=None, waypoints=None, task_type=None, place=None):
        self._id = id
        self._task_type_id = task_type_id
        self._task_type = task_type
        self._waypoints = waypoints
        self._place = place
    
    @property
    def id(self):
        return self._id
    
    @property
    def task_type_id(self):
        return self._task_type_id
    
    @property
    def task_type(self):
        return self._task_type
    
    @property
    def waypoints(self):
        return self._waypoints
    
    @property
    def place(self):
        return self._place