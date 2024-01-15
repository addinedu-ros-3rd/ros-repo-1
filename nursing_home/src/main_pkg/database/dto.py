class Task:
    def __init__(self, id=None, task_type_id=None, goal_point=None, task_type=None, place=None):
        self._id = id
        self._task_type_id = task_type_id
        self._task_type = task_type
        self._goal_point = goal_point
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
    def goal_point(self):
        return self._goal_point
    
    @property
    def place(self):
        return self._place