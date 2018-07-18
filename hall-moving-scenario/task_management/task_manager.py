import heapq

from enum import Enum
from task import Task

KNOWN_LABELS = {8: 'Chair', 10 : 'Table', 15: 'Plant', 17: 'Sofa', 19: 'Monitor'}
TASK_NO = 1


class TaskType(Enum):
    SHOW_ON_MAP = 1
    GO_TO = 2
    SAY_SOMETHING = 3
    FIND_OBJECT = 4


class ClassType(Enum):
    PERSON = 1
    OBJECT = 2
    QRCODE = 3


class TaskManager:
    def __init__(self):
        self.encountered_people = {}
        self.encountered_locations = {}

        self.ongoing_tasks = []


    def create_tasks(self, type, data, value=None):
        tasks = []
        if type == TaskType.SHOW_ON_MAP:
            tasks = self.create_tasks_show(data, value)
        elif type == TaskType.GO_TO:
            tasks = self.create_tasks_go(data)
        elif type == TaskType.SAY_SOMETHING:
            tasks = self.create_tasks_say(data)
        elif type == TaskType.FIND_OBJECT:
            tasks = self.create_tasks_find(data)

        for task in tasks:
            heapq.heappush(self.ongoing_tasks, (task.priority, task))
        return tasks


    def create_tasks_show(self, data, value):
        tasks = []
        if value == 'people':
            for info in data: # info = [id, position, name]
                if len(info) > 2:
                    if info[2] in self.encountered_people:
                        self.encountered_people[info[2]].coordinates = info[1]
                    else:
                        show_task = Task(
                            type=TaskType.SHOW_ON_MAP,
                            class_type=ClassType.PERSON,
                            label=info[2],
                            coordinates=info[1]
                        )
                        self.encountered_people[info[2]] = show_task
                        tasks.append(show_task)

        elif value == 'qrcodes':
            for (label, position) in data: # info = [label, position]
                if label in self.encountered_locations:
                    self.encountered_locations[label].coordinates = position
                else:
                    show_task = Task(
                        type=TaskType.SHOW_ON_MAP,
                        class_type=ClassType.QRCODE,
                        label=label,
                        coordinates=position
                    )
                    self.encountered_locations[label] = show_task
                    tasks.append(show_task)                   

        elif value == 'objects':
            for (class_id, position) in data: # info = [class_id, position]
                if class_id in KNOWN_LABELS:
                    show_task = Task(
                        type=TaskType.SHOW_ON_MAP,
                        class_type=ClassType.OBJECT,
                        label=str(KNOWN_LABELS[class_id]),
                        coordinates=position
                    )
                    tasks.append(show_task)

        return tasks


    def create_tasks_go(self, data):
        pass


    def create_tasks_say(self, data):
        pass


    def create_tasks_find(self, data):
        pass


    def get_next_task(self):
        for task in self.tasks:
            if not task.is_done():
                return task
        return None

