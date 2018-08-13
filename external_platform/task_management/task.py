from task_management.i_task import ITask, TaskStatus


class SayTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, text):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.text = text

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.text, self.on_success, self.on_fail)

    def stringify(self):
        return '\n%s: \tPrior: %d \tStatus: %s \tText: %s' % (self.__class__.__name__, self.priority, str(self.status), self.text[:20])


class MoveToTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, target):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.target = target

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.target, self.on_success, self.on_fail)

    def stringify(self):
        if self.target:
            position = self.target.pose.position
            position = '(%.2f %.2f %.2f)' % (position.x, position.y, position.z)
        else:
            position = 'none'
        return '\n%s: \tPrior: %d \tStatus: %s \tTarget: %s' % (self.__class__.__name__, self.priority, str(self.status), position)


class SearchPersonTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, name):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.name = name

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.name, self.on_success, self.on_fail)

    def stringify(self):
        return '\n%s: \tPrior: %d \tStatus: %s \tPerson: %s' % (self.__class__.__name__, self.priority, str(self.status), self.name)


class ShowURLTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, url):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.url = url

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.url, self.on_success, self.on_fail)

    def stringify(self):
        return '\n%s: \tPrior: %d \tStatus: %s \tURL: %s' % (self.__class__.__name__, self.priority, str(self.status), self.url)


