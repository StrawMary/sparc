from task_management.i_task import ITask, TaskStatus

import tasks_config as cfg


class SayTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, text):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.text = text

    def run(self):
        self.status = TaskStatus.RUNNING
        if callable(self.text):
            self.text = self.text()
        self.run_method(self.text, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Target=%s' % \
               (self.__class__.__name__, self.priority, str(self.status),
                str(self.text) if len(str(self.text)) < cfg.max_display_length else str(self.text)[:(cfg.max_display_length-3)] + '...')


class ListenTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, keywords):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.keywords = keywords
        self.result = None

    def save_result(self, text):
        self.result = text

    def get_result(self):
        return self.result

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.keywords, self.save_result, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Target=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), self.keywords)


class MoveToTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, target, offset = 0):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.target = target
        self.offset = offset

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.target, self.offset, self.on_success, self.on_fail)

    def stringify(self):
        if isinstance(self.target, basestring):
            return self.target

        if self.target:
            position = self.target.pose.position
            position = '(%.2f %.2f %.2f)' % (position.x, position.y, position.z)
        else:
            position = 'none'
        return '%16s: Prior=%d Status=%-20s Target=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), position)


class SearchTargetTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, target, result_key=None):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.target = target
        self.result_key = result_key

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.target, self.result_key, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Target=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), self.target)


class WaitTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, target):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.target = target

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.target, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Target=%s' % \
               (self.__class__.__name__, self.priority, str(self.status),
                self.target)


class ShowURLTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, url):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.url = url

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.url, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Target=%s' % \
               (self.__class__.__name__, self.priority, str(self.status),
                self.url if len(self.url) < cfg.max_display_length else self.url[:(cfg.max_display_length-3)] + '...')


class ActuationTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, target, optional_entities=None):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.target = target
        self.optional_entities = optional_entities

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.target, self.optional_entities, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Target=%s Optional=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), self.target, self.optional_entities)


class RememberTargetTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, target, target_type):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.target = target
        self.target_type = target_type

    def run(self):
        self.status = TaskStatus.RUNNING
        if callable(self.target):
            self.target = self.target()
        self.run_method(self.target, self.target_type, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Target=%s Target_type=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), str(self.target), self.target_type)


class PoseTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, pose_name):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.pose_name = pose_name

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.pose_name, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Pose=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), str(self.pose_name))


class ChoregrapheBehaviourTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, beh_name):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.behaviour_name = beh_name

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.behaviour_name, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Behaviour=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), str(self.behaviour_name))


class RecognizeActivityTask(ITask):
    def __init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority, activity):
        ITask.__init__(self, run_method, stop_method, success_child, fail_child, add_to_queue_method, priority)
        self.activity = activity
        self.result = None

    def save_result(self, recognized_activity):
        self.result = recognized_activity

    def get_result(self):
        return self.result

    def run(self):
        self.status = TaskStatus.RUNNING
        self.run_method(self.activity, self.save_result, self.on_success, self.on_fail)

    def stringify(self):
        return '%16s: Prior=%d Status=%-20s Activity=%s' % \
               (self.__class__.__name__, self.priority, str(self.status), str(self.activity))
