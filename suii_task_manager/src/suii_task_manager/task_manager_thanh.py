from task_with_action import TaskWithAction 
from protocol.enum_task_action import TaskActionType

#TODO: discuss data type of result

class TaskManager:
    def format_drive(self, dest, result):
        twa = TaskWithAction()
        twa.set_destination(dest)
        twa.set_action(TaskActionType.DRIVE)
        result.append(twa)

    def format_pick_task(self, task, result):
        twa = TaskWithAction()
        twa.copy_from_task(task)
        twa.set_action(TaskActionType.PICK)
        result.append(twa)