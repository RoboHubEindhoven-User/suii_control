from Task import Task
from TaskProtocol import TaskProtocol

class TaskWithAction(Task):
    def __init__(self):
        Task.__init__(self)
        self.action = -1
        self.action_str = TaskProtocol.lookUpKey(TaskProtocol.taskActionDict, self.action)

    def setAction(self, id):
        self.action = id
        self.action_str = TaskProtocol.lookUpKey(TaskProtocol.taskActionDict, id)
    
    def copyFromTask(self, task):
        self.setType(task.type)
        self.setSrc(task.src)
        self.setDest(task.dest)
        self.setObj(task.obj)
        self.setContainer(task.container)
    
    def __repr__(self):
        return "<<'%s'>> %s" % (self.action_str, Task.__repr__(self))
