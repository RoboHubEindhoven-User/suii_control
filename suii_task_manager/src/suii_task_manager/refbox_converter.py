from suii_task_manager.task_manager import TaskManager
from suii_task_manager.task_protocol import TaskProtocol
from suii_task_manager.task_list import TaskList
from suii_task_manager.protocol.enum_task_type import TaskType
from suii_task_manager.task import Task

class RefBoxConverter:
    @staticmethod
    def transportation_task_to_task(task):
        source = TaskProtocol.look_up_value(TaskProtocol.location_dict, task.transportation_task.source.type.data)
        if (source == -1):
            return None
        
        destination = TaskProtocol.look_up_value(TaskProtocol.location_dict, task.transportation_task.destination.type.data)
        if (destination == -1):
            return None
        
        object_to_pick = TaskProtocol.look_up_value(TaskProtocol.object_dict, task.transportation_task.object.description.data)
        if (object_to_pick == -1):
            return None
            
        container = -1
        if (task.transportation_task.container.description.data != ""):
            container = TaskProtocol.look_up_value(TaskProtocol.container_dict, task.transportation_task.container.description.data)
            if (container == -1):
                return None
        
        # create a new task
        tmp_task = Task()
        tmp_task.set_type(TaskProtocol.look_up_value(TaskProtocol.task_type_dict, TaskType.TRANSPORTATION.fullname))
        tmp_task.set_source(source) 
        location = source if (source == -1) else destination
        tmp_task.set_destination(location)
        tmp_task.set_object(object_to_pick)
        tmp_task.set_container(container)

        return tmp_task

    @staticmethod 
    def navigation_task_to_task(task):
        destination = TaskProtocol.look_up_value(TaskProtocol.location_dict, task.transportation_task.destination.description.data)
        if (destination == -1):
            return None
        tmp_task = Task() 
        tmp_task.set_type (TaskProtocol.look_up_value(TaskProtocol.task_type_dict, TaskType.NAVIGATION.fullname)) 
        tmp_task.set_source(-1) 
        tmp_task.set_destination(destination)
        tmp_task.set_object(-1)
        tmp_task.set_container(-1)
        return tmp_task