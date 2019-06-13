from suii_protocol.task_protocol import TaskProtocol
from suii_mux_manager_comm.task_list import TaskList
from suii_protocol.protocol.enum_task_type import TaskType
from suii_mux_manager_comm.task import Task

## ===== RefBoxConverter ===== ##
# Converts from Refbox's ROS to our objects
class RefBoxConverter:
    @staticmethod
    def transportation_task_to_task(task):
        source = TaskProtocol.look_up_value(TaskProtocol.location_dict, task.transportation_task.source.description.data)
        if (source == -1):
            print("Error look up for: %s" % task.transportation_task.source.description.data)
            print("Please add to suii_protocol/protocol/enum_location_identifier")
            print("And specify if you want instance ids generated in suii_protocol/task_protocol.py")
            return None
        
        destination = TaskProtocol.look_up_value(TaskProtocol.location_dict, task.transportation_task.destination.description.data)
        if (destination == -1):
            print("Error look up for: %s" % task.transportation_task.destination.description.data)
            print("Please add to suii_protocol/protocol/enum_location_identifier")
            print("And specify if you want instance ids generated in suii_protocol/task_protocol.py")
            return None
        
        object_to_pick = TaskProtocol.look_up_value(TaskProtocol.object_dict, task.transportation_task.object.description.data)
        if (object_to_pick == -1):
            print("Error look up for: %s" % task.transportation_task.object.description.data)
            print("Please add to suii_protocol/protocol/enum_object_identifier")
            return None
            
        container = -1
        if (task.transportation_task.container.description.data != ""):
            container = TaskProtocol.look_up_value(TaskProtocol.container_dict, task.transportation_task.container.description.data)
            if (container == -1):
                print("Error look up for: %s" % task.transportation_task.container.description.data)
                print("Please add to suii_protocol/protocol/enum_object_identifier")
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
        dest_str = task.navigation_task.location.description.data
        dest_str = dest_str.replace("Waypoint", "Way Point") # We don't use the same string for SOME reason :)

        destination = TaskProtocol.look_up_value(TaskProtocol.location_dict, dest_str)
        if (destination == -1):
            print("Converter: Destination %s not found" % dest_str)
            print("Please add to suii_protocol/protocol/enum_location_identifier")
            print("And specify if you want instance ids generated in suii_protocol/task_protocol.py")
            return None
        tmp_task = Task() 
        tmp_task.set_type (TaskProtocol.look_up_value(TaskProtocol.task_type_dict, TaskType.NAVIGATION.fullname)) 
        tmp_task.set_source(-1) 
        tmp_task.set_destination(destination)
        tmp_task.set_object(-1)
        tmp_task.set_container(-1)
        return tmp_task
    
    @staticmethod
    def navigation_task_list_to_task_list(nav_task_list):
        result = TaskList()
        for item in nav_task_list:
            converted_task = RefBoxConverter.navigation_task_to_task(item)
            result.task_list.append(converted_task)
        return result

    @staticmethod
    def transportation_task_list_to_task_list(trans_task_list):
        result = TaskList()
        for item in trans_task_list:
            converted_task = RefBoxConverter.transportation_task_to_task(item)
            result.task_list.append(converted_task)
        return result

    @staticmethod
    def ros_msg_to_task_list_object (msg):
        tasks = msg.tasks
        result = TaskList()
        for task in tasks:
            if task.type.data == int(TaskType.TRANSPORTATION):
                result.task_list.append(RefBoxConverter.transportation_task_to_task(task))
            elif task.type.data == int(TaskType.NAVIGATION):
                result.task_list.append(RefBoxConverter.navigation_task_to_task(task))
            else:
                return None 
        return result