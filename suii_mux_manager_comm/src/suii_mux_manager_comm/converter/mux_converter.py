from suii_msgs.msg import SuiiTask, SuiiTaskList
from suii_mux_manager_comm.task_list import TaskList
from suii_mux_manager_comm.task import TaskStatus
from suii_mux_manager_comm.task_with_action import TaskWithAction
from suii_mux_manager_comm.action_list import ActionList

class MuxConverter:
    @staticmethod 
    def ros_to_action(ros_msg):
        task = TaskWithAction()
        task.set_action(ros_msg.action)
        task.set_source(ros_msg.source)
        task.set_destination(ros_msg.destination)
        task.set_object(ros_msg.object)
        task.set_container(ros_msg.container)
        task.status = (ros_msg.status)
        task.error_code = ros_msg.error_code
        return task

    @staticmethod
    def ros_to_action_list(ros_msg):
        result = ActionList()
        for item in ros_msg.tasks:
            result.task_list.append(MuxConverter.ros_to_action(item))
        return result

    @staticmethod
    def action_to_ros(task, status):
        ros_task = SuiiTask()
        ros_task.action = task.action
        ros_task.action = task.action
        ros_task.source = task.source
        ros_task.destination = task.destination
        ros_task.object = task.object
        ros_task.container = task.container
        ros_task.status = status
        ros_task.error_code = task.error_code
        return ros_task 

    @staticmethod
    def action_list_to_ros(task_list, error_index=-1, status=-1):
        ros_task_list = SuiiTaskList()
        ros_task_list.error_index = error_index
        for item in task_list:
            ros_task_list.tasks.append(MuxConverter.action_to_ros(item, status))
        return ros_task_list