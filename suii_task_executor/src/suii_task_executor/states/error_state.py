#! /usr/bin/env python
from zinki_smachine import State
from state_properties import StateName, TransitionName
import rospy 
from suii_msgs.msg import SuiiTask, SuiiTaskList

from suii_mux_manager_comm.action_list import ActionList
from suii_mux_manager_comm.converter.mux_converter import MuxConverter
from suii_mux_manager_comm.action_list import ActionList
from suii_mux_manager_comm.task_with_action import TaskWithAction
from suii_mux_manager_comm.task import Task, TaskStatus

class ErrorState(State):
    def __init__(self, fsm):
        super(ErrorState, self).__init__(fsm)
        self.list = ActionList()
        self.error_code = -1    # not in use for now, will need later
        self.error_index = -1   # index of the error'd item
        self.error_pub = rospy.Publisher('/suii_task_executor/replan', SuiiTaskList, queue_size=10, latch=True)
        self.error_list = SuiiTaskList

    def execute(self):
        rospy.logerr("Error state: index received=%d" % (self.error_index))
        self.propagate_error()
    
    def propagate_error(self):
        self.error_list = MuxConverter.action_list_to_ros(self.list.task_list, error_index=self.error_index)
        self.error_list.tasks[self.error_index].status = int(TaskStatus.FAILED)
        rospy.loginfo("Publishing list with error...")
        self.error_pub.publish(self.error_list)
        self.fsm.transition(TransitionName.IDLE_STATE)
