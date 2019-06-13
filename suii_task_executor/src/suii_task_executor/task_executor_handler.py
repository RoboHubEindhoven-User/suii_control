#! /usr/bin/env python

import rospy 
from suii_msgs.msg import SuiiTask, SuiiTaskList

from zinki_smachine import *

from states.idle_state import IdleState
from states.convert_data_state import ConvertDataState
from states.task_select_state import TaskSelectState
from states.task_execution_state import TaskExecutionState
from states.emergency_state import EmergencyState
from states.error_state import ErrorState
from states.state_properties import StateName, TransitionName

from suii_mux_manager_comm.action_list import ActionList
from suii_mux_manager_comm.task_with_action import TaskWithAction 
from suii_mux_manager_comm.task import Task, TaskStatus
from suii_mux_manager_comm.task_list import TaskList

class TaskExecutor:
    def __init__(self):
        self.fsm = StateMachine(name='TaskExecutorStateMachine')
        self.input_sub = rospy.Subscriber('/suii_task_executor/input', SuiiTaskList, self.input_callback)
        self.action_list = ActionList()

        # States
        self.fsm.add_state(StateName.IDLE_STATE,                IdleState(self.fsm))
        self.fsm.add_state(StateName.CONVERT_DATA_STATE,        ConvertDataState(self.fsm))
        self.fsm.add_state(StateName.TASK_SELECT,               TaskSelectState(self.fsm))
        self.fsm.add_state(StateName.TASK_EXECUTION,            TaskExecutionState(self.fsm))
        self.fsm.add_state(StateName.EMERGENCY,                 EmergencyState(self.fsm))
        self.fsm.add_state(StateName.ERROR,                     ErrorState(self.fsm))
        
        # Transitions
        self.fsm.add_transition(TransitionName.IDLE_STATE,              Transition(StateName.IDLE_STATE))
        self.fsm.add_transition(TransitionName.CONVERT_DATA_STATE,      Transition(StateName.CONVERT_DATA_STATE))
        self.fsm.add_transition(TransitionName.TASK_SELECT,             Transition(StateName.TASK_SELECT))   
        self.fsm.add_transition(TransitionName.TASK_EXECUTION,          Transition(StateName.TASK_EXECUTION))
        self.fsm.add_transition(TransitionName.EMERGENCY,               Transition(StateName.EMERGENCY))
        self.fsm.add_transition(TransitionName.ERROR,                   Transition(StateName.ERROR))

        # set default state
        self.fsm.set_state(StateName.IDLE_STATE)
        rospy.loginfo('Handler initialized!')

    def input_callback (self, msg):
        rospy.loginfo("new input received")
        self.fsm.states[StateName.CONVERT_DATA_STATE].buffer = msg
        self.fsm.transition(TransitionName.CONVERT_DATA_STATE)
        rospy.loginfo("New task list to buffer!") 

    def execute(self):
        self.fsm.execute()

if __name__ == '__main__':
    rospy.init_node('suii_task_executor')
    te = TaskExecutor()
    while not rospy.is_shutdown():
        te.execute()