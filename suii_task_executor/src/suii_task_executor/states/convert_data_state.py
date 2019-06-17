#! /usr/bin/env python
from suii_mux_manager_comm.converter.mux_converter import MuxConverter
from suii_mux_manager_comm.action_list import ActionList
from suii_mux_manager_comm.task_with_action import TaskWithAction
from suii_mux_manager_comm.task import Task, TaskStatus

from zinki_smachine.state import State
from state_properties import StateName, TransitionName

## ===== ConvertDataState ===== ##
# Action: converts ROS msg to ActionList()
class ConvertDataState(State):
    def __init__(self, fsm):
        super(ConvertDataState, self).__init__(fsm)
        self.buffer = [] # Data buffer from ros  

    def execute(self):
        print('Converting data...')
        self.action_list = MuxConverter.ros_to_action_list(self.buffer)
        # Pass data down to TaskSelect
        self.fsm.states[StateName.TASK_SELECT].list = self.action_list
        self.fsm.transition(TransitionName.TASK_SELECT)