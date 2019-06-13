#! /usr/bin/env python
from zinki_smachine import *
from suii_mux_manager_comm.action_list import ActionList
from state_properties import StateName, TransitionName
from convert_data_state import ConvertDataState
from task_execution_state import TaskExecutionState

class TaskSelectState(State):
    def __init__(self, fsm):
        super(TaskSelectState, self).__init__(fsm)
        self.list = ActionList()
        self.index = 0
        self.error = False
        self.done = False

    def enter(self):
        if (str(self.fsm.prev_state) == TaskExecutionState.__name__):
            if (self.error):
                print("That wasnt very polite of you item index %d" % self.index)
            elif (self.index >= len(self.list.task_list) - 1):
                self.done = True
            else:
                self.index = self.index + 1
        elif (str(self.fsm.prev_state) == ConvertDataState.__name__):
            self.index = 0
            self.error = False
            self.done = False
    
    def execute(self):
        if not self.error:
            if not self.done:
                print("Selecting task @ index %d " % self.index)
                self.fsm.states[StateName.TASK_EXECUTION].task = self.list.task_list[self.index]
                self.fsm.transition(TransitionName.TASK_EXECUTION)        
            else:
                self.fsm.transition(TransitionName.IDLE_STATE)
        else:
            self.fsm.states[StateName.ERROR].list = self.list 
            self.fsm.states[StateName.ERROR].error_index = self.index
            self.fsm.transition(TransitionName.ERROR)
    