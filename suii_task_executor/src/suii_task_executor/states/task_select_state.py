#! /usr/bin/env python
from zinki_smachine import *
from suii_mux_manager_comm.action_list import ActionList
from state_properties import StateName, TransitionName
from convert_data_state import ConvertDataState
from task_execution_state import TaskExecutionState

## ===== TaskSelectState ===== ##
# Action: select next task to execute
#           or transition to error

class TaskSelectState(State):
    def __init__(self, fsm):
        super(TaskSelectState, self).__init__(fsm)
        self.list = ActionList()
        self.index = 0
        self.error = False
        self.done = False

    def enter(self):
        # If the previous state was TaskExecution
        if (str(self.fsm.prev_state) == TaskExecutionState.__name__):
            # If we error:
            if self.error:
                print("Task select receives error")
            # If we did not error
            else:
                # If we reached the end of the list
                if (self.index >= len(self.list.task_list) - 1):
                    self.done = True # Then we're done
                # Else: go to the next task
                else:
                    self.index = self.index + 1
        # Else if the previous state was ConvertDataState
        elif (str(self.fsm.prev_state) == ConvertDataState.__name__):
            # Then reset variables because we're beginning a new mux session
            self.index = 0
            self.error = False
            self.done = False
    
    def execute(self):
        # If it didn't error
        if not self.error:
            # If it's not done
            if not self.done:
                # Then go back to TaskExecution
                print("Selecting task @ index %d " % self.index)
                # Pass data down to TaskExecution state
                self.fsm.states[StateName.TASK_EXECUTION].task = self.list.task_list[self.index]
                self.fsm.transition(TransitionName.TASK_EXECUTION)    
            # Else going to Idle    
            else:
                print("We are (in theory) done!")
                self.fsm.transition(TransitionName.IDLE_STATE)
        # If it did error
        else:
            # Passing error data down to Error state
            self.fsm.states[StateName.ERROR].list = self.list 
            self.fsm.states[StateName.ERROR].error_index = self.index
            self.fsm.transition(TransitionName.ERROR)
    