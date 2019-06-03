#! /usr/bin/env python
from zinki_smachine.state_machine import *
from suii_task_executor.states.idle_states import *
from suii_task_executor.states.task_select_state import *
from suii_task_executor.states.task_execution_state import *
from suii_task_executor.states.emergency_state import *
from suii_task_executor.states.error_state import *
from suii_task_executor.states.state_properties import *

class TaskExecutor:
    def __init__(self, name, age):
        self.fsm = StateMachine()

        # # States
        # self.fsm.add_state(StateName.TASKSELECT, TaskSelectState(self.fsm))
        # self.fsm.add_state(StateName.NAVIGATION, NavigationState(self.fsm))
        # self.fsm.add_state(StateName.GRASP,      ObjectGraspState(self.fsm))
        # self.fsm.add_state(StateName.PLACE,      ObjectPlaceState(self.fsm))
        # self.fsm.add_state(StateName.EMERGENCY,  ErrorState(self.fsm))
        # self.fsm.add_state(StateName.ERROR,      EmergencyState(self.fsm))
        # # Transitions
        # self.fsm.add_transition(TransitionName.TASKSELECT, Transition(StateName.TASKSELECT))
        # self.fsm.add_transition(TransitionName.NAVIGATION, Transition(StateName.NAVIGATION))
        # self.fsm.add_transition(TransitionName.GRASP,      Transition(StateName.GRASP))
        # self.fsm.add_transition(TransitionName.PLACE,      Transition(StateName.PLACE))
        # self.fsm.add_transition(TransitionName.EMERGENCY,  Transition(StateName.EMERGENCY))
        # self.fsm.add_transition(TransitionName.ERROR,      Transition(StateName.ERROR))

        # States
        self.fsm.add_state(StateName.IDLE_STATE,     IdleState(self.fsm))
        self.fsm.add_state(StateName.TASK_SELECT,    TaskSelectState(self.fsm))
        self.fsm.add_state(StateName.TASK_EXECUTION, TaskExecutionState(self.fsm))
        self.fsm.add_state(StateName.EMERGENCY,      ErrorState(self.fsm))
        self.fsm.add_state(StateName.ERROR,          EmergencyState(self.fsm))
        # Transitions
        self.fsm.add_transition(TransitionName.IDLE_STATE,     Transition(StateName.IDLE_STATE))
        self.fsm.add_transition(TransitionName.TASK_SELECT,    Transition(StateName.TASK_SELECT))
        self.fsm.add_transition(TransitionName.TASK_EXECUTION, Transition(StateName.TASK_EXECUTION))
        self.fsm.add_transition(TransitionName.EMERGENCY,      Transition(StateName.EMERGENCY))
        self.fsm.add_transition(TransitionName.ERROR,          Transition(StateName.ERROR))
        
        # set default state
        self.fsm.set_state(StateName.IDLE_STATE)

    def execute(self):
        self.fsm.execute()

