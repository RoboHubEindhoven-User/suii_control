#!/usr/bin/env python

'''bmt_state.py: 
'''

__author__      = "Zinkeng Thierry"
from yaml import dump

from suii_control.state_machines import *

class BMT_StateName(Enum):
    TASK = "task_state"
    MOVE = "move_state"
    WAIT = "wait_state"
    EXIT = "exit_state"
    EMRG = "emergency_state" 

class BMT_TransTo(Enum):
    TASK = "to_task"
    MOVE = "to_move"
    WAIT = "to_wait"
    EXIT = "to_exit"
    EMRG = "to_emergency" 

class TaskState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(TaskState, self).__init__(fsm)
        
    def enter(self):
        super(TaskState, self).enter()
    
    def execute(self):
        super(TaskState, self).execute()
        self.fsm.transition(BMT_TransTo.MOVE)

    def exit(self):
        super(TaskState, self).exit()

class MoveState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(MoveState, self).__init__(fsm)
        
    def enter(self):
        super(MoveState, self).enter()
    
    def execute(self):
        super(MoveState, self).execute()
        self.fsm.transition(BMT_TransTo.WAIT)

    def exit(self):
        super(MoveState, self).exit()

    
class WaitState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(WaitState, self).__init__(fsm)
        
    def enter(self):
        super(WaitState, self).enter()
    
    def execute(self):
        super(WaitState, self).execute()
        self.fsm.transition(BMT_TransTo.EXIT)

    def exit(self):
        super(WaitState, self).exit()

class ExitState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(ExitState, self).__init__(fsm)
        
    def enter(self):
        super(ExitState, self).enter()
    
    def execute(self):
        super(ExitState, self).execute()
        self.fsm.transition(BMT_TransTo.MOVE)

    def exit(self):
        super(ExitState, self).exit()

class EmergencyStopState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm):
        super(EmergencyStopState, self).__init__(fsm)
        
    def enter(self):
        super(EmergencyStopState, self).enter()
    
    def execute(self):
        super(EmergencyStopState, self).execute()
        self.fsm.transition(BMT_TransTo.MOVE)

    def exit(self):
        super(EmergencyStopState, self).exit()


#********************************************************************************************

class BasicManipulationState(State):
    ''' Implements the action and or guard of a state'''
    def __init__(self, fsm, refbox):
        super(BasicManipulationState, self).__init__(fsm)
        
        self.refbox = refbox
        self.sub_fsm = StateMachine(self);
        self.rate = rospy.Rate(10)
        
        # States
        self.bnt_state = TaskState(self.sub_fsm)
        self.sub_fsm.add_state(BMT_StateName.TASK, self.bnt_state)
        self.sub_fsm.add_state(BMT_StateName.MOVE, MoveState(self.sub_fsm))
        self.sub_fsm.add_state(BMT_StateName.WAIT, WaitState(self.sub_fsm))
        self.sub_fsm.add_state(BMT_StateName.EXIT, ExitState(self.sub_fsm))
        self.sub_fsm.add_state(BMT_StateName.EMRG, EmergencyStopState(self.sub_fsm))

        # Transitions
        self.sub_fsm.add_transition(BMT_TransTo.TASK, Transition(BMT_StateName.TASK))
        self.sub_fsm.add_transition(BMT_TransTo.MOVE, Transition(BMT_StateName.MOVE))
        self.sub_fsm.add_transition(BMT_TransTo.WAIT, Transition(BMT_StateName.WAIT))
        self.sub_fsm.add_transition(BMT_TransTo.EXIT, Transition(BMT_StateName.EXIT))
        self.sub_fsm.add_transition(BMT_TransTo.EMRG, Transition(BMT_StateName.EMRG))
        
        # set default state
        self.sub_fsm.set_state(BMT_StateName.TASK)
        
    def enter(self):
        super(BasicManipulationState, self).enter()
    
    def execute(self):
        super(BasicManipulationState, self).execute()
        condition = False

        task_list = self.refbox.get_transportation_tasks()
        print dump(task_list)
        # while not condition:
        #     self.sub_fsm.execute();
        
        self.fsm.transition(Trans_To.TSL)

    def exit(self):
        super(BasicManipulationState, self).exit()