#! /usr/bin/env python
from zinki_smachine import *

class TaskExecutionState(State):
    def __init__(self, fsm):
         super(TaskExecutionState, self).__init__(fsm)