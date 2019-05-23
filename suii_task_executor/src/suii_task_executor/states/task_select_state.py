#! /usr/bin/env python
from zinki_smachine import *

class TaskSelectState(State):
    def __init__(self, fsm):
         super(TaskSelectState, self).__init__(fsm)

    def enter(self):
        super(TaskSelectState, self).enter()
    
    def execute(self):
        super(TaskSelectState, self).execute()

    def exit(self):
        super(TaskSelectState, self).exit()
