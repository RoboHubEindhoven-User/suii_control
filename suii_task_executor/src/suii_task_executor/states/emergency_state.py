#! /usr/bin/env python
from zinki_smachine import *

class EmergencyState(State):
    def __init__(self, fsm):
         super(EmergencyState, self).__init__(fsm)