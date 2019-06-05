#! /usr/bin/env python
from zinki_smachine import *

class NavigationState(State):
    def __init__(self, fsm):
         super(NavigationState, self).__init__(fsm)