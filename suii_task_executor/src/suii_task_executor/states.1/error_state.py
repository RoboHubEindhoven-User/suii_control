#! /usr/bin/env python
from zinki_smachine import *

class ErrorState(State):
    def __init__(self, fsm):
         super(ErrorState, self).__init__(fsm)