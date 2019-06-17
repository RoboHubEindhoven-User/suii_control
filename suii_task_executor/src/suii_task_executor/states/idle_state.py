#! /usr/bin/env python
from zinki_smachine import State

## ===== IdleState ===== ##
# Action: Nothing
class IdleState(State):
    def __init__(self, fsm):
        super(IdleState, self).__init__(fsm)