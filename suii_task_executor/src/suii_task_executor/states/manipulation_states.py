#! /usr/bin/env python
from zinki_smachine import *


class ObjectGraspState(State):
    def __init__(self, fsm):
         super(ObjectGraspState, self).__init__(fsm)


class ObjectPlaceState(State):
    def __init__(self, fsm):
         super(ObjectPlaceState, self).__init__(fsm)