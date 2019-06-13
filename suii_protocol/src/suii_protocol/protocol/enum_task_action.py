#! /usr/bin/env python
from enum import Enum

class TaskActionType(Enum):   
   PICK              = 1, 'PICK'
   PICK_FROM_ROBOT   = 2, 'PICK_FROM_ROBOT'
   PLACE             = 3, 'PLACE'
   PLACE_TO_ROBOT    = 4, 'PLACE_TO_ROBOT'
   DRIVE             = 5, 'DRIVE'
   MOVE_TO_DRIVE     = 6, 'MOVE_TO_DRIVE'
   FIND_HOLE         = 7, 'FIND_HOLE'

   def __new__(cls, value, name):
        member = object.__new__(cls)
        member._value_ = value
        member.fullname = name
        return member

   def __int__(self):
        return self.value