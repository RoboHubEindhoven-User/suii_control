#! /usr/bin/env python
from enum import Enum

class TaskType(Enum):
   TRANSPORTATION = 1, 'TRANSPORTATION'
   NAVIGATION = 2, 'NAVIGATION'

   def __new__(cls, value, name):
        member = object.__new__(cls)
        member._value_ = value
        member.fullname = name
        return member

   def __int__(self):
        return self.value